#include "test_check.h"
#include "navigationTack.h"
#include "navigationGeo.h"
#include "sharedSensor.h"

#include <math.h>
#include <string.h>

// Leg 0 starts here in every test, so positions are given in meters north and east of it
#define LAT0  40.7608
#define LON0 -111.8910
#define ROUTE_VERSION 7u

static int approx(double a, double b, double tol) { return fabs(a - b) <= tol; }
static int angleApprox(double a, double b, double tol) { return fabs(navGeo_wrap180(a - b)) <= tol; }

static double lat_of(double north) { return LAT0 + north / NAV_M_PER_DEG_LAT; }
static double lon_of(double east)  { return LON0 + east / (NAV_M_PER_DEG_LAT * cos(LAT0 * NAV_DEG2RAD)); }

// One waypoint `north` meters up the leg, 20 m margin unless a test changes it
static NavRoute_t route_north(double north, uint16_t margin_m)
{
    NavRoute_t r;
    memset(&r, 0, sizeof r);
    r.count = 1;
    r.arrive_m = 8;
    r.margin_warning_m = 15;
    r.wp[0].lat = lat_of(north);
    r.wp[0].lon = lon_of(0.0);
    r.wp[0].margin_m = margin_m;
    return r;
}

// Healthy inputs: boat at (north, east) on `heading`, wind coming from `twd`
static NavTackInput_t input(uint32_t now, double north, double east, uint16_t heading, double twd, const NavRoute_t *route)
{
    NavTackInput_t in;
    memset(&in, 0, sizeof in);
    in.now_ms = now;
    in.enabled = true;
    in.lat = lat_of(north);
    in.lon = lon_of(east);
    in.heading = heading;
    in.ang_wind = (int16_t)lround(navGeo_wrap180(twd - heading));
    in.gps_valid = VALIDITY_VALID;
    in.imu_valid = VALIDITY_VALID;
    in.wind_valid = VALIDITY_VALID;
    in.gps_timestamp = in.imu_timestamp = in.wind_timestamp = now;
    in.route_version = (route != NULL) ? ROUTE_VERSION : 0;
    in.route = route;
    return in;
}

// A state already sailing leg 0 of ROUTE_VERSION, with a full wind buffer
static NavTackState_t sailing(uint8_t nav_state, int8_t tack_side, uint32_t tack_ms, double twd)
{
    NavTackState_t s;
    memset(&s, 0, sizeof s);
    s.nav_state = nav_state;
    s.route_version = ROUTE_VERSION;
    s.leg_start_lat = LAT0;
    s.leg_start_lon = LON0;
    s.tack_side = tack_side;
    s.tack_ms = tack_ms;
    for (unsigned i = 0; i < NAV_WIND_WINDOW; i++) { s.wind[i] = twd; }
    s.wind_count = NAV_WIND_WINDOW;
    return s;
}

#define NOW 100000u
#define LONG_AGO (NOW - NAV_TACK_MIN_INTERVAL_MS)

/////////////////////////////////////////////////////////////////////////////////////////

static void tack_inputsOk(void)
{
    NavTackInput_t in = input(NOW, 0, 0, 0, 90, NULL);
    CHECK(inputs_ok(&in));

    in.gps_valid = VALIDITY_VALID_PARTIAL_DATA;     CHECK(inputs_ok(&in));
    in.gps_valid = VALIDITY_VALID_WARMING;          CHECK(!inputs_ok(&in));
    in.gps_valid = VALIDITY_INVALID_GPS_NO_FIX;     CHECK(!inputs_ok(&in));
    in.gps_valid = VALIDITY_VALID;

    in.imu_valid = VALIDITY_VALID_PARTIAL_DATA;     CHECK(!inputs_ok(&in)); // only the GPS may be partial
    in.imu_valid = VALIDITY_VALID;
    in.wind_valid = VALIDITY_VALID_WINDOW_FILLING;  CHECK(!inputs_ok(&in));
    in.wind_valid = VALIDITY_VALID;

    in.wind_timestamp = NOW - NAV_WIND_STALE_MS;        CHECK(inputs_ok(&in));
    in.wind_timestamp = NOW - NAV_WIND_STALE_MS - 1u;   CHECK(!inputs_ok(&in));
    in.wind_timestamp = NOW;
    in.imu_timestamp = NOW - NAV_IMU_STALE_MS;          CHECK(inputs_ok(&in));
    in.imu_timestamp = NOW - NAV_IMU_STALE_MS - 1u;     CHECK(!inputs_ok(&in));
    in.imu_timestamp = NOW;
    in.gps_timestamp = NOW - NAV_GPS_STALE_MS;          CHECK(inputs_ok(&in));
    in.gps_timestamp = NOW - NAV_GPS_STALE_MS - 1u;     CHECK(!inputs_ok(&in));

    // The tick counter wrapped between the sample and now
    in = input(5u, 0, 0, 0, 90, NULL);
    in.gps_timestamp = in.imu_timestamp = in.wind_timestamp = 0xFFFFFFF0u;
    CHECK(inputs_ok(&in));

    // A sample read just after now_ms was taken is fresh, not 49 days old
    in = input(NOW, 0, 0, 0, 90, NULL);
    in.wind_timestamp = NOW + 3u;
    CHECK(inputs_ok(&in));
}



static void tack_windFrom(void)
{
    NavTackInput_t in = input(NOW, 0, 0, 350, 10, NULL);
    CHECK(in.ang_wind == 20);
    CHECK(wind_from(&in) == 10.0);                  // default: apparent wind, wrapped through north

    in.heading = 10; in.ang_wind = -20;
    CHECK(wind_from(&in) == 350.0);

    // Heading north at 2 m/s in a 4 m/s easterly: the vane reads 63 deg, the true wind is 90
    in = input(NOW, 0, 0, 0, 0, NULL);
    in.ang_wind = 63;
    in.sog_mps = 2.0f; in.cog_deg = 0; in.vel_valid = true;
    CHECK(wind_estimate(&in, 0.0) == 63.0);
    CHECK(angleApprox(wind_estimate(&in, 4.0), 90.0, 1.0));

    // The worked case: 3 m/s from the east looks like 71.6 deg from a boat doing 1 m/s north. The vane
    // reports whole degrees, so 71 and 72 land either side of 90
    in.sog_mps = 1.0f;
    in.ang_wind = 72;
    CHECK(angleApprox(wind_estimate(&in, 3.0), 90.0, 0.5));
    CHECK(wind_estimate(&in, 3.0) > 90.0);
    in.ang_wind = 71;
    CHECK(wind_estimate(&in, 3.0) < 90.0);
    CHECK(angleApprox(wind_estimate(&in, 3.0), 90.0, 1.0));
    in.ang_wind = 63;
    in.sog_mps = 2.0f;

    // The velocity is unusable: fall back to apparent
    in.vel_valid = false;                           CHECK(wind_estimate(&in, 4.0) == 63.0);
    in.vel_valid = true; in.sog_mps = 0.2f;         CHECK(wind_estimate(&in, 4.0) == 63.0);
    in.sog_mps = 2.0f; in.cog_deg = 0xFFFFu;        CHECK(wind_estimate(&in, 4.0) == 63.0); // GPS gave no course
    in.cog_deg = 0;

    // Assumed wind too light to produce this apparent wind
    in.ang_wind = 90;
    CHECK(wind_estimate(&in, 1.0) == 90.0);
}



static void tack_windUpdate(void)
{
    NavTackState_t s;
    memset(&s, 0, sizeof s);

    // 350 and 10 average to north, and the ring holds only the newest NAV_WIND_WINDOW samples
    for (unsigned i = 0; i < NAV_WIND_WINDOW + 5u; i++)
    {
        NavTackInput_t in = input(NOW, 0, 0, 0, (i % 2u) ? 10.0 : 350.0, NULL);
        double twd = wind_update(&in, &s, true);
        if (i == 0) { CHECK(twd == 350.0); CHECK(s.wind_count == 1); }
    }
    CHECK(s.wind_count == NAV_WIND_WINDOW);
    CHECK(s.wind_next == 5);
    NavTackInput_t in = input(NOW, 0, 0, 0, 350, NULL);
    CHECK(angleApprox(wind_update(&in, &s, true), 0.0, 1.5));

    // Nothing is added on bad inputs or during a tack, the mean stays as it was
    s = sailing(NAV_STATE_DIRECT, 0, LONG_AGO, 90);
    in = input(NOW, 0, 0, 0, 200, NULL);
    CHECK(angleApprox(wind_update(&in, &s, false), 90.0, 1e-9));
    CHECK(s.wind_next == 0);
    s.nav_state = NAV_STATE_TACKING;
    CHECK(angleApprox(wind_update(&in, &s, true), 90.0, 1e-9));
    CHECK(s.wind_next == 0);
    s.nav_state = NAV_STATE_UPWIND_PORT;
    wind_update(&in, &s, true);
    CHECK(s.wind_next == 1);
}



static void tack_legGeometry(void)
{
    NavRoute_t r = route_north(200, 20);
    NavTackState_t s = sailing(NAV_STATE_DIRECT, 0, LONG_AGO, 90);
    NavTackInput_t in = input(NOW, 50, 10, 0, 90, &r);

    NavTackLeg_t leg = leg_geometry(&in, &s, 90.0);
    CHECK(approx(leg.xte, 10.0, 0.01));             // east of a northbound leg = starboard
    CHECK(approx(leg.along, 50.0, 0.01));
    CHECK(approx(leg.leg_len, 200.0, 0.01));
    CHECK(angleApprox(leg.leg_brg, 0.0, 0.01));
    CHECK(angleApprox(leg.brg_wp, 356.19, 0.01));
    CHECK(approx(leg.rel_wp, -93.81, 0.01));
    CHECK(approx(leg.dist_wp, 150.33, 0.01));
    CHECK(leg.margin == 20.0);
}



static void tack_directHeading(void)
{
    NavTackLeg_t leg = { .xte = 0, .leg_brg = 0, .rel_wp = -90, .margin = 20 };
    CHECK(angleApprox(direct_heading(&leg, 90.0), 0.0, 1e-9));

    leg.xte = 10;   CHECK(angleApprox(direct_heading(&leg, 90.0), 337.5, 1e-9)); // starboard of the line: steer to port
    leg.xte = -10;  CHECK(angleApprox(direct_heading(&leg, 90.0), 22.5, 1e-9));
    leg.xte = 20;   CHECK(angleApprox(direct_heading(&leg, 90.0), 315.0, 1e-9));
    leg.xte = 500;  CHECK(angleApprox(direct_heading(&leg, 90.0), 315.0, 1e-9)); // clamped at the margin
    leg.xte = -500; CHECK(angleApprox(direct_heading(&leg, 90.0), 45.0, 1e-9));

    // Wind from 50: the return heading of 45 is in the no-go zone, so stop at its edge on the waypoint's side
    leg.xte = -20; leg.rel_wp = -50;
    CHECK(angleApprox(direct_heading(&leg, 50.0), 5.0, 1e-9));
    leg.rel_wp = 50; // waypoint the other side of the wind
    CHECK(angleApprox(direct_heading(&leg, 50.0), 95.0, 1e-9));

    leg.margin = 0; leg.xte = 5; leg.rel_wp = -90; // no margin: no return angle, and no divide by zero
    CHECK(angleApprox(direct_heading(&leg, 90.0), 0.0, 1e-9));
}

/////////////////////////////////////////////////////////////////////////////////////////

static void tack_idle(void)
{
    NavRoute_t r = route_north(200, 20);
    NavTackState_t s = sailing(NAV_STATE_UPWIND_PORT, -1, LONG_AGO, 0);
    TelemetryNavigation_t out;
    memset(&out, 0xAA, sizeof out);

    // Disabled: everything at its default, and the stored wind is dropped
    NavTackInput_t in = input(NOW, 50, 0, 45, 0, &r);
    in.enabled = false;
    navTack_step(&in, &s, &out);
    CHECK(out.nav_state == NAV_STATE_IDLE);
    CHECK(out.desired_heading == NAV_HEADING_NONE);
    CHECK(out.route_segment == NAV_SEGMENT_NONE);
    CHECK(out.drift_distance == 0);
    CHECK(out.flags == 0);
    CHECK(out.tack_side == 0);
    CHECK(out.timestamp == NOW);
    CHECK(out.valid == VALIDITY_VALID);
    CHECK(s.wind_count == 0);

    // Enabled again: waits for wind, then carries on with the same leg and picks its tack again
    for (unsigned i = 0; i < NAV_WIND_MIN_SAMPLES; i++)
    {
        in = input(NOW + 500u * (i + 1u), 50, 0, 45, 0, &r);
        navTack_step(&in, &s, &out);
        if (i < NAV_WIND_MIN_SAMPLES - 1u) { CHECK(out.nav_state == NAV_STATE_FAULT); }
    }
    CHECK(out.nav_state == NAV_STATE_UPWIND_PORT);
    CHECK(out.desired_heading == 45);
    CHECK(s.route_version == ROUTE_VERSION);

    // No route: idle, but the wind is still sampled
    memset(&s, 0, sizeof s);
    in = input(NOW, 0, 0, 0, 90, NULL);
    navTack_step(&in, &s, &out);
    CHECK(out.nav_state == NAV_STATE_IDLE);
    CHECK(s.wind_count == 1);

    // A version without a route to go with it
    in.route_version = ROUTE_VERSION;
    navTack_step(&in, &s, &out);
    CHECK(out.nav_state == NAV_STATE_IDLE);
    r.count = 0; in.route = &r;
    navTack_step(&in, &s, &out);
    CHECK(out.nav_state == NAV_STATE_IDLE);
}



static void tack_newRoute(void)
{
    NavRoute_t r = route_north(200, 20);
    NavTackState_t s;
    memset(&s, 0, sizeof s);
    TelemetryNavigation_t out;

    // Bad inputs: the route is not taken, and is tried again
    NavTackInput_t in = input(NOW, 30, 40, 0, 90, &r);
    in.gps_valid = VALIDITY_INVALID_GPS_NO_FIX;
    navTack_step(&in, &s, &out);
    CHECK(out.nav_state == NAV_STATE_FAULT);
    CHECK(s.route_version == 0);

    // Taken on the first good cycle, from where the boat is. FAULT until there is enough wind
    in = input(NOW + 500u, 30, 40, 0, 90, &r);
    navTack_step(&in, &s, &out);
    CHECK(out.nav_state == NAV_STATE_FAULT);
    CHECK(out.desired_heading == NAV_HEADING_NONE);
    CHECK(s.route_version == ROUTE_VERSION);
    CHECK(s.leg == 0);
    CHECK(s.leg_start_lat == in.lat && s.leg_start_lon == in.lon);
    CHECK(s.tack_ms == NOW + 500u - NAV_TACK_MIN_INTERVAL_MS);

    for (unsigned i = 2; i <= NAV_WIND_MIN_SAMPLES; i++)
    {
        in = input(NOW + 500u * i, 30, 40, 0, 90, &r);
        navTack_step(&in, &s, &out);
        if (i < NAV_WIND_MIN_SAMPLES) { CHECK(out.nav_state == NAV_STATE_FAULT); }
    }

    // Beam reach to a waypoint 170 m north and 40 m west of the start
    CHECK(out.nav_state == NAV_STATE_DIRECT);
    CHECK(out.desired_heading == 347);              // atan2(-40, 170) = -13.2 deg
    CHECK(out.route_segment == 0);
    CHECK(out.drift_distance == 0);
    CHECK(out.tack_side == 0);
}



static void tack_direct(void)
{
    NavRoute_t r = route_north(200, 20);
    TelemetryNavigation_t out;

    // On the line
    NavTackState_t s = sailing(NAV_STATE_DIRECT, 0, LONG_AGO, 90);
    NavTackInput_t in = input(NOW, 50, 0, 0, 90, &r);
    navTack_step(&in, &s, &out);
    CHECK(out.nav_state == NAV_STATE_DIRECT);
    CHECK(out.desired_heading == 0);

    // At the margin to starboard: full return angle to port
    in = input(NOW, 50, 20, 0, 90, &r);
    navTack_step(&in, &s, &out);
    CHECK(out.nav_state == NAV_STATE_DIRECT);
    CHECK(out.desired_heading == 315);
    CHECK(out.drift_distance == 20);
    CHECK(out.route_segment == 0);

    // 10 m to starboard with a 20 m margin: half the return angle, 337.5
    in = input(NOW, 50, 10, 0, 90, &r);
    navTack_step(&in, &s, &out);
    CHECK(out.nav_state == NAV_STATE_DIRECT);
    CHECK(out.desired_heading == 337 || out.desired_heading == 338);
    CHECK(out.drift_distance == 10);

    // Wind from 55 with the boat at the port margin. The waypoint is 47 deg off the wind, so it is
    // sailable, but steering 45 back to the line would be in the no-go zone: held at its edge on the
    // waypoint's side
    s = sailing(NAV_STATE_DIRECT, 0, LONG_AGO, 55);
    in = input(NOW, 50, -20, 0, 55, &r);
    navTack_step(&in, &s, &out);
    CHECK(out.nav_state == NAV_STATE_DIRECT);
    CHECK(out.desired_heading == 10);
    s = sailing(NAV_STATE_DIRECT, 0, LONG_AGO, 90);

    // 359.6 rounds to 360, which must go out as 0
    in = input(NOW, 50, 0.1778, 0, 180, &r);
    navTack_step(&in, &s, &out);
    CHECK(out.desired_heading == 0);
}



static void tack_upwindEntry(void)
{
    NavRoute_t r = route_north(200, 20);
    TelemetryNavigation_t out;

    // Dead upwind: take the tack closer to where the bow already points
    NavTackState_t s = sailing(NAV_STATE_DIRECT, 0, LONG_AGO, 0);
    NavTackInput_t in = input(NOW, 0, 0, 40, 0, &r);
    navTack_step(&in, &s, &out);
    CHECK(out.nav_state == NAV_STATE_UPWIND_PORT);
    CHECK(out.desired_heading == 45);
    CHECK(out.tack_side == -1);
    CHECK(s.tack_ms == LONG_AGO);                   // not a tack

    s = sailing(NAV_STATE_DIRECT, 0, LONG_AGO, 0);
    in = input(NOW, 0, 0, 320, 0, &r);
    navTack_step(&in, &s, &out);
    CHECK(out.nav_state == NAV_STATE_UPWIND_STBD);
    CHECK(out.desired_heading == 315);
    CHECK(out.tack_side == 1);

    // Already at the starboard edge: the closer tack heads out of the corridor, so tack onto the other
    s = sailing(NAV_STATE_DIRECT, 0, LONG_AGO, 0);
    in = input(NOW, 50, 18, 40, 0, &r);
    navTack_step(&in, &s, &out);
    CHECK(out.nav_state == NAV_STATE_TACKING);
    CHECK(out.desired_heading == 315);
    CHECK(out.tack_side == 1);
    CHECK(s.tack_ms == NOW);

    // Same spot, but the closer tack heads back in
    s = sailing(NAV_STATE_DIRECT, 0, LONG_AGO, 0);
    in = input(NOW, 50, 18, 320, 0, &r);
    navTack_step(&in, &s, &out);
    CHECK(out.nav_state == NAV_STATE_UPWIND_STBD);
}



static void tack_corridor(void)
{
    NavRoute_t r = route_north(200, 20);
    TelemetryNavigation_t out;

    // Port tack heads east. Just short of 0.8 x margin: carry on
    NavTackState_t s = sailing(NAV_STATE_UPWIND_PORT, -1, LONG_AGO, 0);
    NavTackInput_t in = input(NOW, 50, 15.9, 45, 0, &r);
    navTack_step(&in, &s, &out);
    CHECK(out.nav_state == NAV_STATE_UPWIND_PORT);
    CHECK(out.desired_heading == 45);

    // 0.7 x margin: no tack
    in = input(NOW, 50, 14, 45, 0, &r);
    navTack_step(&in, &s, &out);
    CHECK(out.nav_state == NAV_STATE_UPWIND_PORT);

    // Past 0.8 x margin: tack
    in = input(NOW, 51, 16.5, 45, 0, &r);
    navTack_step(&in, &s, &out);
    CHECK(out.nav_state == NAV_STATE_TACKING);
    CHECK(out.desired_heading == 315);
    CHECK(out.tack_side == 1);
    CHECK(s.tack_ms == NOW);

    // On starboard tack at the starboard edge the boat is heading back in: no tack
    s = sailing(NAV_STATE_UPWIND_STBD, 1, LONG_AGO, 0);
    in = input(NOW, 50, 17, 315, 0, &r);
    navTack_step(&in, &s, &out);
    CHECK(out.nav_state == NAV_STATE_UPWIND_STBD);
    CHECK(out.desired_heading == 315);

    // Same at the margin itself
    in = input(NOW, 50, 20, 315, 0, &r);
    navTack_step(&in, &s, &out);
    CHECK(out.nav_state == NAV_STATE_UPWIND_STBD);

    // At the port edge heading out, but the last tack was 5 s ago
    s = sailing(NAV_STATE_UPWIND_STBD, 1, NOW - 5000u, 0);
    in = input(NOW, 50, -17, 315, 0, &r);
    navTack_step(&in, &s, &out);
    CHECK(out.nav_state == NAV_STATE_UPWIND_STBD);
    CHECK(s.tack_ms == NOW - 5000u);

    // One ms short of the interval, then exactly on it
    s.tack_ms = LONG_AGO + 1u;
    navTack_step(&in, &s, &out);
    CHECK(out.nav_state == NAV_STATE_UPWIND_STBD);
    s.tack_ms = LONG_AGO;
    navTack_step(&in, &s, &out);
    CHECK(out.nav_state == NAV_STATE_TACKING);
    CHECK(out.desired_heading == 45);
    CHECK(out.tack_side == -1);
}



static void tack_tacking(void)
{
    NavRoute_t r = route_north(200, 20);
    TelemetryNavigation_t out;

    // Still on the old heading: hold the new tack's heading
    NavTackState_t s = sailing(NAV_STATE_TACKING, 1, NOW - 2000u, 0);
    NavTackInput_t in = input(NOW, 50, 17, 45, 0, &r);
    navTack_step(&in, &s, &out);
    CHECK(out.nav_state == NAV_STATE_TACKING);
    CHECK(out.desired_heading == 315);
    CHECK(out.flags == 0);
    CHECK(s.wind_next == 0);                        // no wind sample during the tack

    // The vane swings during the turn, the target does not follow it
    in = input(NOW, 50, 17, 10, 30, &r);
    navTack_step(&in, &s, &out);
    CHECK(out.desired_heading == 315);

    // 21 deg off is not done, 20 is
    in = input(NOW, 50, 17, 336, 0, &r);
    navTack_step(&in, &s, &out);
    CHECK(out.nav_state == NAV_STATE_TACKING);
    in = input(NOW, 50, 17, 335, 0, &r);
    navTack_step(&in, &s, &out);
    CHECK(out.nav_state == NAV_STATE_UPWIND_STBD);
    CHECK(out.desired_heading == 315);
    CHECK(out.tack_side == 1);

    // The corridor trigger does not fire mid-tack, even this far out on the old heading
    s = sailing(NAV_STATE_TACKING, 1, LONG_AGO, 0);
    in = input(NOW, 50, 19, 45, 0, &r);
    navTack_step(&in, &s, &out);
    CHECK(out.nav_state == NAV_STATE_TACKING);
    CHECK(out.tack_side == 1);

    // Slow tack: flagged only after NAV_TACK_SLOW_MS, and it keeps going
    s = sailing(NAV_STATE_TACKING, 1, NOW - NAV_TACK_SLOW_MS, 0);
    in = input(NOW, 50, 17, 45, 0, &r);
    navTack_step(&in, &s, &out);
    CHECK(out.flags == 0);
    s.tack_ms = NOW - NAV_TACK_SLOW_MS - 1u;
    navTack_step(&in, &s, &out);
    CHECK(out.flags == NAV_FLAG_TACK_SLOW);
    CHECK(out.nav_state == NAV_STATE_TACKING);
    CHECK(out.desired_heading == 315);
}



static void tack_layline(void)
{
    NavRoute_t r = route_north(200, 200); // wide corridor, so only the layline rule is in play
    TelemetryNavigation_t out;

    // Waypoint 47 deg off the wind: inside the hysteresis band, keep beating
    NavTackState_t s = sailing(NAV_STATE_UPWIND_PORT, -1, LONG_AGO, 0);
    NavTackInput_t in = input(NOW, 100, -107.2, 45, 0, &r);
    navTack_step(&in, &s, &out);
    CHECK(out.nav_state == NAV_STATE_UPWIND_PORT);
    CHECK(out.desired_heading == 45);

    // 51 deg off: past the band, leaves upwind
    s = sailing(NAV_STATE_UPWIND_PORT, -1, LONG_AGO, 0);
    in = input(NOW, 100, -123.5, 45, 0, &r);
    navTack_step(&in, &s, &out);
    CHECK(out.nav_state == NAV_STATE_DIRECT);
    s = sailing(NAV_STATE_UPWIND_PORT, -1, LONG_AGO, 0);

    // 56 deg off, on this tack's side: bear away, no turn through the wind
    in = input(NOW, 100, -150, 45, 0, &r);
    navTack_step(&in, &s, &out);
    CHECK(out.nav_state == NAV_STATE_DIRECT);
    CHECK(out.tack_side == 0);
    CHECK(out.desired_heading == 45);               // 33.75 back to the line is in the no-go zone, held at its edge

    // 56 deg off on the other side, too soon after the last tack: keep this tack
    s = sailing(NAV_STATE_UPWIND_PORT, -1, NOW - 1000u, 0);
    in = input(NOW, 100, 150, 45, 0, &r);
    navTack_step(&in, &s, &out);
    CHECK(out.nav_state == NAV_STATE_UPWIND_PORT);
    CHECK(out.desired_heading == 45);

    // Once allowed: tack onto the waypoint's side
    s.tack_ms = LONG_AGO;
    navTack_step(&in, &s, &out);
    CHECK(out.nav_state == NAV_STATE_TACKING);
    CHECK(out.tack_side == 1);
    CHECK(out.desired_heading == 315);
    CHECK(s.tack_ms == NOW);

    // Tack done, then DIRECT on the cycle after
    in = input(NOW + 500u, 100, 150, 320, 0, &r);
    navTack_step(&in, &s, &out);
    CHECK(out.nav_state == NAV_STATE_UPWIND_STBD);
    in = input(NOW + 1000u, 100, 150, 315, 0, &r);
    navTack_step(&in, &s, &out);
    CHECK(out.nav_state == NAV_STATE_DIRECT);
    CHECK(out.desired_heading == 315);
}



static void tack_arrival(void)
{
    TelemetryNavigation_t out;

    // Two legs: 200 m north, then 200 m east. Wind from the south, so both are downwind or a reach
    NavRoute_t r = route_north(200, 20);
    r.count = 2;
    r.wp[1].lat = lat_of(200);
    r.wp[1].lon = lon_of(200);
    r.wp[1].margin_m = 20;

    // 9 m short: still leg 0
    NavTackState_t s = sailing(NAV_STATE_DIRECT, 0, LONG_AGO, 180);
    NavTackInput_t in = input(NOW, 191, 0, 0, 180, &r);
    navTack_step(&in, &s, &out);
    CHECK(out.route_segment == 0);
    CHECK(s.leg == 0);

    // Inside the arrival radius: on to leg 1, which starts at the waypoint just reached
    in = input(NOW, 195, 0, 0, 180, &r);
    navTack_step(&in, &s, &out);
    CHECK(s.leg == 1);
    CHECK(s.leg_start_lat == r.wp[0].lat && s.leg_start_lon == r.wp[0].lon);
    CHECK(out.nav_state == NAV_STATE_DIRECT);
    CHECK(out.route_segment == 1);
    CHECK(out.drift_distance == 5);                 // 5 m south of an eastbound leg = starboard
    CHECK(out.desired_heading == 79);               // 90 - 45 * 5/20

    // A tack in progress ends with the leg
    s = sailing(NAV_STATE_TACKING, 1, NOW, 180);
    navTack_step(&in, &s, &out);
    CHECK(out.nav_state == NAV_STATE_DIRECT);
    CHECK(out.tack_side == 0);

    // Last waypoint: ARRIVED, and it stays that way
    in = input(NOW, 200, 196, 90, 180, &r);
    navTack_step(&in, &s, &out);
    CHECK(out.nav_state == NAV_STATE_ARRIVED);
    CHECK(out.desired_heading == NAV_HEADING_NONE);
    CHECK(out.route_segment == NAV_SEGMENT_NONE);
    in = input(NOW + 500u, 150, 150, 90, 180, &r);
    navTack_step(&in, &s, &out);
    CHECK(out.nav_state == NAV_STATE_ARRIVED);

    // Bad inputs show as FAULT, then it is ARRIVED again, not sailing
    in.wind_valid = VALIDITY_INVALID_WIND;
    navTack_step(&in, &s, &out);
    CHECK(out.nav_state == NAV_STATE_FAULT);
    in.wind_valid = VALIDITY_VALID;
    navTack_step(&in, &s, &out);
    CHECK(out.nav_state == NAV_STATE_ARRIVED);

    // Sailed past the waypoint outside the arrival radius, inside the margin: reached
    r = route_north(200, 20);
    s = sailing(NAV_STATE_DIRECT, 0, LONG_AGO, 180);
    in = input(NOW, 201, 10, 0, 180, &r);
    navTack_step(&in, &s, &out);
    CHECK(out.nav_state == NAV_STATE_ARRIVED);

    // Past it but outside the margin: not reached, steer back
    s = sailing(NAV_STATE_DIRECT, 0, LONG_AGO, 180);
    in = input(NOW, 201, 25, 0, 180, &r);
    navTack_step(&in, &s, &out);
    CHECK(out.nav_state == NAV_STATE_DIRECT);
    CHECK(out.desired_heading == 315);

    // A new route while on leg 1 of the old one: back to leg 0, starting from the boat
    r.count = 2;
    r.wp[1].lat = lat_of(200);
    r.wp[1].lon = lon_of(200);
    r.wp[1].margin_m = 20;
    s = sailing(NAV_STATE_UPWIND_STBD, 1, NOW - 1000u, 180);
    s.leg = 1;
    s.leg_start_lat = r.wp[0].lat;
    s.leg_start_lon = r.wp[0].lon;
    in = input(NOW, 120, 60, 0, 180, &r);
    in.route_version = ROUTE_VERSION + 1u;
    navTack_step(&in, &s, &out);
    CHECK(s.route_version == ROUTE_VERSION + 1u);
    CHECK(s.leg == 0);
    CHECK(s.leg_start_lat == in.lat && s.leg_start_lon == in.lon);
    CHECK(s.tack_ms == NOW - NAV_TACK_MIN_INTERVAL_MS);
    CHECK(out.nav_state == NAV_STATE_DIRECT);
    CHECK(out.route_segment == 0);
    CHECK(out.drift_distance == 0);
    CHECK(out.desired_heading == 323);              // atan2(-60, 80) = -36.9 deg
    r = route_north(200, 20);

    // Both waypoints inside the arrival radius at once
    r.count = 2;
    r.wp[1].lat = lat_of(203);
    r.wp[1].lon = lon_of(0);
    r.wp[1].margin_m = 20;
    s = sailing(NAV_STATE_DIRECT, 0, LONG_AGO, 180);
    in = input(NOW, 199, 0, 0, 180, &r);
    navTack_step(&in, &s, &out);
    CHECK(out.nav_state == NAV_STATE_ARRIVED);
    CHECK(s.leg == 2);
}



static void tack_faults(void)
{
    NavRoute_t r = route_north(200, 20);
    TelemetryNavigation_t out;

    // Exactly margin + warning is still inside
    NavTackState_t s = sailing(NAV_STATE_DIRECT, 0, LONG_AGO, 90);
    NavTackInput_t in = input(NOW, 50, 35, 0, 90, &r);
    navTack_step(&in, &s, &out);
    CHECK(out.nav_state == NAV_STATE_DIRECT);
    CHECK(!s.corridor_fault);

    // Beyond it: FAULT with the drift still reported
    in = input(NOW, 50, 36, 0, 90, &r);
    navTack_step(&in, &s, &out);
    CHECK(out.nav_state == NAV_STATE_FAULT);
    CHECK(out.desired_heading == NAV_HEADING_NONE);
    CHECK(out.drift_distance == 36);
    CHECK(s.corridor_fault);

    // It holds after the boat is back on the line
    in = input(NOW + 500u, 60, 0, 0, 90, &r);
    navTack_step(&in, &s, &out);
    CHECK(out.nav_state == NAV_STATE_FAULT);

    // Only a new route clears it
    in.route_version = ROUTE_VERSION + 1u;
    navTack_step(&in, &s, &out);
    CHECK(out.nav_state == NAV_STATE_DIRECT);
    CHECK(!s.corridor_fault);
    CHECK(s.leg_start_lat == in.lat);

    // Stale GPS: FAULT, and planning resumes when it is fresh again
    s = sailing(NAV_STATE_DIRECT, 0, LONG_AGO, 90);
    in = input(NOW, 50, 0, 0, 90, &r);
    in.gps_timestamp = NOW - NAV_GPS_STALE_MS - 1u;
    navTack_step(&in, &s, &out);
    CHECK(out.nav_state == NAV_STATE_FAULT);
    CHECK(out.desired_heading == NAV_HEADING_NONE);
    in = input(NOW + 500u, 50, 0, 0, 90, &r);
    navTack_step(&in, &s, &out);
    CHECK(out.nav_state == NAV_STATE_DIRECT);
    CHECK(out.desired_heading == 0);
    CHECK(!s.corridor_fault);

    // A stale vane for one cycle does not latch: the same tack is picked up again
    s = sailing(NAV_STATE_UPWIND_PORT, -1, LONG_AGO, 0);
    in = input(NOW, 50, 5, 45, 0, &r);
    in.wind_timestamp = NOW - NAV_WIND_STALE_MS - 1u;
    navTack_step(&in, &s, &out);
    CHECK(out.nav_state == NAV_STATE_FAULT);
    CHECK(out.tack_side == 0);
    in = input(NOW + 500u, 50, 5, 45, 0, &r);
    navTack_step(&in, &s, &out);
    CHECK(out.nav_state == NAV_STATE_UPWIND_PORT);
    CHECK(out.desired_heading == 45);
    CHECK(s.tack_ms == LONG_AGO);
}



// Closes the loop: the boat moves along the heading LTP asks for, into a steady northerly, to a
// waypoint 300 m dead upwind. It has to get there by tacking, without leaving the corridor.
static void tack_beat(void)
{
    NavRoute_t r = route_north(300, 30);
    NavTackState_t s;
    memset(&s, 0, sizeof s);
    TelemetryNavigation_t out;

    double north = 0, east = 0, heading = 40, max_xte = 0;
    int tacks = 0, arrived = 0;
    uint8_t last_state = NAV_STATE_IDLE;

    for (uint32_t t = 0; t < 1200000u && !arrived; t += 500u)
    {
        NavTackInput_t in = input(NOW + t, north, east, (uint16_t)lround(navGeo_wrap360(heading)) % 360u, 0, &r);
        navTack_step(&in, &s, &out);

        if (out.nav_state == NAV_STATE_TACKING && last_state != NAV_STATE_TACKING) { tacks++; }
        last_state = out.nav_state;
        if (out.nav_state == NAV_STATE_ARRIVED) { arrived = 1; break; }
        if (out.nav_state == NAV_STATE_FAULT && t > 5000u) { break; }
        if (out.desired_heading == NAV_HEADING_NONE) { continue; }

        // Turn at up to 10 deg/s toward the target, 1 m/s through the water, never inside the no-go zone
        double err = navGeo_wrap180(out.desired_heading - heading);
        if (err > 5.0) { err = 5.0; }
        if (err < -5.0) { err = -5.0; }
        heading = navGeo_wrap360(heading + err);
        north += 0.5 * cos(heading * NAV_DEG2RAD);
        east  += 0.5 * sin(heading * NAV_DEG2RAD);
        if (fabs(east) > max_xte) { max_xte = fabs(east); }
        if (out.nav_state == NAV_STATE_UPWIND_PORT || out.nav_state == NAV_STATE_UPWIND_STBD)
        {
            CHECK(fabs(navGeo_wrap180(out.desired_heading - 0.0)) >= NAV_NO_GO_DEG - 0.5);
        }
    }

    CHECK(arrived);
    CHECK(tacks >= 4);
    CHECK(max_xte < 30.0);              // never past the margin
    CHECK(max_xte > 20.0);              // and it used the corridor rather than short-tacking up the middle
    CHECK(!s.corridor_fault);
}

/////////////////////////////////////////////////////////////////////////////////////////

void test_tack(void)
{
    tack_inputsOk();
    tack_windFrom();
    tack_windUpdate();
    tack_legGeometry();
    tack_directHeading();
    tack_idle();
    tack_newRoute();
    tack_direct();
    tack_upwindEntry();
    tack_corridor();
    tack_tacking();
    tack_layline();
    tack_arrival();
    tack_faults();
    tack_beat();
}
