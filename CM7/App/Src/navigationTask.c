#include "navigationTask.h"
#include "navigationTack.h"
#include "navigationRoute.h"
#include "navigationGeo.h"

#include "arbiterRudder.h"
#include "pidHeading.h"
#include "commander.h"
#include "telemetry.h"
#include "sensorGPS.h"
#include "sensorIMU.h"
#include "sensorWind.h"

#include "queue.h"

#include <math.h>

#if defined(CM7_NAVIGATION_ENABLE) && !defined(CM7_COMMANDER_ENABLE)
#error "LTP needs CM7_COMMANDER_ENABLE: without it ARRIVED and corridor faults can't stop the boat"
#endif

#if (SYSTEM_TICK_RATE_HZ % NAV_TACK_TASK_FREQUENCY) != 0u
#warning "NAV_TACK_TASK_FREQUENCY does not divide the tick rate, the period will be rounded down"
#endif

TaskHandle_t navigationTask_task;

static QueueHandle_t telemetry_queue;

// Kept off the task's stack
static NavRoute_t route;            // the route LTP is sailing
static NavRoute_t route_next;       // a new route is copied here first, so a failed copy leaves route alone
static uint32_t route_version;      // version of route, 0 = none yet
static NavTackState_t st;
static TelemetryNavigation_t out;
static MetadataGPS_t gps;
static MetadataIMU_t imu;
static MetadataWind_t wind_meta;

static void gather_inputs(NavTackInput_t *in);
static void update_route(void);
static bool recent(uint32_t now_ms, uint32_t timestamp, uint32_t stale_ms);

#ifdef NAV_TEST_ROUTE
#warning "NAV_TEST_ROUTE is on: the boat loads its own route at the first GPS fix. Bench and sim only, turn it off for the lake"

// north, east metres from the first good fix; margin in metres. An 80 m x 60 m box that ends where it
// started: one leg in each direction, so whatever the wind does, one leg is upwind.
static const struct { int16_t n, e; uint16_t margin; } test_wp[] = {
    { 80, 0, 15 }, { 80, 60, 15 }, { 0, 60, 15 }, { 0, 0, 15 },
};
#define TEST_WP_N (sizeof test_wp / sizeof test_wp[0])
_Static_assert(TEST_WP_N <= NAV_ROUTE_MAX_WAYPOINTS, "the test route has too many waypoints");

// Goes through the Route Manager like an upload from the ground station, so it gets the same checks.
// Normally only the Commander's task calls the Route Manager: this runs once, before any route exists.
static bool load_test_route(double lat0, double lon0)
{
    ipc_cmd_waypoint_t wp[TEST_WP_N];
    double m_per_deg_lon = NAV_M_PER_DEG_LAT * cos(lat0 * NAV_DEG2RAD);
    for (uint16_t i = 0; i < TEST_WP_N; i++) {
        wp[i] = (ipc_cmd_waypoint_t){ .index = i, .margin_m = test_wp[i].margin,
            .lat_e7 = (int32_t)lround((lat0 + test_wp[i].n / NAV_M_PER_DEG_LAT) * 1e7),
            .lon_e7 = (int32_t)lround((lon0 + test_wp[i].e / m_per_deg_lon) * 1e7) };
        navigationRoute_addWaypoint(&wp[i]);
    }
    ipc_cmd_route_commit_t c = { .count = TEST_WP_N, .crc16 = crc16_ccitt((const uint8_t *)wp, sizeof wp),
                                 .arrive_m = 8, .margin_warning_m = 10 };

    // Outside the printf: with the debug off its arguments are never evaluated
    cmd_result_t result = navigationRoute_commit(&c);
    navigationTask_printf("test route: commit result %d", (int)result);
    return result == CMD_RESULT_OK;
}
#endif

RtosInit_e navigationTask_rtosInit(void)
{
    navigationTask_printf("navigationTask_rtosInit start");
    if ((telemetry_queue = xQueueCreate(1, sizeof(TelemetryNavigation_t))) == NULL) { return RTOS_INIT_FAIL; }
    memset(&st, 0, sizeof st); // nav_state IDLE, no route seen
    route_version = 0;
    navigationTask_printf("navigationTask_rtosInit complete");
    return RTOS_INIT_SUCCESS;
}

void navigationTask_handler(void *argument)
{
    navigationTask_printf("navigationTask_handler start");
    TickType_t lastWakeup = xTaskGetTickCount();

    bool heading_sent = false;      // false until a target has been sent in this spell of AUTO
    uint16_t heading_last = 0;
    bool safe_sent = false;         // SAFE already asked for, for the arrival or corridor fault still in effect
    uint8_t state_prev = NAV_STATE_IDLE;
    #ifdef CM7_NAVIGATION_DEBUG
    uint32_t loop_count = 0;
    #endif
    #ifdef NAV_TEST_ROUTE
    bool test_route_loaded = false;
    #endif

    // loop that runs at 2 Hz: read the sensors and the route, plan, send the heading
    for (;;)
    {
        if (!xTaskDelayUntil(&lastWakeup, configTICK_RATE_HZ / NAV_TACK_TASK_FREQUENCY))
        {
            continue;
        }

        NavTackInput_t in;
        gather_inputs(&in);

        #ifdef NAV_TEST_ROUTE
        // Once, around the first good fix, and only if nothing has loaded a route. Fusion reports 0, 0
        // until it has taken its first position, which must not become the corner of the box
        bool gps_ok = (in.gps_valid == VALIDITY_VALID || in.gps_valid == VALIDITY_VALID_PARTIAL_DATA) &&
                      recent(in.now_ms, in.gps_timestamp, NAV_GPS_STALE_MS) && (in.lat != 0.0 || in.lon != 0.0);
        if (!test_route_loaded && navigationRoute_version() == 0 && gps_ok)
        {
            test_route_loaded = load_test_route(in.lat, in.lon); // a failed commit is tried again next cycle
        }
        #endif

        update_route();
        in.route_version = route_version;
        in.route = &route;

        navTack_step(&in, &st, &out);

        if (!in.enabled)
        {
            heading_sent = false; // coming back to AUTO sends a fresh target
        }
        else if (out.desired_heading != NAV_HEADING_NONE)
        {
            int change = abs(ANG_WRAP_180((int)out.desired_heading - (int)heading_last));
            bool tack_started = (out.nav_state == NAV_STATE_TACKING && state_prev != NAV_STATE_TACKING);

            // Setting the target restarts the PID, so small changes are held back
            if (!heading_sent || change >= (int)NAV_HEADING_DEADBAND_DEG || tack_started)
            {
                pidHeading_setTarget(out.desired_heading);
                heading_last = out.desired_heading;
                heading_sent = true;
            }
        }
        state_prev = out.nav_state;

        // Route finished, or outside the corridor: hand the boat to SAFE, once each time that becomes
        // true while in AUTO. Coming back to AUTO with it still true asks again. An input FAULT is
        // left alone, the PID keeps its last target until the inputs come back.
        bool want_safe = in.enabled && (out.nav_state == NAV_STATE_ARRIVED || st.corridor_fault);
        if (want_safe && !safe_sent)
        {
            navigationTask_printf("%s, requesting SAFE", st.corridor_fault ? "outside the corridor" : "route finished");
            commander_requestSafe();
        }
        safe_sent = want_safe;

        xQueueOverwrite(telemetry_queue, &out);

        #ifdef CM7_NAVIGATION_DEBUG
        // print everything once a second
        if (++loop_count % NAV_TACK_TASK_FREQUENCY == 0)
        {
            navigationTask_printf("state %u leg %u xte %d m | wind apparent %d estimated %d | heading %u target %d | %d cm/s%s",
                (unsigned)out.nav_state, (unsigned)st.leg, (int)out.drift_distance,
                (int)in.ang_wind, (int)lround(navGeo_circularMean(st.wind, st.wind_count)),
                (unsigned)in.heading, (out.desired_heading == NAV_HEADING_NONE) ? -1 : (int)out.desired_heading,
                (int)lroundf(in.sog_mps * 100.0f), in.enabled ? "" : " | not in AUTO");
        }
        #endif
    }
}

bool navigationTask_getTelemetry(TelemetryNavigation_t *telemetry)
{
    return (telemetry_queue != NULL) && xQueuePeek(telemetry_queue, telemetry, 0) == pdTRUE;
}

/////////////////////////////////////////////////////////////////////////////////////////

// A sensor whose getter fails is left at valid 0 (VALIDITY_UNKNOWN), which LTP treats as a bad input
static void gather_inputs(NavTackInput_t *in)
{
    memset(in, 0, sizeof *in);
    in->now_ms = HAL_GetTick();
    in->enabled = (arbiterRudder_getMode() == ARBITER_MODE_AUTO);

    if (sensorGPS_getMetadata(&gps))
    {
        in->gps_valid = gps.valid;
        in->gps_timestamp = gps.timestamp;
        in->sog_mps = gps.sog_mps;
        in->cog_deg = gps.cog_deg;
        in->vel_valid = (gps.vel_valid != 0) && recent(in->now_ms, gps.vel_timestamp, NAV_GPS_STALE_MS);
    }
    if (sensorIMU_getMetadata(&imu))
    {
        in->imu_valid = imu.valid;
        in->imu_timestamp = imu.timestamp;
    }
    if (sensorWind_getMetadata(&wind_meta))
    {
        in->wind_valid = wind_meta.valid;
        in->wind_timestamp = wind_meta.timestamp;
    }

    // Position and heading come from fusion. Without them neither sensor is any use to LTP
    TelemetryBoatGlobalOrientation_t pose;
    if (telemetry_getBoatGlobalOrientation(&pose) == TELEMETRY_OK)
    {
        in->lat = pose.latitude;
        in->lon = pose.longitude;
        in->heading = (uint16_t)(pose.angP_heading % 360u);
    }
    else
    {
        in->gps_valid = 0;
        in->imu_valid = 0;
    }

    TelemetryWind_t wind;
    if (telemetry_getWind(&wind) == TELEMETRY_OK) { in->ang_wind = wind.ang_wind; }
    else { in->wind_valid = 0; }
}

// Takes a new route only as a whole: the version must be the same before and after the copy,
// otherwise a commit landed in between and it is tried again next cycle.
static void update_route(void)
{
    uint32_t version = navigationRoute_version();
    if (version == route_version || version == 0) { return; }

    if (navigationRoute_copy(&route_next) && navigationRoute_version() == version)
    {
        route = route_next;
        route_version = version;
        navigationTask_printf("route version %lu, %u waypoints", (unsigned long)version, (unsigned)route.count);
    }
}

// Read as signed, like the checks in navTack_step: a sample stamped just after now_ms is recent
static bool recent(uint32_t now_ms, uint32_t timestamp, uint32_t stale_ms)
{
    return (int32_t)(now_ms - timestamp) <= (int32_t)stale_ms;
}
