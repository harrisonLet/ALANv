#include "test_check.h"
#include "navigationGeo.h"

#include <math.h>

// Salt Lake City, roughly where the lake tests happen
#define SLC_LAT  40.7608
#define SLC_LON -111.8910

static int approx(double a, double b, double tol) { return fabs(a - b) <= tol; }
static int angleApprox(double a, double b, double tol) { return fabs(navGeo_wrap180(a - b)) <= tol; }

/////////////////////////////////////////////////////////////////////////////////////////

static void geo_wrap(void)
{
    CHECK(navGeo_wrap360(0.0) == 0.0);
    CHECK(navGeo_wrap360(359.5) == 359.5);
    CHECK(navGeo_wrap360(360.0) == 0.0);
    CHECK(navGeo_wrap360(725.0) == 5.0);
    CHECK(navGeo_wrap360(-90.0) == 270.0);
    CHECK(navGeo_wrap360(-720.0) == 0.0);
    CHECK(navGeo_wrap360(-1e-15) == 0.0); // -1e-15 + 360 rounds to 360, must not be returned

    CHECK(navGeo_wrap180(0.0) == 0.0);
    CHECK(navGeo_wrap180(45.0) == 45.0);
    CHECK(navGeo_wrap180(-45.0) == -45.0);
    CHECK(navGeo_wrap180(180.0) == -180.0); // [-180, 180), same as ANG_WRAP_180
    CHECK(navGeo_wrap180(-180.0) == -180.0);
    CHECK(navGeo_wrap180(190.0) == -170.0);
    CHECK(navGeo_wrap180(-190.0) == 170.0);
    CHECK(navGeo_wrap180(359.0) == -1.0);
    CHECK(navGeo_wrap180(540.0) == -180.0);
}



static void geo_toLocal(void)
{
    NavLocal_t p = navGeo_toLocal(SLC_LAT, SLC_LON, SLC_LAT, SLC_LON);
    CHECK(p.north == 0.0 && p.east == 0.0);

    // 0.001 deg of latitude is 111.32 m anywhere
    p = navGeo_toLocal(SLC_LAT, SLC_LON, SLC_LAT + 0.001, SLC_LON);
    CHECK(approx(p.north, 111.32, 1e-6));
    CHECK(p.east == 0.0);

    // 0.001 deg of longitude shrinks with cos(latitude)
    p = navGeo_toLocal(SLC_LAT, SLC_LON, SLC_LAT, SLC_LON + 0.001);
    CHECK(p.north == 0.0);
    CHECK(approx(p.east, 84.3184355, 1e-6));

    // at the equator both are 111.32 m, and south/west are negative
    p = navGeo_toLocal(0.0, 0.0, -0.001, -0.001);
    CHECK(approx(p.north, -111.32, 1e-6));
    CHECK(approx(p.east, -111.32, 1e-6));
}



static void geo_bearingDistance(void)
{
    NavLocal_t o = { 0.0, 0.0 };
    NavLocal_t n = { 100.0, 0.0 }, e = { 0.0, 100.0 }, s = { -100.0, 0.0 }, w = { 0.0, -100.0 };

    CHECK(approx(navGeo_bearing(o, n), 0.0, 1e-9));
    CHECK(approx(navGeo_bearing(o, e), 90.0, 1e-9));
    CHECK(approx(navGeo_bearing(o, s), 180.0, 1e-9));
    CHECK(approx(navGeo_bearing(o, w), 270.0, 1e-9));
    CHECK(approx(navGeo_bearing(o, (NavLocal_t){ 100.0, 100.0 }), 45.0, 1e-9));
    CHECK(approx(navGeo_bearing(o, (NavLocal_t){ -100.0, -100.0 }), 225.0, 1e-9));
    CHECK(approx(navGeo_bearing(n, o), 180.0, 1e-9));
    CHECK(approx(navGeo_bearing(e, n), 315.0, 1e-9)); // from neither point at the origin
    CHECK(navGeo_bearing(o, o) == 0.0);

    CHECK(approx(navGeo_distance(o, (NavLocal_t){ 30.0, 40.0 }), 50.0, 1e-9));
    CHECK(approx(navGeo_distance(n, s), 200.0, 1e-9));
    CHECK(approx(navGeo_distance(s, n), 200.0, 1e-9));
    CHECK(navGeo_distance(e, e) == 0.0);

    // A ~600 m leg near SLC. Reference values from sim/plant/alan_plant/geo.py (simulation
    // branch) and a haversine. Bearing matches geo.py exactly; distance is 0.11% longer than
    // both because the firmware uses 111320 m/deg and geo.py uses R = 6371 km.
    NavLocal_t start = navGeo_toLocal(SLC_LAT, SLC_LON, SLC_LAT, SLC_LON);
    NavLocal_t end   = navGeo_toLocal(SLC_LAT, SLC_LON, 40.7642, -111.8853);
    CHECK(approx(navGeo_bearing(start, end), 51.779301, 1e-6)); // geo.py
    CHECK(approx(navGeo_bearing(start, end), 51.776728, 0.01)); // haversine
    CHECK(approx(navGeo_distance(start, end), 611.0675, 611.0675 * 0.002)); // geo.py
    CHECK(approx(navGeo_distance(start, end), 611.0579, 611.0579 * 0.002)); // haversine
}



static void geo_track(void)
{
    // Northbound leg, 100 m
    NavLocal_t start = { 0.0, 0.0 }, end = { 100.0, 0.0 };

    CHECK(approx(navGeo_crossTrack(start, end, (NavLocal_t){ 50.0, 10.0 }), 10.0, 1e-9));   // east = starboard
    CHECK(approx(navGeo_crossTrack(start, end, (NavLocal_t){ 50.0, -10.0 }), -10.0, 1e-9)); // west = port
    CHECK(approx(navGeo_crossTrack(start, end, (NavLocal_t){ 70.0, 0.0 }), 0.0, 1e-9));     // on the line
    CHECK(approx(navGeo_crossTrack(start, end, (NavLocal_t){ 150.0, 5.0 }), 5.0, 1e-9));    // past the end, still the line's distance

    CHECK(approx(navGeo_alongTrack(start, end, (NavLocal_t){ 50.0, 10.0 }), 50.0, 1e-9));
    CHECK(approx(navGeo_alongTrack(start, end, (NavLocal_t){ -20.0, 3.0 }), -20.0, 1e-9)); // behind the start
    CHECK(approx(navGeo_alongTrack(start, end, (NavLocal_t){ 100.0, 0.0 }), 100.0, 1e-9)); // at the end
    CHECK(navGeo_alongTrack(start, end, (NavLocal_t){ 105.0, 30.0 }) >= 100.0);           // passed it 30 m off the line

    // Same leg sailed south: east is now port
    CHECK(approx(navGeo_crossTrack(end, start, (NavLocal_t){ 50.0, 10.0 }), -10.0, 1e-9));
    CHECK(approx(navGeo_alongTrack(end, start, (NavLocal_t){ 20.0, 10.0 }), 80.0, 1e-9));

    // Eastbound leg away from the origin: south is starboard
    NavLocal_t a = { 20.0, 20.0 }, b = { 20.0, 120.0 };
    CHECK(approx(navGeo_crossTrack(a, b, (NavLocal_t){ 15.0, 70.0 }), 5.0, 1e-9));
    CHECK(approx(navGeo_alongTrack(a, b, (NavLocal_t){ 15.0, 70.0 }), 50.0, 1e-9));

    // Diagonal leg along a 3-4-5 triangle: boat at (4, -3) is 5 m off, 90 deg to port
    NavLocal_t diag = { 30.0, 40.0 };
    CHECK(approx(navGeo_crossTrack(start, diag, (NavLocal_t){ 4.0, -3.0 }), -5.0, 1e-9));
    CHECK(approx(navGeo_alongTrack(start, diag, (NavLocal_t){ 4.0, -3.0 }), 0.0, 1e-9));
    CHECK(approx(navGeo_alongTrack(start, diag, (NavLocal_t){ 3.0, 4.0 }), 5.0, 1e-9));

    // Zero-length leg: no line to measure from
    CHECK(navGeo_crossTrack(start, start, (NavLocal_t){ 5.0, 5.0 }) == 0.0);
    CHECK(navGeo_alongTrack(start, start, (NavLocal_t){ 5.0, 5.0 }) == 0.0);

    // From GPS: a boat just east of a northbound leg is to starboard
    NavLocal_t gs = navGeo_toLocal(SLC_LAT, SLC_LON, SLC_LAT, SLC_LON);
    NavLocal_t ge = navGeo_toLocal(SLC_LAT, SLC_LON, SLC_LAT + 0.002, SLC_LON);
    NavLocal_t gb = navGeo_toLocal(SLC_LAT, SLC_LON, SLC_LAT + 0.001, SLC_LON + 0.0001);
    CHECK(approx(navGeo_crossTrack(gs, ge, gb), 8.43184355, 1e-6));
    CHECK(approx(navGeo_alongTrack(gs, ge, gb), 111.32, 1e-6));
}



static void geo_circularMean(void)
{
    double across_north[] = { 350.0, 10.0 };
    double m = navGeo_circularMean(across_north, 2);
    CHECK(angleApprox(m, 0.0, 1e-9)); // not 180
    CHECK(m >= 0.0 && m < 360.0);

    double east[] = { 80.0, 100.0 };
    CHECK(approx(navGeo_circularMean(east, 2), 90.0, 1e-9));

    double west[] = { 260.0, 280.0 };
    CHECK(approx(navGeo_circularMean(west, 2), 270.0, 1e-9));

    double three[] = { 355.0, 5.0, 15.0 };
    CHECK(approx(navGeo_circularMean(three, 3), 5.0, 1e-9));

    double negative[] = { -30.0, -10.0 }; // inputs need not be wrapped
    CHECK(approx(navGeo_circularMean(negative, 2), 340.0, 1e-9));

    double one[] = { 123.0 };
    CHECK(approx(navGeo_circularMean(one, 1), 123.0, 1e-9));

    // A full 20-sample window: 10 at 40 deg and 10 at 50 deg
    double window[20];
    for (int i = 0; i < 20; i++) { window[i] = (i % 2) ? 50.0 : 40.0; }
    CHECK(approx(navGeo_circularMean(window, 20), 45.0, 1e-9));

    CHECK(navGeo_circularMean(across_north, 0) == 0.0); // empty
}

/////////////////////////////////////////////////////////////////////////////////////////

void test_geo(void)
{
    geo_wrap();
    geo_toLocal();
    geo_bearingDistance();
    geo_track();
    geo_circularMean();
}
