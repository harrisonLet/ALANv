#include "test_check.h"
#include "navigationRoute.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

// A lake-test waypoint near Salt Lake City, a little farther north for each index
static ipc_cmd_waypoint_t route_wp(uint16_t index)
{
    ipc_cmd_waypoint_t wp = { index, 20, 407608000 + index * 10000, -1118910000 };
    return wp;
}

// Commit for wps[0..count-1], with the CRC computed the way the ground station does
static ipc_cmd_route_commit_t route_commitFor(const ipc_cmd_waypoint_t *wps, uint16_t count)
{
    ipc_cmd_route_commit_t c = { count, crc16_ccitt((const uint8_t *)wps, count * sizeof(*wps)), 0, 5, 0 };
    return c;
}

// Send wps[0..count-1] in order
static void route_send(const ipc_cmd_waypoint_t *wps, uint16_t count)
{
    for (uint16_t i = 0; i < count; i++) { CHECK(navigationRoute_addWaypoint(&wps[i]) == CMD_RESULT_OK); }
}

// Any commit clears the spare buffer and mask; count 0 is the cheapest one
static void route_clear(void)
{
    ipc_cmd_route_commit_t c = { 0 };
    navigationRoute_commit(&c);
}

/////////////////////////////////////////////////////////////////////////////////////////

// CRC-16/CCITT-FALSE check value. The ground station's binascii.crc_hqx(b"123456789", 0xFFFF)
// must give the same, or no route will ever commit.
static void route_crc(void)
{
    CHECK(crc16_ccitt((const uint8_t *)"123456789", 9) == 0x29B1);
}



static void route_addWaypoint(void)
{
    ipc_cmd_waypoint_t wp = route_wp(0);
    CHECK(navigationRoute_addWaypoint(&wp) == CMD_RESULT_OK);
    wp = route_wp(31);
    CHECK(navigationRoute_addWaypoint(&wp) == CMD_RESULT_OK);

    wp = route_wp(32);
    CHECK(navigationRoute_addWaypoint(&wp) == CMD_RESULT_TOO_MANY);
    wp.index = 0xFFFF;
    CHECK(navigationRoute_addWaypoint(&wp) == CMD_RESULT_TOO_MANY);
    wp.index = 32; wp.margin_m = 0; // index is checked before the values
    CHECK(navigationRoute_addWaypoint(&wp) == CMD_RESULT_TOO_MANY);

    // Coordinate limits are inclusive
    wp = route_wp(1); wp.lat_e7 =  900000000;  CHECK(navigationRoute_addWaypoint(&wp) == CMD_RESULT_OK);
    wp = route_wp(1); wp.lat_e7 = -900000000;  CHECK(navigationRoute_addWaypoint(&wp) == CMD_RESULT_OK);
    wp = route_wp(1); wp.lat_e7 =  900000001;  CHECK(navigationRoute_addWaypoint(&wp) == CMD_RESULT_BAD_VALUE);
    wp = route_wp(1); wp.lat_e7 = -900000001;  CHECK(navigationRoute_addWaypoint(&wp) == CMD_RESULT_BAD_VALUE);
    wp = route_wp(1); wp.lat_e7 = INT32_MIN;   CHECK(navigationRoute_addWaypoint(&wp) == CMD_RESULT_BAD_VALUE);
    wp = route_wp(1); wp.lon_e7 =  1800000000; CHECK(navigationRoute_addWaypoint(&wp) == CMD_RESULT_OK);
    wp = route_wp(1); wp.lon_e7 = -1800000000; CHECK(navigationRoute_addWaypoint(&wp) == CMD_RESULT_OK);
    wp = route_wp(1); wp.lon_e7 =  1800000001; CHECK(navigationRoute_addWaypoint(&wp) == CMD_RESULT_BAD_VALUE);
    wp = route_wp(1); wp.lon_e7 = -1800000001; CHECK(navigationRoute_addWaypoint(&wp) == CMD_RESULT_BAD_VALUE);
    wp = route_wp(1); wp.lon_e7 = INT32_MIN;   CHECK(navigationRoute_addWaypoint(&wp) == CMD_RESULT_BAD_VALUE);
    wp = route_wp(1); wp.margin_m = 0;         CHECK(navigationRoute_addWaypoint(&wp) == CMD_RESULT_BAD_VALUE);
    wp = route_wp(1); wp.margin_m = 1;         CHECK(navigationRoute_addWaypoint(&wp) == CMD_RESULT_OK);

    route_clear();
}



static void route_commit(void)
{
    ipc_cmd_waypoint_t wps[NAV_ROUTE_MAX_WAYPOINTS];
    for (uint16_t i = 0; i < NAV_ROUTE_MAX_WAYPOINTS; i++) { wps[i] = route_wp(i); }
    ipc_cmd_route_commit_t c3 = route_commitFor(wps, 3);
    ipc_cmd_route_commit_t c32 = route_commitFor(wps, 32);
    ipc_cmd_waypoint_t wp;

    // Good upload of 3 waypoints
    route_send(wps, 3);
    CHECK(navigationRoute_commit(&c3) == CMD_RESULT_OK);

    // The commit cleared the mask, so committing again without resending is INCOMPLETE
    CHECK(navigationRoute_commit(&c3) == CMD_RESULT_INCOMPLETE);

    // Waypoint 1 never arrived
    route_send(&wps[0], 1);
    route_send(&wps[2], 1);
    CHECK(navigationRoute_commit(&c3) == CMD_RESULT_INCOMPLETE);

    // Waypoint 1 was rejected with BAD_VALUE, so it was not stored
    route_send(&wps[0], 1);
    route_send(&wps[2], 1);
    wp = wps[1]; wp.margin_m = 0;
    CHECK(navigationRoute_addWaypoint(&wp) == CMD_RESULT_BAD_VALUE);
    CHECK(navigationRoute_commit(&c3) == CMD_RESULT_INCOMPLETE);

    // Wrong CRC in the commit
    route_send(wps, 3);
    ipc_cmd_route_commit_t bad = c3; bad.crc16 ^= 0x0001;
    CHECK(navigationRoute_commit(&bad) == CMD_RESULT_BAD_CRC);

    // A waypoint that differs from what the CRC was computed over, by 1e-7 deg
    route_send(wps, 2);
    wp = wps[2]; wp.lon_e7 += 1;
    CHECK(navigationRoute_addWaypoint(&wp) == CMD_RESULT_OK);
    CHECK(navigationRoute_commit(&c3) == CMD_RESULT_BAD_CRC);

    // Count 0 and 33, then check the failed commit still cleared the mask
    route_send(wps, 32);
    ipc_cmd_route_commit_t c0 = route_commitFor(wps, 0);
    CHECK(navigationRoute_commit(&c0) == CMD_RESULT_TOO_MANY);
    route_send(wps, 32);
    ipc_cmd_route_commit_t c33 = c32; c33.count = 33;
    CHECK(navigationRoute_commit(&c33) == CMD_RESULT_TOO_MANY);
    CHECK(navigationRoute_commit(&c32) == CMD_RESULT_INCOMPLETE);

    // A full 32-waypoint route needs every bit; 31 of 32 is not enough
    route_send(wps, 32);
    CHECK(navigationRoute_commit(&c32) == CMD_RESULT_OK);
    route_send(wps, 31);
    CHECK(navigationRoute_commit(&c32) == CMD_RESULT_INCOMPLETE);

    // Waypoints may arrive in any order, and a resent waypoint replaces the first copy
    wp = wps[1]; wp.margin_m = 99;
    CHECK(navigationRoute_addWaypoint(&wp) == CMD_RESULT_OK);
    route_send(&wps[2], 1);
    route_send(&wps[0], 1);
    route_send(&wps[1], 1);
    CHECK(navigationRoute_commit(&c3) == CMD_RESULT_OK);

    // Waypoints past count are ignored, and cleared with the rest
    route_send(wps, 5);
    CHECK(navigationRoute_commit(&c3) == CMD_RESULT_OK);
    route_send(wps, 3);
    ipc_cmd_route_commit_t c5 = route_commitFor(wps, 5);
    CHECK(navigationRoute_commit(&c5) == CMD_RESULT_INCOMPLETE);
}



// Must run before any good commit, while there is still no route
static void route_copyEmpty(void)
{
    NavRoute_t out;
    memset(&out, 0xA5, sizeof(out));
    CHECK(navigationRoute_copy(&out) == false);

    // out is unchanged on false
    NavRoute_t sentinel;
    memset(&sentinel, 0xA5, sizeof(sentinel));
    CHECK(memcmp(&out, &sentinel, sizeof(out)) == 0);
}



// Must run before any other test commits, so the counts start from 0
static void route_version(void)
{
    ipc_cmd_waypoint_t wps[3] = { route_wp(0), route_wp(1), route_wp(2) };
    ipc_cmd_route_commit_t c3 = route_commitFor(wps, 3);

    CHECK(navigationRoute_version() == 0);

    route_send(wps, 3);
    CHECK(navigationRoute_commit(&c3) == CMD_RESULT_OK);
    CHECK(navigationRoute_version() == 1);

    route_send(wps, 3);
    CHECK(navigationRoute_commit(&c3) == CMD_RESULT_OK);
    CHECK(navigationRoute_version() == 2);

    // Bad CRC, then a missing waypoint: neither moves the version
    route_send(wps, 3);
    ipc_cmd_route_commit_t bad = c3; bad.crc16 ^= 0x0001;
    CHECK(navigationRoute_commit(&bad) == CMD_RESULT_BAD_CRC);
    CHECK(navigationRoute_version() == 2);
    route_send(wps, 2);
    CHECK(navigationRoute_commit(&c3) == CMD_RESULT_INCOMPLETE);
    CHECK(navigationRoute_version() == 2);
}



static void route_copy(void)
{
    ipc_cmd_waypoint_t wps[NAV_ROUTE_MAX_WAYPOINTS];
    for (uint16_t i = 0; i < NAV_ROUTE_MAX_WAYPOINTS; i++) { wps[i] = route_wp(i); }
    wps[0].lat_e7 = 407608123;
    NavRoute_t out;

    // arrive_m 0 becomes the default; the e7 integer converts to degrees within 1e-9
    route_send(wps, 3);
    ipc_cmd_route_commit_t c3 = route_commitFor(wps, 3);
    CHECK(navigationRoute_commit(&c3) == CMD_RESULT_OK);
    CHECK(navigationRoute_copy(&out) == true);
    CHECK(out.count == 3);
    CHECK(out.arrive_m == NAV_ROUTE_DEFAULT_ARRIVE_M);
    CHECK(out.margin_warning_m == 5);
    CHECK(fabs(out.wp[0].lat - 40.7608123) < 1e-9);
    CHECK(fabs(out.wp[0].lon - -111.8910000) < 1e-9);
    CHECK(out.wp[0].margin_m == 20);

    // A nonzero arrive_m is kept as sent
    route_send(wps, 3);
    c3.arrive_m = 15;
    CHECK(navigationRoute_commit(&c3) == CMD_RESULT_OK);
    CHECK(navigationRoute_copy(&out) == true);
    CHECK(out.arrive_m == 15);

    // A full 32-waypoint route comes through whole, last waypoint included
    route_send(wps, 32);
    ipc_cmd_route_commit_t c32 = route_commitFor(wps, 32);
    CHECK(navigationRoute_commit(&c32) == CMD_RESULT_OK);
    CHECK(navigationRoute_copy(&out) == true);
    CHECK(out.count == 32);
    CHECK(fabs(out.wp[31].lat - (407608000 + 31 * 10000) / 1e7) < 1e-9);

    // A failed commit leaves the active route alone
    route_send(wps, 3);
    c3.crc16 ^= 0x0001;
    CHECK(navigationRoute_commit(&c3) == CMD_RESULT_BAD_CRC);
    CHECK(navigationRoute_copy(&out) == true);
    CHECK(out.count == 32);
}

/////////////////////////////////////////////////////////////////////////////////////////

void test_route(void)
{
    route_copyEmpty();
    route_version();
    route_crc();
    route_addWaypoint();
    route_commit();
    route_copy();
}
