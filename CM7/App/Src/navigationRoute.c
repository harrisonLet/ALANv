/////////////////////////////////////////////////////////////////////////////////////////
// Route Manager. Never include system.h; the mutex is the only FreeRTOS call and is
// stubbed out under NAV_HOST_TEST so CM7/Test/navigation can build this file.
//
// navigationRoute_version() and navigationRoute_copy() are still stubs.
/////////////////////////////////////////////////////////////////////////////////////////

#include "navigationRoute.h"

#include <string.h>

/////////////////////////////////////////////////////////////////////////////////////////

// The CRC runs over spare as raw bytes, and the ground station packs exactly 12 per waypoint
_Static_assert(sizeof(ipc_cmd_waypoint_t) == 12, "ipc_cmd_waypoint_t must be 12 bytes with no padding, the route CRC depends on it");

static ipc_cmd_waypoint_t spare[NAV_ROUTE_MAX_WAYPOINTS]; // raw upload in progress
static uint32_t rx_mask; // which spare slots have arived to checks which bits are set
static NavRoute_t active; // the route LTP will sail
static uint32_t version; // 0 means no route, +1 for every good commit

#ifndef NAV_HOST_TEST
#include "FreeRTOS.h"
#include "semphr.h"
static SemaphoreHandle_t route_mutex;
#define ROUTE_LOCK()   (xSemaphoreTake(route_mutex, pdMS_TO_TICKS(10)) == pdTRUE)
#define ROUTE_UNLOCK() xSemaphoreGive(route_mutex)
#else
#define ROUTE_LOCK()   true
#define ROUTE_UNLOCK() ((void)0)
#endif

/////////////////////////////////////////////////////////////////////////////////////////

// crc16 provided by claude. need the same code on the ground station sending the commands
uint16_t crc16_ccitt(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}



void navigationRoute_init(void)
{
#ifndef NAV_HOST_TEST
    route_mutex = xSemaphoreCreateMutex();
#endif
}



uint8_t navigationRoute_addWaypoint(const ipc_cmd_waypoint_t *wp)
{
    if (wp->index >= NAV_ROUTE_MAX_WAYPOINTS) { return IPC_RESULT_TOO_MANY; }

    // Compared both ways rather than with abs(), which is undefined for INT32_MIN
    if (wp->lat_e7 > 900000000 || wp->lat_e7 < -900000000 ||
        wp->lon_e7 > 1800000000 || wp->lon_e7 < -1800000000 ||
        wp->margin_m == 0) { return IPC_RESULT_BAD_VALUE; }

    spare[wp->index] = *wp;
    rx_mask |= 1u << wp->index;
    return IPC_RESULT_OK;
}



uint8_t navigationRoute_commit(const ipc_cmd_route_commit_t *commit)
{
    uint16_t count = commit->count;
    // Bits 0..count-1. (1u << 32) is undefined behavior, so a full route (or a bad count) gets all ones
    uint32_t required = (count >= NAV_ROUTE_MAX_WAYPOINTS) ? 0xFFFFFFFFu : (1u << count) - 1u;
    uint8_t result;

    if (count == 0 || count > NAV_ROUTE_MAX_WAYPOINTS) { result = IPC_RESULT_TOO_MANY; }
    else if ((rx_mask & required) != required) { result = IPC_RESULT_INCOMPLETE; }
    else if (crc16_ccitt((const uint8_t *)spare, count * sizeof(ipc_cmd_waypoint_t)) != commit->crc16) { result = IPC_RESULT_BAD_CRC; }
    else if (!ROUTE_LOCK()) { result = IPC_RESULT_NONE; } // LTP held the route for 10 ms; not OK, so the ground station resends
    else
    {
        active.count = (uint8_t)count;
        active.arrive_m = commit->arrive_m ? commit->arrive_m : NAV_ROUTE_DEFAULT_ARRIVE_M;
        active.margin_warning_m = commit->margin_warning_m;
        for (uint16_t i = 0; i < count; i++)
        {
            active.wp[i].lat = spare[i].lat_e7 / 1e7;
            active.wp[i].lon = spare[i].lon_e7 / 1e7;
            active.wp[i].margin_m = spare[i].margin_m;
        }
        version++;
        ROUTE_UNLOCK();
        result = IPC_RESULT_OK;
    }

    // Every path: the next upload starts from an empty spare buffer
    memset(spare, 0, sizeof(spare));
    rx_mask = 0;
    return result;
}



uint32_t navigationRoute_version(void)
{
    return 0;
}



bool navigationRoute_copy(NavRoute_t *out)
{
    (void)out;
    return false;
}
