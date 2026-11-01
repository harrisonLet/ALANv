/////////////////////////////////////////////////////////////////////////////////////////
// Route Manager. Never include system.h; the mutex is the only FreeRTOS call and is
// stubbed out under NAV_HOST_TEST so CM7/Test/navigation can build this file.
//
// This is an outline stub only to help compile
/////////////////////////////////////////////////////////////////////////////////////////

#include "navigationRoute.h"

/////////////////////////////////////////////////////////////////////////////////////////

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
    (void)wp;
    return IPC_RESULT_NONE;
}



uint8_t navigationRoute_commit(const ipc_cmd_route_commit_t *commit)
{
    (void)commit;
    return IPC_RESULT_NONE;
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
