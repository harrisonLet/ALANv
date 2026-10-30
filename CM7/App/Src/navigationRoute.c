/////////////////////////////////////////////////////////////////////////////////////////
// Route Manager. Never include system.h; the mutex is the only FreeRTOS call and is
// stubbed out under NAV_HOST_TEST so CM7/Test/navigation can build this file.
//
// STUB: compiles only. Bodies come with the Route Manager step of the design plan.
/////////////////////////////////////////////////////////////////////////////////////////

#include "navigationRoute.h"

/////////////////////////////////////////////////////////////////////////////////////////

uint16_t crc16_ccitt(const uint8_t *data, size_t len)
{
    (void)data;
    (void)len;
    return 0;
}



void navigationRoute_init(void)
{
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
