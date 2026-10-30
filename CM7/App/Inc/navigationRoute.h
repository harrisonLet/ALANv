#ifndef NAVIGATION_ROUTE_H
#define NAVIGATION_ROUTE_H

// Route Manager: receives waypoints from the command ring, checks the upload, holds the
// active route for LTP. No HAL calls and only one FreeRTOS call (the mutex, stubbed out
// under NAV_HOST_TEST). Never include system.h here.

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "ipc.h"

#define NAV_ROUTE_MAX_WAYPOINTS     32u // fits the 32-bit received mask
#define NAV_ROUTE_DEFAULT_ARRIVE_M  8u  // used when ROUTE_COMMIT sends arrive_m = 0

typedef struct {
    double   lat;      // degrees, + = north
    double   lon;      // degrees, + = east
    uint16_t margin_m; // corridor half-width of the leg that ENDS here, m
} NavWaypoint_t;

typedef struct {
    uint8_t       count;            // number of waypoints, 1..NAV_ROUTE_MAX_WAYPOINTS
    uint8_t       arrive_m;         // arrival radius for every waypoint, m
    uint8_t       margin_warning_m; // warning zone beyond each leg's margin, m
    NavWaypoint_t wp[NAV_ROUTE_MAX_WAYPOINTS];
} NavRoute_t;

/**
  * @brief  CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no final XOR.
  * @param  data  Bytes to check.
  * @param  len   Number of bytes.
  * @retval CRC. "123456789" gives 0x29B1.
  */
uint16_t crc16_ccitt(const uint8_t *data, size_t len);

/**
  * @brief  Clear the active route, the spare buffer and the received mask. Version goes to 0.
  * @retval None
  */
void navigationRoute_init(void);

/**
  * @brief  Check a waypoint and copy it into the spare buffer.
  * @param  wp  Waypoint from the command ring.
  * @retval IPC_RESULT_OK         Stored.
  * @retval IPC_RESULT_BAD_VALUE  Latitude outside +-90, longitude outside +-180, or margin 0. Not stored.
  * @retval IPC_RESULT_TOO_MANY   Index is NAV_ROUTE_MAX_WAYPOINTS or more. Not stored.
  */
uint8_t navigationRoute_addWaypoint(const ipc_cmd_waypoint_t *wp);

/**
  * @brief  Make the spare buffer the active route if the upload is complete and the CRC matches.
  * @param  commit  Commit from the command ring.
  * @retval IPC_RESULT_OK          Committed, version incremented.
  * @retval IPC_RESULT_TOO_MANY    Count is 0 or more than NAV_ROUTE_MAX_WAYPOINTS.
  * @retval IPC_RESULT_INCOMPLETE  A waypoint between 0 and count-1 never arrived.
  * @retval IPC_RESULT_BAD_CRC     The checksum does not match.
  * @note   Clears the spare buffer and the received mask whatever the result. On failure the active route is untouched.
  */
uint8_t navigationRoute_commit(const ipc_cmd_route_commit_t *commit);

/**
  * @brief  Version of the active route.
  * @retval 0 = no route. Goes up by one on every good commit.
  */
uint32_t navigationRoute_version(void);

/**
  * @brief  Copy the active route under the mutex.
  * @param  out  Destination for the route.
  * @retval true   Copied.
  * @retval false  No route, or the mutex could not be taken. out is unchanged.
  */
bool navigationRoute_copy(NavRoute_t *out);

#endif // NAVIGATION_ROUTE_H
