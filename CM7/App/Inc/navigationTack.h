#ifndef NAVIGATION_TACK_H
#define NAVIGATION_TACK_H

// Live Tack Planning (LTP) step: one call per cycle turns inputs and state into the
// navigation output. No RTOS calls inside; the task wrapper gathers the inputs and
// publishes the output. Never include system.h here.

#include <stdbool.h>
#include <stdint.h>

#include "navigationRoute.h"
#include "sharedNavigation.h"

#define NAV_TACK_TASK_FREQUENCY     2u      // Hz, vTaskDelayUntil
#define NAV_NO_GO_DEG               45.0    // closest the boat sails to the true wind; tune
#define NAV_TACK_MIN_INTERVAL_MS    15000u  // minimum time between tacks
#define NAV_TACK_DONE_DEG           20.0    // tack finished when within this of the new heading
#define NAV_TACK_SLOW_MS            20000u  // sets NAV_FLAG_TACK_SLOW; report only, no fault
#define NAV_WIND_WINDOW             20u     // samples, a 10 s circular mean at 2 Hz
#define NAV_WIND_STALE_MS           1000u
#define NAV_IMU_STALE_MS            1000u
#define NAV_GPS_STALE_MS            5000u   // same as SENSOR_GPS_STALE, but sensorGPS.h pulls in system.h
#define NAV_RETURN_ANGLE_DEG        45.0    // angle back toward the line on a DIRECT leg; tune

#define NAV_HEADING_NONE            0xFFFFu // desired_heading when no heading is published
#define NAV_SEGMENT_NONE            0xFFu   // route_segment when no route is loaded
#define NAV_FLAG_TACK_SLOW          0x01u   // flags bit 0: tack taking too long

/**
 * Everything LTP reads in one cycle, gathered by the task wrapper.
 */
typedef struct {
    uint32_t now_ms;            // HAL_GetTick() at the start of the cycle
    bool     enabled;           // set by Commander (a flag until Commander exists)

    double   lat;               // degrees, + = north, from fusion
    double   lon;               // degrees, + = east, from fusion
    uint16_t heading;           // 0..359 deg true, from fusion
    int16_t  ang_wind;          // apparent wind, -180..180 deg vs the bow, + = from starboard

    uint8_t  gps_valid;         // Validity_e, from sensorGPS_getMetadata()
    uint32_t gps_timestamp;     // ms
    uint8_t  imu_valid;         // Validity_e, from sensorIMU_getMetadata()
    uint32_t imu_timestamp;     // ms
    uint8_t  wind_valid;        // Validity_e, from the wind metadata
    uint32_t wind_timestamp;    // ms

    uint32_t          route_version; // navigationRoute_version(), 0 = no route
    const NavRoute_t *route;         // the wrapper's copy from navigationRoute_copy()
} NavTackInput_t;

/**
 * LTP memory between cycles. Zero-initialize before the first step: nav_state 0 is
 * NAV_STATE_IDLE and route_version 0 means no route has been seen yet.
 */
typedef struct {
    uint8_t  nav_state;              // NavState_e
    uint32_t route_version;          // route version the current leg belongs to
    uint8_t  leg;                    // index of the waypoint being sailed to
    double   leg_start_lat;          // degrees
    double   leg_start_lon;          // degrees
    int8_t   tack_side;              // -1 = wind from port, +1 = wind from starboard, 0 = not upwind
    uint32_t tack_ms;                // time of the last tack, ms
    bool     corridor_fault;         // beyond margin + warning; holds until a new route
    double   wind[NAV_WIND_WINDOW];  // true wind direction samples, deg
    uint8_t  wind_count;             // samples in wind[], up to NAV_WIND_WINDOW
    uint8_t  wind_next;              // slot for the next sample
} NavTackState_t;

/**
  * @brief  Run one LTP cycle: check inputs, track the leg, pick the mode, decide tacks.
  * @param  in     This cycle's inputs.
  * @param  state  LTP memory, updated in place.
  * @param  out    Output to publish. desired_heading is NAV_HEADING_NONE in IDLE, ARRIVED and FAULT.
  * @retval None
  */
void navTack_step(const NavTackInput_t *in, NavTackState_t *state, TelemetryNavigation_t *out);

#endif // NAVIGATION_TACK_H
