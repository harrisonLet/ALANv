#ifndef NAVIGATION_TACK_H
#define NAVIGATION_TACK_H

// Live Tack Planning (LTP) step: one call per cycle turns inputs and state into the
// navigation output. No RTOS calls inside; the task wrapper gathers the inputs and
// publishes the output. Never include system.h here.

#include <stdbool.h>
#include <stdint.h>

#include "navigationRoute.h"
#include "sharedNavigation.h"

#define NAV_TACK_TASK_FREQUENCY     2u      // Hz
#define NAV_NO_GO_DEG               45.0    // closest the boat sails to the wind, measured against the estimated wind, which is apparent unless the estimate is on; tune on the water
#define NAV_NO_GO_HYST_DEG          5.0     // go back to DIRECT only at NO_GO + this, so it can't flip-flop
#define NAV_TACK_XTE_FRACTION       0.8     // tack at this fraction of the leg's margin_m, leaving room for the turn
#define NAV_TACK_MIN_INTERVAL_MS    15000u  // minimum time between tacks
#define NAV_TACK_DONE_DEG           20.0    // tack finished within this of the new heading
#define NAV_TACK_SLOW_MS            20000u  // sets NAV_FLAG_TACK_SLOW; report only
#define NAV_WIND_WINDOW             20u     // samples, 10 s circular mean at 2 Hz
#define NAV_WIND_MIN_SAMPLES        4u      // 2 s of wind before planning
#define NAV_WIND_STALE_MS           1000u
#define NAV_IMU_STALE_MS            1000u
#define NAV_GPS_STALE_MS            5000u
#define NAV_RETURN_ANGLE_DEG        45.0    // DIRECT legs: steer this far back toward the line at the margin
#define NAV_HEADING_DEADBAND_DEG    3u      // task only: re-send the PID target past this change
#define NAV_SOG_MIN_MPS             0.3     // below this, GPS course is noise
#define NAV_TRUE_WIND_ASSUMED_MPS   0.0     // 0 = apparent wind (default); > 0 turns on the estimate

/**
 * Everything LTP reads in one cycle, gathered by the task wrapper.
 */
typedef struct {
    uint32_t now_ms;            // HAL_GetTick() at the start of the cycle
    bool     enabled;           // rudder arbiter is in AUTO

    double   lat;               // degrees, + = north, from fusion
    double   lon;               // degrees, + = east, from fusion
    uint16_t heading;           // 0..359 deg true, from fusion
    int16_t  ang_wind;          // apparent wind, -180..180 deg vs the bow, + = from starboard
    float    sog_mps;           // speed over ground, m/s, from GPS RMC
    uint16_t cog_deg;           // course over ground, 0..359 true
    bool     vel_valid;         // RMC status 'A' and fresh

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
    double   wind[NAV_WIND_WINDOW];  // wind direction samples from wind_from(), deg true, "from"
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

/**
 * The current leg seen from its start, in meters and degrees true. Filled by leg_geometry().
 */
typedef struct {
    double xte;      // cross-track error, m, + = starboard of the path
    double along;    // distance made along the leg, m
    double leg_len;  // m
    double leg_brg;  // bearing of the leg, start to waypoint
    double brg_wp;   // bearing from the boat to the waypoint
    double rel_wp;   // brg_wp relative to the wind, -180..180
    double dist_wp;  // distance from the boat to the waypoint, m
    double margin;   // corridor half-width of this leg, m
} NavTackLeg_t;

// The helpers behind navTack_step. Static in the firmware, visible to CM7/Test/navigation so each
// one can be tested on its own.
#ifdef NAV_HOST_TEST
#define NAV_TACK_HELPER
bool         inputs_ok(const NavTackInput_t *in);
double       wind_estimate(const NavTackInput_t *in, double true_wind_mps);
double       wind_from(const NavTackInput_t *in);
double       wind_update(const NavTackInput_t *in, NavTackState_t *state, bool ok);
NavTackLeg_t leg_geometry(const NavTackInput_t *in, const NavTackState_t *state, double twd);
double       direct_heading(const NavTackLeg_t *leg, double twd);
double       direct_step(const NavTackInput_t *in, NavTackState_t *state, const NavTackLeg_t *leg, double twd);
double       upwind_step(const NavTackInput_t *in, NavTackState_t *state, const NavTackLeg_t *leg, double twd);
double       tacking_step(const NavTackInput_t *in, NavTackState_t *state, double twd, uint8_t *flags);
#else
#define NAV_TACK_HELPER static
#endif

#endif // NAVIGATION_TACK_H
