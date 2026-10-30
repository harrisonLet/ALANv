#pragma once

#include <stdint.h>

/////////////////////////////////////////////////////////////////////////////
// Navigation

/**
 * This structure represents a nautical vector. A single nautical vector represents
 * an ideal path from a starting position to an ending position and the margins and
 * ranges in which the boat can safely navigate.
 */
typedef struct {
    /**
     * The latitude of the starting position of the vector.
     *
     * Measured in degrees this value represents a GPS coordinate.
     *
     * - Operational Range: -90 to 90 degrees for latitude.
     * - Precision: 4e-6 to 8e-6 degree but in practice it can really only give 1 meter accuracy
     */
    double start_latitude;
    /**
     * The longitude of the starting position of the vector.
     *
     * Measured in degrees this value represents a GPS coordinate.
     *
     * - Operational Range: -180 to 180 degrees for longitude.
     * - Precision: 4e-6 to 8e-6 degree but in practice it can really only give 1 meter accuracy
     */
    double start_longitude;
    /**
     * The absolute distance from the starting position within which the boat
     * can safely begin navigation. When chaining multiple vectors, the boat 
     * must first enter this range before proceeding along the vector. Measured
     * in meters, this values is a radius around the starting position.
     *
     * - Operational Range: 0 to 255 meters.
     * - Precision: 1 meter
     */
    uint8_t start_range;

    /**
     * The latitude of the ending position of the vector.
     *
     * Measured in degrees this value represents a GPS coordinate.
     *
     * - Operational Range: -90 to 90 degrees for latitude.
     * - Precision: 4e-6 to 8e-6 degree but in practice it can really only give 1 meter accuracy
     */
    double end_latitude;
    /**
     * The longitude of the ending position of the vector.
     *
     * Measured in degrees this value represents a GPS coordinate.
     *
     * - Operational Range: -180 to 180 degrees for longitude.
     * - Precision: 4e-6 to 8e-6 degree but in practice it can really only give 1 meter accuracy
     */
    double end_longitude;
    /**
     * The absolute distance from the ending position within which the boat
     * can safely conclude navigation. Measured in meters, this value is a radius
     * around the ending position.
     *
     * - Operational Range: 0 to 255 meters.
     * - Precision: 1 meter
     */
    uint8_t end_range;

    /**
     * The margin from the ideal path within which the boat can safely navigate.
     *
     * Measured in meters, this value represents the allowable deviation from the 
     * ideal path when measuring perpendicular to that path. The value only represents
     * deviation in a single direction, so a margin of 5 meters represents a 10 meter
     * wide corridor around the ideal path.
     *
     * - Operational Range: 0 to 255 meters.
     * - Precision: 1 meter
     */
    uint8_t margin_safe;
    /**
     * The margin from the corridor created by the ideal path and the margin_safe 
     * within which the boat must perform emergency corrective action to return
     * to the safe corridor. Measured in meters, this value represents the area beyond
     * the safe operational corridor. Distance is measured perpendicular to the ideal 
     * path and only in one direction, additionally it is measured beyond the safe 
     * corridor. This means that a margin_safe of 5 meters (10 meter corridor) with a
     * margin_warning of 3 meters would represent a 16 meter wide corridor in total,
     * where the outer 3 meters of both sides represent the warning zone beyond the 
     * safe corridor.
     *
     * This special zone exists to provide an early warning to the boat that it is
     * approaching the limits of the safe corridor, allowing for corrective action
     * before entering potentially hazardous areas. If the boat leaves this warning zone,
     * it should assume recovery is impossible without external intervention and enter
     * a full shut down procedure to prevent potential damage or hazards.
     *
     * - Operational Range: 0 to 255 meters.
     * - Precision: 1 meter
     */
    uint8_t margin_warning;
} NauticalVector_t;

/**
 * A linked list node containing a nautical vector and references to the previous and 
 * next nodes.
 */
typedef struct {
    uint32_t nv_previous; /**< Reference to the previous node in the linked list. */
    NauticalVector_t nv; /**< The nautical vector contained in this node. */
    uint32_t nv_next; /**< Reference to the next node in the linked list. */
} NauticalVectorLL_t;


/**
 * State of the Live Tack Planner (LTP). Sent to the ground station in telemetry,
 * so the numbers are fixed: never renumber an existing state, only add new ones
 * at the end. Stored as uint8_t in structs.
 *
 * IDLE is 0 so a zero-initialized struct reads as "not running".
 *
 * True wind direction: the compass direction where the wind is blowing from, which LTP calc
 * calculates as heading + vane angle
 *
 * No-go: the range of headings within 45 degrees of the wind that the baot can't sail because
 * the sail stalls. For example an upwind waypoint it sails at twd of +/- 45 degrees and zig zag.
 */
typedef enum {
    NAV_STATE_IDLE        = 0x00, /**< Not enabled, or no route loaded. No heading.*/
    NAV_STATE_DIRECT      = 0x01, /**< Waypoint is sailable, steering straight at it. */
    NAV_STATE_UPWIND_PORT = 0x02, /**< Beating with the wind from port (heading = True wind direction + no-go). */
    NAV_STATE_UPWIND_STBD = 0x03, /**< Beating with the wind from starboard (heading = True wind direction - no-go). */
    NAV_STATE_TACKING     = 0x04, /**< Turning onto the other tack, holding the new tack's heading. */
    NAV_STATE_ARRIVED     = 0x05, /**< Last waypoint reached. No heading published. */
    NAV_STATE_FAULT       = 0x06, /**< Bad or stale input, or outside the warning zone. No heading published. */
} NavState_e;


typedef struct {
    uint16_t desired_heading;  // 0..359 deg true, 0xFFFF = none
    uint8_t  nav_state;        // NavState_e
    uint8_t  route_segment;    // current leg index, 0xFF = no route
    int16_t  drift_distance;   // cross-track error, m, + = starboard of the path
    int8_t   tack_side;        // -1 = wind from port, +1 = wind from starboard, 0 = not upwind
    uint8_t  flags;            // bit 0: NAV_FLAG_TACK_SLOW
    uint32_t timestamp;        // ms
    uint8_t  valid;            // Validity_e
} TelemetryNavigation_t;