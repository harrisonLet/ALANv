#pragma once

#include <stdint.h>

/////////////////////////////////////////////////////////////////////////////
// Boat Relative Orientation

/**
 * Represents the orientation of the vessel relative to itself. I.E. how the 
 * boats orientation changes not what its absolute orientation is.
 *
 * AXES: (Angular)
 * - Roll: Rotation around the longitudinal axis (bow to stern)
 * - Pitch: Rotation around the lateral axis (port to starboard)
 * - Yaw: Rotation around the vertical axis (up and down)
 *
 * AXES: (Linear)
 * - Heave: Motion along the vertical axis (up and down)
 * - Surge: Motion along the longitudinal axis (forward and backward)
 * - Sway: Motion along the lateral axis (left and right)
 *
 * SENSOR: Inertial Measurement Unit (IMU) / Global Positioning System (GPS)
 */
typedef struct {
    /**
     * Linear velocity along the heave axis. Measured in millimeters per second.
     */
    int16_t linV_heave;
    /**
     * Linear velocity along the surge axis. Measured in millimeters per second.
     */
    int16_t linV_surge;
    /**
     * Linear velocity along the sway axis. Measured in millimeters per second.
     */
    int16_t linV_sway;
    
    /**
     * Linear acceleration along the heave axis derived from raw sensor readings.
     * Measured in millimeters per second squared.
     */
    int16_t linA_heave;
    /**
     * Linear acceleration along the surge axis derived from raw sensor readings.
     * Measured in millimeters per second squared.
     */
    int16_t linA_surge;
    /**
     * Linear acceleration along the sway axis derived from raw sensor readings.
     * Measured in millimeters per second squared.
     */
    int16_t linA_sway;
} TelemetryBoatRelativeOrientation_t;

#define TELEMETRY_BOAT_RELATIVE_ORIENTATION_STR_SIZE 128u
char *telemetryBoatRelativeOrientation_toString(const TelemetryBoatRelativeOrientation_t *telemetry, char buffer[TELEMETRY_BOAT_RELATIVE_ORIENTATION_STR_SIZE]);



/////////////////////////////////////////////////////////////////////////////
// Boat Global Orientation
#define ANGULAR_POSITION_ROLL_MIN -180
#define ANGULAR_POSITION_ROLL_MAX 180

#define ANGULAR_POSITION_PITCH_MIN -180
#define ANGULAR_POSITION_PITCH_MAX 180

#define ANGULAR_POSITION_HEADING_MIN 0
#define ANGULAR_POSITION_HEADING_MAX 360
#define ANGULAR_POSITION_HEADING_NORTH 0
#define ANGULAR_POSITION_HEADING_EAST 90
#define ANGULAR_POSITION_HEADING_SOUTH 180
#define ANGULAR_POSITION_HEADING_WEST 270

#define LATITUDE_MIN -90.0
#define LATITUDE_MAX 90.0
#define LONGITUDE_MIN -180.0
#define LONGITUDE_MAX 180.0

/**
 * Represents the current global orientation of the vessel.
 *
 * AXES: (Angular)
 * - Roll: Rotation around the longitudinal axis (forward and backward)
 * - Pitch: Rotation around the lateral axis (left and right)
 *
 * AXES: (Heading)
 * - Heading: Rotation around the vertical axis (up and down) representing the 
 *   vessel's orientation relative to magnetic north
 *
 * SENSOR: Inertial Measurement Unit (IMU) / Global Positioning System (GPS)
 */
typedef struct {
    /**
     * Because of gravity there will always be an acceleration in the down 
     * direction. Using this value we can calculate the absolute roll of the 
     * vessel. Measured in degrees.
     *
     * This value represents the Roll of the vessel away from straight up and 
     * down. Thus its operational range is from -180 to 180 degrees. Where
     * positive values indicate a roll towards the starboard side.
     *
     * - Operational Range: -180 to 180 degrees
     * - Precision: 1 degree
     */
    int16_t angP_roll;
    /**
     * Because of gravity there will always be an acceleration in the down 
     * direction. Using this value we can calculate the absolute pitch of the 
     * vessel. Measured in degrees.
     *
     * This value represents the Pitch of the vessel away from straight up and 
     * down. Thus its operational range is from -180 to 180 degrees. Where
     * positive values indicate a pitch towards the bow of the vessel.
     *
     * - Operational Range: -180 to 180 degrees
     * - Precision: 1 degree
     */
    int16_t angP_pitch;
    
    /**
     * By combining the readings of the magnetometer with the integration of
     * the angular velocity in the yaw axis and GPS coordinates, we can find 
     * the absolute heading of the vessel relative to true north.
     *
     * This value represents the heading of the vessel relative to true north.
     * Its operational range is from 0 to 360 degrees, where 0/360 indicates
     * north, 90 indicates east, 180 indicates south, and 270 indicates west.
     *
     * - Operational Range: [0 to 360) degrees
     * - Precision: 1 degree
     */
    uint16_t angP_heading;

    /**
     * Latitude of the vessel obtained from GPS. Its operational range is from -90 
     * to 90 degrees, where positive values indicate north latitude and negative 
     * values indicate south latitude.
     *
     * - Operational Range: -90 to 90 degrees
     * - Precision: 4e-6 to 8e-6 degree but in practice it can really only give 1 meter accuracy
     */
    double latitude;
    /**
     * Longitude of the vessel obtained from GPS. Its operational range is from -180 
     * to 180 degrees, where positive values indicate east longitude and negative 
     * values indicate west longitude. 0 indicates the prime meridian.
     *
     * - Operational Range: -180 to 180 degrees
     * - Precision: 4e-6 to 8e-6 degree but in practice it can really only give 1 meter accuracy
     */
    double longitude;
} TelemetryBoatGlobalOrientation_t;

#define TELEMETRY_BOAT_GLOBAL_ORIENTATION_STR_SIZE 128u
char *telemetryBoatGlobalOrientation_toString(const TelemetryBoatGlobalOrientation_t *telemetry, char buffer[TELEMETRY_BOAT_GLOBAL_ORIENTATION_STR_SIZE]);
