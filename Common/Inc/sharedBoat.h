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
     * Angular velocity around the roll axis. Measured in degrees per second.
     */
    int16_t angV_roll;
    /**
     * Angular velocity around the pitch axis. Measured in degrees per second.
     */
    int16_t angV_pitch;
    /**
     * Angular velocity around the yaw axis. Measured in degrees per second.
     */
    int16_t angV_yaw;

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
     * Linear acceleration along the heave axis derived from linR_heave.
     * Measured in millimeters per second squared.
     */
    int16_t linA_heave;
    /**
     * Linear acceleration along the surge axis derived from linR_surge.
     * Measured in millimeters per second squared.
     */
    int16_t linA_surge;
    /**
     * Linear acceleration along the sway axis derived from linR_sway.
     * Measured in millimeters per second squared.
     */
    int16_t linA_sway;

    uint32_t timestamp; /**< Timestamp for when the measurement was taken */
    uint8_t valid; /**< Indicates if the measurement is valid */
    uint8_t sequence; /**< Sequence number of the measurement, used to ensure a measurment is actually new */
} TelemetryBoatRelativeOrientation_t;



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
     */
    int16_t angP_pitch;
    
    /**
     * By combining the readings of the magnetometer with the integration of
     * the angular velocity in the yaw axis, we can find the absolute heading
     * of the vessel relative to magnetic north.
     *
     * This value represents the Heading of the vessel relative to magnetic north.
     * Its operational range is from 0 to 360 degrees, where 0/360 indicates
     * north, 90 indicates east, 180 indicates south, and 270 indicates west.
     */
    uint16_t angP_heading;

    /**
     * Latitude of the vessel obtained from GPS. Its operational range is from -90 
     * to 90 degrees, where positive values indicate north latitude and negative 
     * values indicate south latitude.
     */
    float latitude;
    /**
     * Longitude of the vessel obtained from GPS. Its operational range is from -180 
     * to 180 degrees, where positive values indicate east longitude and negative 
     * values indicate west longitude. 0 indicates the prime meridian.
     */
    float longitude;
    
    uint32_t timestamp; /**< Timestamp for when the measurement was taken */
    uint8_t valid; /**< Indicates if the measurement is valid */
    uint8_t sequence; /**< Sequence number of the measurement, used to ensure a measurment is actually new */
} TelemetryBoatGlobalOrientation_t;

