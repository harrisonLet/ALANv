/////////////////////////////////////////////////////////////////////////////
// Boat Relative Orientation

/**
 * Represents the change in orientation the vessel is experiencing.
 */
typedef struct {
    int16_t angV_roll; // Raw
    int16_t angV_pitch; // Raw
    int16_t angV_yaw; // Raw

    int16_t linR_heave; // Raw
    int16_t linR_surge; // Raw
    int16_t linR_sway; // Raw
    
    int16_t linA_heave; // Process from linR_heave
    int16_t linA_surge; // Process from linR_surge
    int16_t linA_sway; // Process from linR_sway
} TelemetryBoatRelativeOrientation_t;



/////////////////////////////////////////////////////////////////////////////
// Boat Global Orientation

/**
 * Represents the current global orientation of the vessel.
 *
 * SENSOR: Inertial Measurement Unit (IMU)
 */
typedef struct {
    int16_t angP_roll; // Process from angV_roll
    int16_t angP_pitch; // Process from angV_pitch
    int16_t angP_yaw; // Process from magnetometer
} TelemetryBoatGlobalOrientation_t;



/////////////////////////////////////////////////////////////////////////////
// Boat Global Position

/**
 * Represents the current global position of the vessel.
 *
 * SENSOR: Global Positioning System (GPS)
 */
typedef struct {
    float latitude; // Process from GPS
    float longitude; // Process from GPS
    float altitude; // Process from GPS
} TelemetryGlobalPosition_t;
