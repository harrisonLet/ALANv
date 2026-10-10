/////////////////////////////////////////////////////////////////////////////
// Rudder
#define RUDDER_OP_ANGLE_MIN -90
#define RUDDER_OP_ANGLE_MAX 90

/**
 * Represents the current position and rotation of the rudder relative to 
 * the vessel. This value is not sensed in any way but is instead the assumed
 * position based on the commanded rudder angle.
 */
typedef struct {
    /**
     * Forward angle of the rudder relative to the centerline of the vessel.
     *
     * Measured in degrees this represents the angle between the centerline of
     * the vessel and a second line drawn from the tail to the tip of the rudder.
     * positive values indicate the rudder is pointed towards the starboard side.
     * Negative values indicate the rudder is pointed towards the port side.
     *
     * - Operational Range: -90 to 90 degrees
     * - Precision: 1 degree
     */
    int16_t ang_forward;
    /**
     * Turn angle of the rudder relative to the centerline of the vessel.
     *
     * Measured in degrees this represents the kind of turn that will be performed
     * based on the position of the rudder. Because a rudder pointed toward the
     * starboard side will cause the vessel to turn to port and vice versa it is
     * useful to have a separate representation for the turn angle. For this
     * measurment positive values indicate a turn to starboard and negative values 
     * indicate a turn to port.
     *
     * - Operational Range: -90 to 90 degrees
     * - Precision: 1 degree
     */
    int16_t ang_turn;
} TelemetryRudder_t;