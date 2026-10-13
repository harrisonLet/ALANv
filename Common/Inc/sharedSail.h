/////////////////////////////////////////////////////////////////////////////
// Main Sail
#define MAIN_SAIL_OP_ANGLE_MIN -180
#define MAIN_SAIL_OP_ANGLE_MAX 180

/**
 * Represents the current position and rotation of the sail relative to the vessel.
 *
 * SENSOR: Sail Encoder
 */
typedef struct {
    /**
     * Forward angle of the main sail relative to the centerline of the vessel.
     *
     * Measured in degrees this value represents the angle between the center line
     * of the vessel and a second line drawn from the trailing edge of the main
     * sail to the leading edge of the main sail. Positive values represent the
     * line drawn by the main sail as pointing towards the starboard side of the 
     * vessel. Negative values represent the line drawn by the main sail as pointing
     * towards the port side of the vessel.
     *
     * - Operational Range: -180 to 180
     * - Precision: 1 degree
     */
    int16_t ang_forward;
    /**
     * Perpendicular angle of the main sail pointing off the starboard side of the
     * vessel when the main sail line is pointed directly forward.
     *
     * Measured in degrees this angle is the perpendicular angle of the main sail.
     * 0 degrees represents a main sail that is pointed directly forward. An increase
     * in angle represents a clockwise turn of the main sail. This value will only
     * increase above 0 if ang_port is 0.
     *
     * - Operational Range: 0+ degrees
     * - Precision: 1 degree
     */
    uint16_t ang_starboard;
    /**
     * Perpendicular angle of the main sail pointing off the port side of the
     * vessel when the main sail line is pointed directly forward.
     *
     * Measured in degrees this angle is the perpendicular angle of the main sail.
     * 0 degrees represents a main sail that is pointed directly forward. An increase
     * in angle represents a counter-clockwise turn of the main sail. This value will 
     * only increase above 0 if ang_starboard is 0.
     *
     * - Operational Range: 0+ degrees
     * - Precision: 1 degree
     */
    uint16_t ang_port;
    
    uint32_t timestamp; /**< Timestamp for when the measurement was taken */
    uint8_t valid; /**< Indicates if the measurement is valid */
    uint8_t sequence; /**< Sequence number of the measurement, used to ensure a measurment is actually new */
} TelemetryMainSail_t;



/////////////////////////////////////////////////////////////////////////////
// Tail Sail
#define TAIL_SAIL_FORWARD_OP_ANGLE_MIN -90
#define TAIL_SAIL_FORWARD_OP_ANGLE_MAX 90
#define TAIL_SAIL_REARWARD_OP_ANGLE_ZERO 180
#define TAIL_SAIL_REARWARD_OP_ANGLE_NMIN -90
#define TAIL_SAIL_REARWARD_OP_ANGLE_NMAX -179
#define TAIL_SAIL_REARWARD_OP_ANGLE_PMAX 180
#define TAIL_SAIL_REARWARD_OP_ANGLE_PMIN 90

#define TAIL_SAIL_OP_ANGLE_MIN TAIL_SAIL_FORWARD_OP_ANGLE_MIN
#define TAIL_SAIL_OP_ANGLE_MAX TAIL_SAIL_FORWARD_OP_ANGLE_MAX

/**
 * Represents the current position and rotation of the tail sail relative to the main sail.
 * Because of how ang_forward and ang_rearward are defined they will always have oppsoite
 * sign and an absolute difference of 180 degrees. This means that if ang_forward is greater
 * than ang_rearward (i.e. ang_forward is positive and ang_rearward is negative) then the tail
 * sail is pointing towards the starboard side of the main sail. If ang_forward is less than 
 * ang_rearward it means the tail sail is pointing towards the port side of the main sail.
 *
 * This position is in no way sensed, rather it is the assumed position of the tail sail
 * based on the commanded tail sail position.
 */
typedef struct {
    /**
     * Represents the angle between the main sail and tail sail when measuring in the
     * same direction.
     *
     * Measured in degrees this value represents the angle between a line drawn from the 
     * trailing edge of the main sail to the leading edge of the main sail and a second line 
     * drawn from the trailing edge of the tail sail and the leading edge of the tail sail.
     * Positive values represent the forward line of the tail sail pointing off the starboard
     * side of the main sail. Negative values represent the forward line of the tail sail 
     * pointing off the port side of the main sail.
     *
     * - Operational Range: -90 to 90 degrees
     * - Precision: 1 degree
     */
    int16_t ang_forward;
    /**
     * Represents the angle between the main sail and tail sail when measuring in the 
     * opposite direction.
     *
     * Measured in degrees this value represents the nagle between a line drawn from the
     * trailing edge of the main sail to the leading edge of the main sail and a second line
     * drawn from the leading edge of the tail sail to the trailing edge of the tail sail.
     * Positive values represent the forward (not the rearward) line of the tail sail pointing
     * off the port side of the main sail. Negative values represent the forward (not rearward)
     * line of the tail sail pointing off the starboard side of the main sail.
     *
     * - Operational Zero: 180 degrees
     * - Operational Range: -90 to -179 && 180 to 90 degrees
     * - Precision: 1 degree
     */
    int16_t ang_rearward;

    uint32_t timestamp; /**< Timestamp for when the measurement was taken */
    uint8_t valid; /**< Indicates if the measurement is valid */
    uint8_t sequence; /**< Sequence number of the measurement, used to ensure a measurment is actually new */
} TelemetryTailSail_t;
