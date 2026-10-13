/////////////////////////////////////////////////////////////////////////////
// Command Structures
#define COMMAND_LENGTH 10 // bytes

/**
 * Represents the kind of command being sent. This will need to be updated to
 * include both ship to shore and shore to ship commands.
 */
typedef enum {

/* Shore to Ship Commands, 0x00 to 0x7F */

//  COMMAND_SYSTEM            = 0x00,
    COMMAND_SYSTEM_ACTIVATE   = 0x00, /**< Activate the autonomous system remotly. */
    COMMAND_SYSTEM_STANDBY    = 0x01, /**< Put the boat into standby mode. */

//  COMMAND_NVS               = 0x30,
    COMMAND_NVS_LOAD_START    = 0x31, /**< */
    COMMAND_NVS_LOAD          = 0x32, /**< */
    COMMAND_NVS_LOAD_STOP     = 0x33, /**< */

//  COMMAND_RUDDER            = 0x60,
    COMMAND_RUDDER_MANUAL     = 0x61, /**< */
    COMMAND_RUDDER_DRIVE      = 0x62, /**< */
    COMMAND_RUDDER_AUTONOMOUS = 0x63, /**< */

//  COMMAND_SAIL              = 0x70,
    COMMAND_SAIL_MANUAL       = 0x71, /**< */
    COMMAND_SAIL_DRIVE        = 0x72, /**< */
    COMMAND_SAIL_AUTONOMOUS   = 0x73  /**< */

/* Ship to Shore Commands, 0x80 to 0xFF */

//  COMMAND_SOMETHING         = 0x80

//  COMMAND_SOMETHING_ELSE    = 0x90
} CommandId_e;

/**
 * Represents a single deconstructed command.
 */
typedef struct {
    uint8_t command_id;
    uint8_t args[COMMAND_LENGTH-1];
} CommandALANv_t