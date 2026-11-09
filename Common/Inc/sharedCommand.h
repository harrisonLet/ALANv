#pragma once

#include <stdint.h>

/////////////////////////////////////////////////////////////////////////////
// Command IDs, system modes and command results
//
// All three are sent over the radio, so their numbers are fixed: never
// renumber an existing value, only add new ones. Shared structs store them
// as uint8_t, never as the enum type (enum size depends on compiler flags).

/**
 * Represents the kind of command being sent. This will need to be updated to
 * include both ship to shore and shore to ship commands.
 */
typedef enum {

/* Shore to Ship Commands, 0x00 to 0x7F */

    COMMAND_NONE              = 0x00, /**< Not a command. A zeroed ipc_cmd_t reads as this, never as a real command. */


//  COMMAND_MODE              = 0x10, one command per system_mode_t
    COMMAND_MODE_AUTO                 = 0x10,
    COMMAND_MODE_FULL_MAN             = 0x11,
    COMMAND_MODE_SAFE                 = 0x12, /**< Emergency stop. */
    COMMAND_MODE_AUTO_SAIL_MAN_RUDDER = 0x13,
    COMMAND_MODE_AUTO_RUDDER_MAN_SAIL = 0x14,
    COMMAND_MODE_STANDBY              = 0x15,
    

//  COMMAND_SAIL_RUDDER       = 0X20
//  0x21/0x22: raw[0..1] = int16 degrees, + = starboard, rest 0
    COMMAND_SAIL              = 0X21,
    COMMAND_RUDDER            = 0X22,  

//  COMMAND_NVS               = 0x30,
    COMMAND_NVS_LOAD_START    = 0x31, /**< */
    COMMAND_NVS_LOAD          = 0x32, /**< */
    COMMAND_NVS_LOAD_STOP     = 0x33, /**< */



/* Ship to Shore Commands, 0x80 to 0xFF */

//  COMMAND_SOMETHING         = 0x80

//  COMMAND_SOMETHING_ELSE    = 0x90
} CommandId_e;

/**
 * The boat's single operating mode. In the four control modes it says which of
 * the sail and rudder are autonomous and which follow manual setpoints
 * (COMMAND_SAIL / COMMAND_RUDDER); STANDBY and SAFE take both out of play. The
 * COMMAND_MODE_* command with the same suffix selects each mode. Sent to the
 * ground station in telemetry (mode).
 */
typedef enum {
    SYSTEM_MODE_AUTO                 = 0x00, /**< Sail and rudder both autonomous. */
    SYSTEM_MODE_FULL_MAN             = 0x01, /**< Sail and rudder both manual. */
    SYSTEM_MODE_SAFE                 = 0x02, /**< Emergency stop: autonomy and manual setpoints ignored. */
    SYSTEM_MODE_AUTO_SAIL_MAN_RUDDER = 0x03, /**< Sail autonomous, rudder manual. */
    SYSTEM_MODE_AUTO_RUDDER_MAN_SAIL = 0x04, /**< Rudder autonomous, sail manual. */
    SYSTEM_MODE_STANDBY              = 0x05, /**< Idle: autonomy off, waiting for a mode command. */
} system_mode_t;

/**
 * Result of executing one command on the CM7. Sent to the ground station so it
 * knows whether to resend. Separate from ipc_status_t (ipc.h), which only says
 * whether the command made it across the IPC ring.
 */
typedef enum {
    CMD_RESULT_NONE        = 0x00, /**< Not applied, try again (e.g. route lock busy). */
    CMD_RESULT_OK          = 0x01, /**< Applied. */
    CMD_RESULT_UNSUPPORTED = 0x02, /**< Unknown command type. */
    CMD_RESULT_BAD_LEN     = 0x03, /**< Payload length invalid for this command type. */
    CMD_RESULT_BAD_VALUE   = 0x04, /**< Payload value out of range. */
    CMD_RESULT_TOO_MANY    = 0x05, /**< Index or count over the limit (route waypoints). */
    CMD_RESULT_INCOMPLETE  = 0x06, /**< Route commit before every waypoint arrived. */
    CMD_RESULT_BAD_CRC     = 0x07, /**< Route checksum does not match. */
    CMD_RESULT_DROPPED     = 0x08, /**< Command was missed on ring buffer. */
} cmd_result_t;