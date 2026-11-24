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
    COMMAND_MODE_AUTO                  = 0x10,
    COMMAND_MODE_FULL_MAN              = 0x11,
    COMMAND_MODE_SAFE                  = 0x12, /**< Emergency stop. */
    COMMAND_MODE_AUTO_SAIL_MAN_RUDDER  = 0x13,
    COMMAND_MODE_AUTO_RUDDER_MAN_SAIL  = 0x14,
    COMMAND_MODE_STANDBY               = 0x15,
    COMMAND_MODE_FULL_SEMI             = 0x16,
    COMMAND_MODE_SEMI_RUDDER_MAN_SAIL  = 0x17,
    COMMAND_MODE_SEMI_RUDDER_AUTO_SAIL = 0x18,
    COMMAND_MODE_SEMI_SAIL_MAN_RUDDER  = 0x19,
    COMMAND_MODE_SEMI_SAIL_AUTO_RUDDER = 0x1A,


//  COMMAND_SAIL_RUDDER       = 0X20
//  raw[0..1] = int16 degrees, little-endian, rest 0. Each one only works while its actuator is in the
//  matching mode (SERVO: manual, PID: semi-auto), otherwise CMD_RESULT_WRONG_MODE. None of them change the mode.
//  A new PID target restarts the PID. Resending the target it already holds is ignored, so repeats are safe.
    COMMAND_SAIL_SERVO        = 0X21, /**< Tail servo angle, + = starboard. */
    COMMAND_RUDDER_SERVO      = 0X22, /**< Rudder servo angle, + = starboard. */
    COMMAND_SAIL_PID          = 0x23, /**< Main sail angle for the sail PID to hold, -180 to 180. */
    COMMAND_RUDDER_PID        = 0x24, /**< Heading for the heading PID to hold, 0 to 359. */

//  COMMAND_NVS               = 0x30,
    COMMAND_NVS_LOAD_START    = 0x31, /**< */
    COMMAND_NVS_LOAD          = 0x32, /**< */
    COMMAND_NVS_LOAD_STOP     = 0x33, /**< */



/* Ship to Shore Commands, 0x80 to 0xFF */

//  COMMAND_SOMETHING         = 0x80

//  COMMAND_SOMETHING_ELSE    = 0x90
} CommandId_e;

/**
 * The boat's single operating mode. In the control modes it says how each of
 * the sail and rudder is driven:
 *   auto   PID runs, the planner sets its target
 *   semi   PID runs, COMMAND_SAIL_PID / COMMAND_RUDDER_PID set its target
 *   manual no PID, the servo follows COMMAND_SAIL_SERVO / COMMAND_RUDDER_SERVO directly
 * STANDBY and SAFE take both out of play. The COMMAND_MODE_* command with the
 * same suffix selects each mode, so a mode must stay below 0x10. Sent to the
 * ground station in telemetry (mode).
 */
typedef enum {
    SYSTEM_MODE_AUTO                  = 0x00, /**< Sail and rudder both autonomous. */
    SYSTEM_MODE_FULL_MAN              = 0x01, /**< Sail and rudder both manual. */
    SYSTEM_MODE_SAFE                  = 0x02, /**< Emergency stop: autonomy and manual setpoints ignored. */
    SYSTEM_MODE_AUTO_SAIL_MAN_RUDDER  = 0x03, /**< Sail autonomous, rudder manual. */
    SYSTEM_MODE_AUTO_RUDDER_MAN_SAIL  = 0x04, /**< Rudder autonomous, sail manual. */
    SYSTEM_MODE_STANDBY               = 0x05, /**< Idle: autonomy off, waiting for a mode command. */
    SYSTEM_MODE_FULL_SEMI             = 0x06, /**< Sail and rudder both semi-auto. */
    SYSTEM_MODE_SEMI_RUDDER_MAN_SAIL  = 0x07, /**< Rudder semi-auto, sail manual. */
    SYSTEM_MODE_SEMI_RUDDER_AUTO_SAIL = 0x08, /**< Rudder semi-auto, sail autonomous. */
    SYSTEM_MODE_SEMI_SAIL_MAN_RUDDER  = 0x09, /**< Sail semi-auto, rudder manual. */
    SYSTEM_MODE_SEMI_SAIL_AUTO_RUDDER = 0x0A, /**< Sail semi-auto, rudder autonomous. */
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
    CMD_RESULT_WRONG_MODE  = 0x09, /**< Not allowed in the current mode (e.g. COMMAND_RUDDER_SERVO while the rudder is not manual). */
} cmd_result_t;