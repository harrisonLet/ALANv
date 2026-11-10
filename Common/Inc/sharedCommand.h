#pragma once

#include <stdint.h>

/////////////////////////////////////////////////////////////////////////////
// Command IDs, system state bits and command results
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

    COMMAND_NONE              = 0x00, /**< Heartbeat. Keeps the link alive, has no result. A zeroed ipc_cmd_t reads as this. */


//  COMMAND_STATE             = 0x10, each one turns one SYSTEM_STATE_* bit on or off, no payload.
//  A planner needs its PID: LTP_ON needs the rudder PID on, RUDDER_PID_OFF needs LTP off (same for
//  LSP and the sail PID). A command that breaks that rule is ignored and returns CMD_RESULT_WRONG_MODE.
//  Turning on something that is already on (or off something already off) is OK, so repeats are safe.
//  While killed, any of these leaves the kill, starting from everything off.
    COMMAND_RUDDER_PID_ON     = 0x10,
    COMMAND_RUDDER_PID_OFF    = 0x11,
    COMMAND_LTP_ON            = 0x12,
    COMMAND_LTP_OFF           = 0x13,
    COMMAND_SAIL_PID_ON       = 0x14,
    COMMAND_SAIL_PID_OFF      = 0x15,
    COMMAND_LSP_ON            = 0x16,
    COMMAND_LSP_OFF           = 0x17,
    COMMAND_KILL              = 0x1F, /**< Turns everything off and holds both servos at their safe position. */


//  COMMAND_SAIL_RUDDER       = 0X20
//  raw[0..1] = int16 degrees, little-endian, rest 0. Each one is only allowed in the state listed,
//  otherwise it is ignored and returns CMD_RESULT_WRONG_MODE. None of them change the state.
//  A new PID target restarts the PID. Resending the target it already holds is ignored, so repeats are safe.
    COMMAND_SAIL_SERVO        = 0X21, /**< Tail servo angle, + = starboard. Sail PID off (so LSP off). */
    COMMAND_RUDDER_SERVO      = 0X22, /**< Rudder servo angle, + = starboard. Rudder PID off (so LTP off). */
    COMMAND_SAIL_PID          = 0x23, /**< Main sail angle for the sail PID to hold, -180 to 180. Sail PID on, LSP off. */
    COMMAND_RUDDER_PID        = 0x24, /**< Heading for the heading PID to hold, 0 to 359. Rudder PID on, LTP off. */

//  COMMAND_NVS               = 0x30,
    COMMAND_NVS_LOAD_START    = 0x31, /**< */
    COMMAND_NVS_LOAD          = 0x32, /**< */
    COMMAND_NVS_LOAD_STOP     = 0x33, /**< */



/* Ship to Shore Commands, 0x80 to 0xFF */

//  COMMAND_SOMETHING         = 0x80

//  COMMAND_SOMETHING_ELSE    = 0x90
} CommandId_e;

/**
 * The boat's state, tracked by the Commander as one byte of these bits and sent
 * to the ground station in telemetry (mode). Each actuator ends up in one of three
 * states, set by its PID and planner bits:
 *   PID off              manual: the servo follows COMMAND_SAIL_SERVO / COMMAND_RUDDER_SERVO
 *   PID on, planner off  the PID holds the target from COMMAND_SAIL_PID / COMMAND_RUDDER_PID
 *   PID on, planner on   autonomous: the planner sets the PID target
 * A planner bit is never set without its PID bit. KILLED is only ever set alone.
 */
#define SYSTEM_STATE_RUDDER_PID  (0x01U) /**< Heading PID drives the rudder. */
#define SYSTEM_STATE_LTP         (0x02U) /**< Live Tack Planning sets the heading target. */
#define SYSTEM_STATE_SAIL_PID    (0x04U) /**< Sail PID drives the tail. */
#define SYSTEM_STATE_LSP         (0x08U) /**< Live Sail Planning sets the sail target. */
#define SYSTEM_STATE_KILLED      (0x80U) /**< Kill switch: everything off, both servos at their safe position. */
#define SYSTEM_STATE_AUTO        (SYSTEM_STATE_RUDDER_PID | SYSTEM_STATE_LTP | SYSTEM_STATE_SAIL_PID | SYSTEM_STATE_LSP)

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