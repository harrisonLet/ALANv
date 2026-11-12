#ifndef COMMANDS_H
#define COMMANDS_H

/*
 * Command IDs, system modes and command results the boat understands.
 *
 * Copy of Common/Inc/sharedCommand.h on command-pipeline. Keep the numbers identical:
 * they go over the radio, so the boat and the ground station must agree on every value.
 */

/* Shore to ship command types (byte [4] of a command packet) */
enum {
    COMMAND_NONE                       = 0x00, /* heartbeat, keeps the link alive */

    /* 0x10 | system mode: one command per mode */
    COMMAND_MODE_AUTO                  = 0x10,
    COMMAND_MODE_FULL_MAN              = 0x11,
    COMMAND_MODE_SAFE                  = 0x12,
    COMMAND_MODE_AUTO_SAIL_MAN_RUDDER  = 0x13,
    COMMAND_MODE_AUTO_RUDDER_MAN_SAIL  = 0x14,
    COMMAND_MODE_STANDBY               = 0x15,
    COMMAND_MODE_FULL_SEMI             = 0x16,
    COMMAND_MODE_SEMI_RUDDER_MAN_SAIL  = 0x17,
    COMMAND_MODE_SEMI_RUDDER_AUTO_SAIL = 0x18,
    COMMAND_MODE_SEMI_SAIL_MAN_RUDDER  = 0x19,
    COMMAND_MODE_SEMI_SAIL_AUTO_RUDDER = 0x1A,

    /* payload = int16 degrees, little-endian (2 bytes) */
    COMMAND_SAIL_SERVO                 = 0x21, /* tail servo angle, sail must be manual */
    COMMAND_RUDDER_SERVO               = 0x22, /* rudder servo angle, rudder must be manual */
    COMMAND_SAIL_PID                   = 0x23, /* main sail angle for the sail PID, sail must be semi */
    COMMAND_RUDDER_PID                 = 0x24, /* heading for the heading PID, rudder must be semi */
};

/* system_mode_t, reported in telemetry (mode) */
enum {
    SYSTEM_MODE_AUTO                  = 0x00,
    SYSTEM_MODE_FULL_MAN              = 0x01,
    SYSTEM_MODE_SAFE                  = 0x02,
    SYSTEM_MODE_AUTO_SAIL_MAN_RUDDER  = 0x03,
    SYSTEM_MODE_AUTO_RUDDER_MAN_SAIL  = 0x04,
    SYSTEM_MODE_STANDBY               = 0x05,
    SYSTEM_MODE_FULL_SEMI             = 0x06,
    SYSTEM_MODE_SEMI_RUDDER_MAN_SAIL  = 0x07,
    SYSTEM_MODE_SEMI_RUDDER_AUTO_SAIL = 0x08,
    SYSTEM_MODE_SEMI_SAIL_MAN_RUDDER  = 0x09,
    SYSTEM_MODE_SEMI_SAIL_AUTO_RUDDER = 0x0A,
    SYSTEM_MODE_COUNT
};

/* cmd_result_t, reported in telemetry (last_cmd_result) */
enum {
    CMD_RESULT_NONE        = 0x00,
    CMD_RESULT_OK          = 0x01,
    CMD_RESULT_UNSUPPORTED = 0x02,
    CMD_RESULT_BAD_LEN     = 0x03,
    CMD_RESULT_BAD_VALUE   = 0x04,
    CMD_RESULT_TOO_MANY    = 0x05,
    CMD_RESULT_INCOMPLETE  = 0x06,
    CMD_RESULT_BAD_CRC     = 0x07,
    CMD_RESULT_DROPPED     = 0x08,
    CMD_RESULT_WRONG_MODE  = 0x09,
    CMD_RESULT_COUNT
};

/* Limits the boat checks (CM7 commander.c decode_setpoint). Checked here too so a typo
   is caught before it uses airtime. */
#define LIMIT_SERVO_DEG      90   /* ARBITER_RUDDER_LIMIT / ARBITER_TAIL_LIMIT */
#define LIMIT_SAIL_PID_DEG   180
#define LIMIT_HEADING_MAX    359

#define CMD_PAYLOAD_MAX      16   /* IPC_CMD_DATA_SIZE: payloads are 0 to 16 bytes, len = 1 + payload */

#endif
