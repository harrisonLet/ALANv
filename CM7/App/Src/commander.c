#include "commander.h"
#include "arbiterRudder.h"
#include "arbiterTail.h"

static typedef struct {
    system_mode_t mode;
    uint16_t rudder_delta;
    uint16_t sail_delta;
    ipc_cmd_t current_cmd;
    volatile cmd_result_t prev_cmd_result;
    volatile uint8_t prev_cmd_seq;
} commander_state_t;

TaskHandle_t commander_task == NULL;

commander_state_t state;

// Converts a system mode into the mode each arbiter runs in. No default case, so the compiler flags a new system mode missing here.
static bool arbiter_modes_for(system_mode_t mode, ArbiterMode_e *rudder, ArbiterMode_e *tail);

HardwareInit_e commander_hardware_init(void) {
    return HARDWARE_INIT_SUCCESS;
}

RtosInit_e commander_rtos_init(void) {
    memset(&state, 0 , sizeof(commander_state_t));
    state.mode = SYSTEM_MODE_AUTO;
    state.prev_cmd_result = CMD_RESULT_NONE;
    // Start at 255 so the first command (seq 0) is not flagged as dropped
    state.prev_cmd_seq = 255U;

    return RTOS_INIT_SUCCESS;
}

void commander_handler(void *argument) {
    ipc_cm7_init();
    TickType_t lastWake = xTaskGetTickCount();
    for(;;) {
        // Read Frequency
        if (!xTaskDelayUntil(&lastWake, configTICK_RATE_HZ / COMMANDER_FREQ)) {
            continue;
        }
        lastWake = xTaskGetTickCount();

        // Drain every queued command, pop returns EMPTY once the ring is drained
        while (ipc_cm7_pop_command(&state.current_cmd) == IPC_STATUS_OK) {

            // check for dropped commands (seq is uint8_t and wraps 255 -> 0)
            uint8_t expected_seq = (uint8_t)(state.prev_cmd_seq + 1U);
            if (state.current_cmd.seq != expected_seq) {
                commander_printf("WARN: command dropped | expected seq %u, popped %u",
                    expected_seq, state.current_cmd.seq);
            }

            state.prev_cmd_result = execute_cmd(&state.current_cmd);
            state.prev_cmd_seq = state.current_cmd.seq;
        }
    }
}

cmd_result_t execute_cmd(ipc_cmd_t *cmd) {

    switch (cmd->type >> 4) {
        // CMD 0x0 is a no-op
        case (0U):
            return CMD_RESULT_NONE;

        // 0x1 is a switch mode command
        case (1U): 
            // Isolate last 4 bits (actual mode num)
            return switch_mode(cmd->type & 0x0F);

        // 0x2 is a manual rudder/sail command
        case (2U):
            return rudder_sail_cntrl(cmd);
        
        // 0x3 is a navigation cmd
        case (3U):
            // TODO: implement navigation
            return CMD_RESULT_UNSUPPORTED;

        // 0x4 to 0xF are not handled yet
        default:
            return CMD_RESULT_UNSUPPORTED;
    }
}

cmd_result_t switch_mode(system_mode_t mode) {
    if (mode == state.mode) {
        return CMD_RESULT_OK;
    }

    ArbiterMode_e rudder;
    ArbiterMode_e tail;
    if (!arbiter_modes_for(mode, &rudder, &tail)) {
        return CMD_RESULT_UNSUPPORTED;
    }

    arbiterRudder_setMode(rudder);
    arbiterTail_setMode(tail);
    state.mode = mode;
    return CMD_RESULT_OK;
}

static bool arbiter_modes_for(system_mode_t mode, ArbiterMode_e *rudder, ArbiterMode_e *tail) {
    switch (mode) {
        case SYSTEM_MODE_AUTO:
            *rudder = ARBITER_MODE_AUTO;
            *tail = ARBITER_MODE_AUTO;
            return true;

        case SYSTEM_MODE_FULL_MAN:
            *rudder = ARBITER_MODE_MANUAL;
            *tail = ARBITER_MODE_MANUAL;
            return true;

        case SYSTEM_MODE_AUTO_SAIL_MAN_RUDDER:
            *rudder = ARBITER_MODE_MANUAL;
            *tail = ARBITER_MODE_AUTO;
            return true;

        case SYSTEM_MODE_AUTO_RUDDER_MAN_SAIL:
            *rudder = ARBITER_MODE_AUTO;
            *tail = ARBITER_MODE_MANUAL;
            return true;

        // Both take the actuators out of play
        case SYSTEM_MODE_STANDBY:
        case SYSTEM_MODE_SAFE:
            *rudder = ARBITER_MODE_SAFE;
            *tail = ARBITER_MODE_SAFE;
            return true;
    }
    return false; // mode came from a radio byte, so it can be out of range
}

cmd_result_t rudder_sail_cntrl(ipc_cmd_t *cmd) {
    // Passsed a non rudder sail command, noop and return
    if (cmd->type >> 4 != 2U) {
        return CMD_RESULT_BAD_VALUE;
    }

    // A manual cmd takes that actuator off autonomy.
    switch (state.mode) {
        case SYSTEM_MODE_AUTO:
            if (cmd->type == COMMAND_SAIL) {
                switch_mode(SYSTEM_MODE_AUTO_RUDDER_MAN_SAIL);
            } else if (cmd->type == COMMAND_RUDDER) {
                switch_mode(SYSTEM_MODE_AUTO_SAIL_MAN_RUDDER);
            }
            return execute_servo(cmd);

        case SYSTEM_MODE_AUTO_RUDDER_MAN_SAIL:
            // Sail already manual, a manual rudder cmd makes both manual
            if (cmd->type == COMMAND_RUDDER) {
                switch_mode(SYSTEM_MODE_FULL_MAN);
            }
            return execute_servo(cmd);

        case SYSTEM_MODE_AUTO_SAIL_MAN_RUDDER:
            // Rudder already manual, a manual sail cmd makes both manual
            if (cmd->type == COMMAND_SAIL) {
                switch_mode(SYSTEM_MODE_FULL_MAN);
            }
            return execute_servo(cmd);

        case SYSTEM_MODE_FULL_MAN:
            return execute_servo(cmd);

        case SYSTEM_MODE_STANDBY:
            return CMD_RESULT_UNSUPPORTED;
        case SYSTEM_MODE_SAFE:
            // In safe/standby mode do not control servo
            return CMD_RESULT_UNSUPPORTED;
    }
    // System is in undefined state
    return CMD_RESULT_BAD_VALUE;
}

cmd_result_t execute_servo(ipc_cmd_t *cmd) {
    // Only sail/rudder commands (0x2X) drive the servos
    if ((cmd->type >> 4) != 2U) {
        return CMD_RESULT_UNSUPPORTED;
    }
    switch (cmd->type) {
        case COMMAND_SAIL:
            // TODO: decode sail setpoint from cmd->raw, range check, drive sail servo
            return CMD_RESULT_OK;

        case COMMAND_RUDDER:
            // TODO: decode rudder setpoint from cmd->raw, range check, drive rudder servo
            return CMD_RESULT_OK;

        default:
            return CMD_RESULT_UNSUPPORTED;
    }
}