#include "commander.h"

typedef struct {
    uint8_t system_state;
    system_mode_t mode;
    uint16_t rudder_delta;
    uint16_t sail_delta;
    ipc_cmd_t current_cmd;
    volatile cmd_result_t prev_cmd_result;
    volatile uint8_t prev_cmd_seq;
} commander_state_t;

commander_state_t state;

HardwareInit_e commander_hardware_init(void) {
    return HARDWARE_INIT_SUCCESS;
}

RtosInit_e commander_rtos_init(void) {
    memset(&state, 0 , sizeof(commander_state_t));
    state.system_state = COMMAND_SYSTEM_ACTIVATE;
    state.mode = COMMAND_MODE_AUTO;
    state.prev_cmd_result = CMD_RESULT_NONE;

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
        bool flushed = false;
        while (!flushed) {
            ipc_status_t result = ipc_cm7_pop_command(&state.current_cmd);

            if (result == IPC_STATUS_FULL) {
                flushed = true;
            }
            if (state.current_cmd.seq != (state.prev_cmd_seq)++) {
                commander_printf("WARN: command %d dropped | popped %d, prev %d", 
                state.current_cmd - 1, state.current_cmd.seq, state.prev_cmd_seq);
                state.prev_cmd_result = CMD_RESULT_DROPPED;
            }
        }
    }
}