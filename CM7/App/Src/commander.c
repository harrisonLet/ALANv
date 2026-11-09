#include "commander.h"
#include "arbiterRudder.h"
#include "arbiterTail.h"
#include "navigationRoute.h"

_Static_assert(sizeof(ipc_cmd_waypoint_t) <= IPC_CMD_DATA_SIZE, "ipc_cmd_waypoint_t must fit in ipc_cmd_t.raw");
_Static_assert(sizeof(ipc_cmd_route_commit_t) <= IPC_CMD_DATA_SIZE, "ipc_cmd_route_commit_t must fit in ipc_cmd_t.raw");

typedef struct {
    volatile system_mode_t mode;
    uint16_t rudder_delta;
    uint16_t sail_delta;
    ipc_cmd_t current_cmd;
    volatile cmd_result_t prev_cmd_result;
    volatile uint8_t prev_cmd_seq;
    volatile uint8_t reported_cmd_seq; // seq that prev_cmd_result belongs to, sent in telemetry
    TickType_t last_cmd_tick;
} commander_state_t;

TaskHandle_t commander_task = NULL;

static commander_state_t state;

// Converts a system mode into the mode each arbiter runs in. No default case, so the compiler flags a new system mode missing here.
static bool arbiter_modes_for(system_mode_t mode, ArbiterMode_e *rudder, ArbiterMode_e *tail);

// Puts both arbiters in the modes for this system mode. Returns false if the mode is not valid.
static bool apply_arbiter_modes(system_mode_t mode);

// True if the sail or the rudder is following manual setpoints. No default case, so the compiler flags a new system mode missing here.
static bool mode_has_manual(system_mode_t mode);

// Decodes and range checks a sail or rudder setpoint. Returns CMD_RESULT_OK if deg can be used.
static cmd_result_t decode_servo_setpoint(const ipc_cmd_t *cmd, int16_t *deg);

#ifdef COMMANDER_SELFTEST
static void commander_selftest(void);
#endif

HardwareInit_e commander_hardware_init(void) {
    return HARDWARE_INIT_SUCCESS;
}

RtosInit_e commander_rtos_init(void) {
    memset(&state, 0 , sizeof(commander_state_t));
    state.mode = COMMANDER_BOOT_MODE;
    state.prev_cmd_result = CMD_RESULT_NONE;
    // Start at 255 so the first command (seq 0) is not flagged as dropped
    state.prev_cmd_seq = 255U;

    return RTOS_INIT_SUCCESS;
}

void commander_handler(void *argument) {
    ipc_cm7_init();

    // Agree with the arbiters from the first cycle, whatever order they initialized in
    apply_arbiter_modes(COMMANDER_BOOT_MODE);

    #ifdef COMMANDER_SELFTEST
    commander_selftest();
    #endif

    state.last_cmd_tick = xTaskGetTickCount();
    TickType_t lastWake = xTaskGetTickCount();
    for(;;) {
        // Read Frequency
        if (!xTaskDelayUntil(&lastWake, configTICK_RATE_HZ / COMMANDER_FREQ)) {
            continue;
        }
        lastWake = xTaskGetTickCount();

        // Drain every queued command, pop returns EMPTY once the ring is drained
        while (ipc_cm7_pop_command(&state.current_cmd) == IPC_STATUS_OK) {
            state.last_cmd_tick = xTaskGetTickCount();

            // check for dropped commands (seq is uint8_t and wraps 255 -> 0)
            uint8_t expected_seq = (uint8_t)(state.prev_cmd_seq + 1U);
            if (state.current_cmd.seq != expected_seq) {
                commander_printf("WARN: command dropped | expected seq %u, popped %u",
                    expected_seq, state.current_cmd.seq);
            }

            cmd_result_t result = execute_cmd(&state.current_cmd);
            state.prev_cmd_seq = state.current_cmd.seq;

            // A heartbeat has no result, and must not replace the one the ground station is waiting for
            if (state.current_cmd.type != COMMAND_NONE) {
                state.prev_cmd_result = result;
                state.reported_cmd_seq = state.current_cmd.seq;
            }
        }

        // AUTO keeps sailing its route without the radio, so only manual modes time out
        if (mode_has_manual(state.mode) &&
            (xTaskGetTickCount() - state.last_cmd_tick) > pdMS_TO_TICKS(COMMANDER_LINK_TIMEOUT_MS)) {
            commander_printf("WARN: no command for %u ms in mode %u, switching to SAFE",
                (unsigned)COMMANDER_LINK_TIMEOUT_MS, (unsigned)state.mode);
            switch_mode(SYSTEM_MODE_SAFE);
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
            return navigation_cmd(cmd);

        // 0x4 to 0xF are not handled yet
        default:
            return CMD_RESULT_UNSUPPORTED;
    }
}

cmd_result_t switch_mode(system_mode_t mode) {
    if (mode == state.mode) {
        return CMD_RESULT_OK;
    }

    if (!apply_arbiter_modes(mode)) {
        return CMD_RESULT_UNSUPPORTED;
    }

    state.mode = mode;
    return CMD_RESULT_OK;
}

static bool apply_arbiter_modes(system_mode_t mode) {
    ArbiterMode_e rudder;
    ArbiterMode_e tail;
    if (!arbiter_modes_for(mode, &rudder, &tail)) {
        return false;
    }

    arbiterRudder_setMode(rudder);
    arbiterTail_setMode(tail);
    return true;
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

    // Reject a bad setpoint here so it cannot change the mode below
    int16_t deg;
    cmd_result_t setpoint = decode_servo_setpoint(cmd, &deg);
    if (setpoint != CMD_RESULT_OK) {
        return setpoint;
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

    int16_t deg;
    cmd_result_t setpoint = decode_servo_setpoint(cmd, &deg);
    if (setpoint != CMD_RESULT_OK) {
        return setpoint;
    }

    switch (cmd->type) {
        case COMMAND_SAIL:
            arbiterTail_request(ARBITER_TAIL_SOURCE_MANUAL, deg);
            return CMD_RESULT_OK;

        case COMMAND_RUDDER:
            arbiterRudder_request(ARBITER_RUDDER_SOURCE_MANUAL, deg);
            return CMD_RESULT_OK;

        default:
            return CMD_RESULT_UNSUPPORTED;
    }
}

static bool mode_has_manual(system_mode_t mode) {
    switch (mode) {
        case SYSTEM_MODE_FULL_MAN:
        case SYSTEM_MODE_AUTO_SAIL_MAN_RUDDER:
        case SYSTEM_MODE_AUTO_RUDDER_MAN_SAIL:
            return true;

        case SYSTEM_MODE_AUTO:
        case SYSTEM_MODE_STANDBY:
        case SYSTEM_MODE_SAFE:
            return false;
    }
    return false;
}

uint8_t commander_getMode(void) {
    return (uint8_t)state.mode;
}

uint8_t commander_getLastSeq(void) {
    return state.reported_cmd_seq;
}

uint8_t commander_getLastResult(void) {
    return (uint8_t)state.prev_cmd_result;
}

cmd_result_t navigation_cmd(ipc_cmd_t *cmd) {
    switch (cmd->type) {
        // commit() clears the spare buffer, so there is nothing to set up
        case COMMAND_NVS_LOAD_START:
            return CMD_RESULT_OK;

        case COMMAND_NVS_LOAD: {
            ipc_cmd_waypoint_t wp;
            memcpy(&wp, cmd->raw, sizeof wp);
            return navigationRoute_addWaypoint(&wp);
        }

        case COMMAND_NVS_LOAD_STOP: {
            ipc_cmd_route_commit_t c;
            memcpy(&c, cmd->raw, sizeof c);
            return navigationRoute_commit(&c);
        }

        default:
            return CMD_RESULT_UNSUPPORTED;
    }
}

static cmd_result_t decode_servo_setpoint(const ipc_cmd_t *cmd, int16_t *deg) {
    int16_t limit;
    switch (cmd->type) {
        case COMMAND_SAIL:
            limit = ARBITER_TAIL_LIMIT;
            break;

        case COMMAND_RUDDER:
            limit = ARBITER_RUDDER_LIMIT;
            break;

        default:
            return CMD_RESULT_UNSUPPORTED;
    }

    // Signed angle in degrees, little-endian in raw[0..1] like the route packing
    memcpy(deg, cmd->raw, sizeof *deg);
    if (*deg < -limit || *deg > limit) {
        return CMD_RESULT_BAD_VALUE;
    }
    return CMD_RESULT_OK;
}

#ifdef COMMANDER_SELFTEST
/////////////////////////////////////////////////////////////////////////////////////////
// Bench test, runs once at boot. Commands take the same path as real ones from execute_cmd
// on, so this covers everything except the radio. Prints each result, watch the tail servo.
/////////////////////////////////////////////////////////////////////////////////////////

#define SELFTEST_STEP_MS 3000U // long enough to watch the tail finish each move
#define SELFTEST_ROUTE_LEN 3U

static uint32_t selftest_failures;

static void selftest_report(const char *name, bool passed, unsigned got, unsigned expected) {
    if (!passed) { selftest_failures++; }
    commander_printf("SELFTEST %s %s | got %u, expected %u", passed ? "PASS" : "FAIL", name, got, expected);
}

static void selftest_send(const char *name, uint8_t type, const void *payload, size_t len, cmd_result_t expected) {
    ipc_cmd_t cmd;
    memset(&cmd, 0, sizeof cmd);
    cmd.type = type;
    memcpy(cmd.raw, payload, len);
    cmd_result_t result = execute_cmd(&cmd);
    selftest_report(name, result == expected, (unsigned)result, (unsigned)expected);
}

// One servo or mode command, then checks the mode and waits so the tail can be watched
static void selftest_step(const char *name, uint8_t type, int16_t deg, cmd_result_t expected, system_mode_t expected_mode) {
    selftest_send(name, type, &deg, sizeof deg, expected);
    selftest_report("  mode", state.mode == expected_mode, (unsigned)state.mode, (unsigned)expected_mode);
    vTaskDelay(pdMS_TO_TICKS(SELFTEST_STEP_MS));
}

static void selftest_route(const char *name, const ipc_cmd_waypoint_t *route, uint16_t crc, cmd_result_t expected) {
    for (uint16_t i = 0; i < SELFTEST_ROUTE_LEN; i++) {
        selftest_send("  waypoint", COMMAND_NVS_LOAD, &route[i], sizeof route[i], CMD_RESULT_OK);
    }
    ipc_cmd_route_commit_t commit = { .count = SELFTEST_ROUTE_LEN, .crc16 = crc };
    selftest_send(name, COMMAND_NVS_LOAD_STOP, &commit, sizeof commit, expected);
}

static void commander_selftest(void) {
    static const ipc_cmd_waypoint_t route[SELFTEST_ROUTE_LEN] = {
        { .index = 0, .margin_m = 20, .lat_e7 = 407608000, .lon_e7 = -1118910000 },
        { .index = 1, .margin_m = 20, .lat_e7 = 407618000, .lon_e7 = -1118910000 },
        { .index = 2, .margin_m = 20, .lat_e7 = 407628000, .lon_e7 = -1118900000 },
    };

    commander_printf("SELFTEST start");
    switch_mode(SYSTEM_MODE_AUTO); // the script assumes AUTO, whatever the boot mode is

    selftest_step("sail +20", COMMAND_SAIL, 20, CMD_RESULT_OK, SYSTEM_MODE_AUTO_RUDDER_MAN_SAIL);
    selftest_step("sail 200 rejected", COMMAND_SAIL, 200, CMD_RESULT_BAD_VALUE, SYSTEM_MODE_AUTO_RUDDER_MAN_SAIL);
    selftest_step("mode SAFE", COMMAND_MODE_SAFE, 0, CMD_RESULT_OK, SYSTEM_MODE_SAFE);
    selftest_step("mode SAFE again", COMMAND_MODE_SAFE, 0, CMD_RESULT_OK, SYSTEM_MODE_SAFE);
    selftest_step("sail in SAFE", COMMAND_SAIL, 20, CMD_RESULT_UNSUPPORTED, SYSTEM_MODE_SAFE);
    selftest_step("mode AUTO", COMMAND_MODE_AUTO, 0, CMD_RESULT_OK, SYSTEM_MODE_AUTO);

    uint16_t crc = crc16_ccitt((const uint8_t *)route, sizeof route);
    selftest_route("route good CRC", route, crc, CMD_RESULT_OK);
    selftest_report("  route version", navigationRoute_version() == 1U, (unsigned)navigationRoute_version(), 1U);
    selftest_route("route bad CRC", route, (uint16_t)~crc, CMD_RESULT_BAD_CRC);
    selftest_report("  route version", navigationRoute_version() == 1U, (unsigned)navigationRoute_version(), 1U);

    commander_printf("SELFTEST done | %u failed", (unsigned)selftest_failures);
}
#endif
