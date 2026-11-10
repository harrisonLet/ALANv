#include "commander.h"
#include "arbiterRudder.h"
#include "arbiterTail.h"
#include "pidHeading.h"
#include "pidSail.h"
#include "processLSP.h"
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

// Puts both arbiters in the modes for this system mode and pauses or resumes the planners to match.
// Returns false if the mode is not valid.
static bool apply_arbiter_modes(system_mode_t mode);

// Suspends a semi-auto actuator's planner so it cannot overwrite the operator's PID target, and resumes it
// when the actuator goes back to auto. Manual and safe leave it alone, since the arbiter ignores the PID there.
static void apply_planners(ArbiterMode_e rudder, ArbiterMode_e tail);

// True if the sail or the rudder follows the ground station (manual or semi-auto), so the mode needs lora.
// No default case, so the compiler flags a new system mode missing here.
static bool mode_needs_lora(system_mode_t mode);

// Decodes and range checks a sail or rudder servo angle or PID target. Returns CMD_RESULT_OK if deg can be used.
static cmd_result_t decode_setpoint(const ipc_cmd_t *cmd, int16_t *deg);

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

        // AUTO keeps sailing its route without the radio, so only modes driven from the ground station time out
        if (mode_needs_lora(state.mode) &&
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

        // 0x2 is a sail/rudder servo angle or PID target
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
    apply_planners(rudder, tail);
    return true;
}

static void apply_planners(ArbiterMode_e rudder, ArbiterMode_e tail) {
    // NULL when LSP is not built in, and vTaskSuspend(NULL) would suspend the Commander itself
    if (processLSP_task != NULL) {
        if (tail == ARBITER_MODE_SEMI) {
            vTaskSuspend(processLSP_task);
        } else if (tail == ARBITER_MODE_AUTO) {
            processLSP_resendTarget(); // pidSail may still hold the operator's target
            vTaskResume(processLSP_task); // no-op if it was never suspended
        }
    }

    // TODO uncomment when the LTP task is implemented. Like LSP it has to send a fresh heading target
    // after it resumes, or pidHeading keeps holding the operator's heading in AUTO.
    // if (processLTP_task != NULL) {
    //     if (rudder == ARBITER_MODE_SEMI) {
    //         vTaskSuspend(processLTP_task);
    //     } else if (rudder == ARBITER_MODE_AUTO) {
    //         vTaskResume(processLTP_task);
    //     }
    // }
    (void)rudder; // TODO remove with the LTP block above
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

        case SYSTEM_MODE_FULL_SEMI:
            *rudder = ARBITER_MODE_SEMI;
            *tail = ARBITER_MODE_SEMI;
            return true;

        case SYSTEM_MODE_SEMI_RUDDER_MAN_SAIL:
            *rudder = ARBITER_MODE_SEMI;
            *tail = ARBITER_MODE_MANUAL;
            return true;

        case SYSTEM_MODE_SEMI_RUDDER_AUTO_SAIL:
            *rudder = ARBITER_MODE_SEMI;
            *tail = ARBITER_MODE_AUTO;
            return true;

        case SYSTEM_MODE_SEMI_SAIL_MAN_RUDDER:
            *rudder = ARBITER_MODE_MANUAL;
            *tail = ARBITER_MODE_SEMI;
            return true;

        case SYSTEM_MODE_SEMI_SAIL_AUTO_RUDDER:
            *rudder = ARBITER_MODE_AUTO;
            *tail = ARBITER_MODE_SEMI;
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

    // Read the actuator's mode from the same table the arbiters were set from, so the two can never disagree
    ArbiterMode_e rudder;
    ArbiterMode_e tail;
    if (!arbiter_modes_for(state.mode, &rudder, &tail)) {
        // System is in undefined state
        return CMD_RESULT_BAD_VALUE;
    }

    // A servo cmd needs its actuator on manual and a PID cmd needs it on semi-auto. Neither changes the
    // mode, the ground station has to send the mode command first.
    switch (cmd->type) {
        case COMMAND_SAIL_SERVO:
            return (tail == ARBITER_MODE_MANUAL) ? execute_servo(cmd) : CMD_RESULT_WRONG_MODE;

        case COMMAND_RUDDER_SERVO:
            return (rudder == ARBITER_MODE_MANUAL) ? execute_servo(cmd) : CMD_RESULT_WRONG_MODE;

        case COMMAND_SAIL_PID:
            return (tail == ARBITER_MODE_SEMI) ? execute_pid(cmd) : CMD_RESULT_WRONG_MODE;

        case COMMAND_RUDDER_PID:
            return (rudder == ARBITER_MODE_SEMI) ? execute_pid(cmd) : CMD_RESULT_WRONG_MODE;

        default:
            return CMD_RESULT_UNSUPPORTED;
    }
}

cmd_result_t execute_servo(ipc_cmd_t *cmd) {
    // Only sail/rudder commands (0x2X) drive the servos
    if ((cmd->type >> 4) != 2U) {
        return CMD_RESULT_UNSUPPORTED;
    }

    int16_t deg;
    cmd_result_t setpoint = decode_setpoint(cmd, &deg);
    if (setpoint != CMD_RESULT_OK) {
        return setpoint;
    }

    switch (cmd->type) {
        case COMMAND_SAIL_SERVO:
            arbiterTail_request(ARBITER_TAIL_SOURCE_MANUAL, deg);
            return CMD_RESULT_OK;

        case COMMAND_RUDDER_SERVO:
            arbiterRudder_request(ARBITER_RUDDER_SOURCE_MANUAL, deg);
            return CMD_RESULT_OK;

        default:
            return CMD_RESULT_UNSUPPORTED;
    }
}

cmd_result_t execute_pid(ipc_cmd_t *cmd) {
    int16_t deg;
    cmd_result_t setpoint = decode_setpoint(cmd, &deg);
    if (setpoint != CMD_RESULT_OK) {
        return setpoint;
    }

    // Setting a target restarts the PID, so a repeat of the target it already holds is skipped to keep its history.
    // Compared against the PID itself, since LSP, the console and the heading hold can all change it too.
    switch (cmd->type) {
        case COMMAND_SAIL_PID:
            if (pidSail_getTarget() != ANG_WRAP_180(deg)) { // pidSail stores 180 as -180
                pidSail_setTarget(deg);
            }
            return CMD_RESULT_OK;

        case COMMAND_RUDDER_PID:
            if (pidHeading_getTarget() != deg) {
                pidHeading_setTarget((uint16_t)deg);
            }
            return CMD_RESULT_OK;

        default:
            return CMD_RESULT_UNSUPPORTED;
    }
}

static bool mode_needs_lora(system_mode_t mode) {
    switch (mode) {
        case SYSTEM_MODE_FULL_MAN:
        case SYSTEM_MODE_AUTO_SAIL_MAN_RUDDER:
        case SYSTEM_MODE_AUTO_RUDDER_MAN_SAIL:
            return true;

        // A semi-auto actuator holds a target the operator picked, with no route behind it
        case SYSTEM_MODE_FULL_SEMI:
        case SYSTEM_MODE_SEMI_RUDDER_MAN_SAIL:
        case SYSTEM_MODE_SEMI_RUDDER_AUTO_SAIL:
        case SYSTEM_MODE_SEMI_SAIL_MAN_RUDDER:
        case SYSTEM_MODE_SEMI_SAIL_AUTO_RUDDER:
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

static cmd_result_t decode_setpoint(const ipc_cmd_t *cmd, int16_t *deg) {
    int16_t min;
    int16_t max;
    switch (cmd->type) {
        case COMMAND_SAIL_SERVO:
            min = -ARBITER_TAIL_LIMIT;
            max = ARBITER_TAIL_LIMIT;
            break;

        case COMMAND_RUDDER_SERVO:
            min = -ARBITER_RUDDER_LIMIT;
            max = ARBITER_RUDDER_LIMIT;
            break;

        // Same ranges the console checks before calling the PID setters
        case COMMAND_SAIL_PID:
            min = -180;
            max = 180;
            break;

        case COMMAND_RUDDER_PID:
            min = 0;
            max = 359;
            break;

        default:
            return CMD_RESULT_UNSUPPORTED;
    }

    // Signed angle in degrees, little-endian in raw[0..1] like the route packing
    memcpy(deg, cmd->raw, sizeof *deg);
    if (*deg < min || *deg > max) {
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

// Checks whether LSP is suspended. Skipped when LSP is not built in.
static void selftest_lsp(bool expect_suspended) {
    if (processLSP_task == NULL) { return; }
    bool suspended = (eTaskGetState(processLSP_task) == eSuspended);
    selftest_report("  LSP suspended", suspended == expect_suspended, (unsigned)suspended, (unsigned)expect_suspended);
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

    selftest_step("sail servo in AUTO", COMMAND_SAIL_SERVO, 20, CMD_RESULT_WRONG_MODE, SYSTEM_MODE_AUTO);
    selftest_step("mode AUTO_RUDDER_MAN_SAIL", COMMAND_MODE_AUTO_RUDDER_MAN_SAIL, 0, CMD_RESULT_OK, SYSTEM_MODE_AUTO_RUDDER_MAN_SAIL);
    selftest_step("sail servo +20", COMMAND_SAIL_SERVO, 20, CMD_RESULT_OK, SYSTEM_MODE_AUTO_RUDDER_MAN_SAIL);
    selftest_step("rudder servo with rudder on auto", COMMAND_RUDDER_SERVO, 20, CMD_RESULT_WRONG_MODE, SYSTEM_MODE_AUTO_RUDDER_MAN_SAIL);
    selftest_step("sail servo 200 rejected", COMMAND_SAIL_SERVO, 200, CMD_RESULT_BAD_VALUE, SYSTEM_MODE_AUTO_RUDDER_MAN_SAIL);
    selftest_step("sail PID with sail on manual", COMMAND_SAIL_PID, 30, CMD_RESULT_WRONG_MODE, SYSTEM_MODE_AUTO_RUDDER_MAN_SAIL);
    selftest_step("mode FULL_SEMI", COMMAND_MODE_FULL_SEMI, 0, CMD_RESULT_OK, SYSTEM_MODE_FULL_SEMI);
    selftest_lsp(true);
    selftest_step("sail servo in FULL_SEMI", COMMAND_SAIL_SERVO, 20, CMD_RESULT_WRONG_MODE, SYSTEM_MODE_FULL_SEMI);
    selftest_step("sail PID 30", COMMAND_SAIL_PID, 30, CMD_RESULT_OK, SYSTEM_MODE_FULL_SEMI);
    selftest_report("  sail PID target", pidSail_getTarget() == 30, (unsigned)pidSail_getTarget(), 30U);
    selftest_step("sail PID 30 repeat", COMMAND_SAIL_PID, 30, CMD_RESULT_OK, SYSTEM_MODE_FULL_SEMI);
    selftest_step("sail PID 200 rejected", COMMAND_SAIL_PID, 200, CMD_RESULT_BAD_VALUE, SYSTEM_MODE_FULL_SEMI);
    selftest_step("rudder PID 400 rejected", COMMAND_RUDDER_PID, 400, CMD_RESULT_BAD_VALUE, SYSTEM_MODE_FULL_SEMI);
    selftest_step("mode SAFE", COMMAND_MODE_SAFE, 0, CMD_RESULT_OK, SYSTEM_MODE_SAFE);
    selftest_lsp(true); // SAFE leaves the planners alone
    selftest_step("mode SAFE again", COMMAND_MODE_SAFE, 0, CMD_RESULT_OK, SYSTEM_MODE_SAFE);
    selftest_step("sail servo in SAFE", COMMAND_SAIL_SERVO, 20, CMD_RESULT_WRONG_MODE, SYSTEM_MODE_SAFE);
    selftest_step("sail PID in SAFE", COMMAND_SAIL_PID, 30, CMD_RESULT_WRONG_MODE, SYSTEM_MODE_SAFE);
    selftest_step("mode AUTO", COMMAND_MODE_AUTO, 0, CMD_RESULT_OK, SYSTEM_MODE_AUTO);
    selftest_lsp(false);

    uint16_t crc = crc16_ccitt((const uint8_t *)route, sizeof route);
    selftest_route("route good CRC", route, crc, CMD_RESULT_OK);
    selftest_report("  route version", navigationRoute_version() == 1U, (unsigned)navigationRoute_version(), 1U);
    selftest_route("route bad CRC", route, (uint16_t)~crc, CMD_RESULT_BAD_CRC);
    selftest_report("  route version", navigationRoute_version() == 1U, (unsigned)navigationRoute_version(), 1U);

    commander_printf("SELFTEST done | %u failed", (unsigned)selftest_failures);
}
#endif
