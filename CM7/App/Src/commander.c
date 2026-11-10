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
    volatile uint8_t state; // SYSTEM_STATE_* bits, written by this task only
    ipc_cmd_t current_cmd;
    volatile cmd_result_t prev_cmd_result;
    volatile uint8_t prev_cmd_seq;
    volatile uint8_t reported_cmd_seq; // seq that prev_cmd_result belongs to, sent in telemetry
    TickType_t last_cmd_tick;
} commander_state_t;

TaskHandle_t commander_task = NULL;

static commander_state_t state;

// True if no planner bit is set without its PID bit, and KILLED is not mixed with anything
static bool state_is_valid(uint8_t s);

// Turns the PIDs and planners on or off and sets both arbiters to match s, then records s as the state.
// s must already be valid.
static void apply_state(uint8_t s);

// Arbiter mode for one actuator: killed -> SAFE, PID off -> MANUAL, PID on -> SEMI, PID and planner on -> AUTO
static ArbiterMode_e arbiter_mode_for(uint8_t s, uint8_t pid_bit, uint8_t planner_bit);

// True if the sail or the rudder follows the ground station (not killed and not fully autonomous),
// so losing the link should kill the boat
static bool state_needs_lora(uint8_t s);

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
    state.state = COMMANDER_BOOT_STATE;
    state.prev_cmd_result = CMD_RESULT_NONE;
    // Start at 255 so the first command (seq 0) is not flagged as dropped
    state.prev_cmd_seq = 255U;

    return RTOS_INIT_SUCCESS;
}

void commander_handler(void *argument) {
    ipc_cm7_init();

    // Agree with the PIDs, planners and arbiters from the first cycle, whatever order they initialized in
    apply_state(COMMANDER_BOOT_STATE);

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

        // Fully autonomous keeps sailing without the radio, so only a ground-station-driven actuator times out
        if (state_needs_lora(state.state) &&
            (xTaskGetTickCount() - state.last_cmd_tick) > pdMS_TO_TICKS(COMMANDER_LINK_TIMEOUT_MS)) {
            commander_printf("WARN: no command for %u ms in state 0x%02X, killing",
                (unsigned)COMMANDER_LINK_TIMEOUT_MS, (unsigned)state.state);
            apply_state(SYSTEM_STATE_KILLED);
        }
    }
}

cmd_result_t execute_cmd(ipc_cmd_t *cmd) {

    switch (cmd->type >> 4) {
        // 0x00 is the heartbeat, the rest of 0x0X is not a command
        case (0U):
            return (cmd->type == COMMAND_NONE) ? CMD_RESULT_NONE : CMD_RESULT_UNSUPPORTED;

        // 0x1 turns a PID or planner on or off, or kills the boat
        case (1U):
            return state_cmd(cmd);

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

cmd_result_t state_cmd(ipc_cmd_t *cmd) {
    uint8_t bit;
    bool on;
    switch (cmd->type) {
        case COMMAND_RUDDER_PID_ON:  bit = SYSTEM_STATE_RUDDER_PID; on = true;  break;
        case COMMAND_RUDDER_PID_OFF: bit = SYSTEM_STATE_RUDDER_PID; on = false; break;
        case COMMAND_LTP_ON:         bit = SYSTEM_STATE_LTP;        on = true;  break;
        case COMMAND_LTP_OFF:        bit = SYSTEM_STATE_LTP;        on = false; break;
        case COMMAND_SAIL_PID_ON:    bit = SYSTEM_STATE_SAIL_PID;   on = true;  break;
        case COMMAND_SAIL_PID_OFF:   bit = SYSTEM_STATE_SAIL_PID;   on = false; break;
        case COMMAND_LSP_ON:         bit = SYSTEM_STATE_LSP;        on = true;  break;
        case COMMAND_LSP_OFF:        bit = SYSTEM_STATE_LSP;        on = false; break;

        case COMMAND_KILL:
            if (state.state != SYSTEM_STATE_KILLED) {
                apply_state(SYSTEM_STATE_KILLED);
            }
            return CMD_RESULT_OK;

        default:
            return CMD_RESULT_UNSUPPORTED;
    }

    // Any on/off command leaves the kill, starting from everything off
    uint8_t current = (state.state & SYSTEM_STATE_KILLED) ? 0U : state.state;
    uint8_t next = on ? (uint8_t)(current | bit) : (uint8_t)(current & ~bit);

    // A planner without its PID is not allowed: ignore the command, nothing changes (kill included)
    if (!state_is_valid(next)) {
        return CMD_RESULT_WRONG_MODE;
    }

    // Repeats change nothing, so they do not restart the PIDs or resend the LSP target
    if (next != state.state) {
        apply_state(next);
    }
    return CMD_RESULT_OK;
}

static bool state_is_valid(uint8_t s) {
    if (s & SYSTEM_STATE_KILLED) {
        return s == SYSTEM_STATE_KILLED;
    }
    if ((s & SYSTEM_STATE_LTP) && !(s & SYSTEM_STATE_RUDDER_PID)) {
        return false;
    }
    if ((s & SYSTEM_STATE_LSP) && !(s & SYSTEM_STATE_SAIL_PID)) {
        return false;
    }
    return (s & (uint8_t)~(SYSTEM_STATE_AUTO | SYSTEM_STATE_KILLED)) == 0U; // no unknown bits
}

static void apply_state(uint8_t s) {
    // Arbiters first, so the servos stop listening to a PID before it is turned off
    arbiterRudder_setMode(arbiter_mode_for(s, SYSTEM_STATE_RUDDER_PID, SYSTEM_STATE_LTP));
    arbiterTail_setMode(arbiter_mode_for(s, SYSTEM_STATE_SAIL_PID, SYSTEM_STATE_LSP));

    pidHeading_setEnabled((s & SYSTEM_STATE_RUDDER_PID) != 0U);
    pidSail_setEnabled((s & SYSTEM_STATE_SAIL_PID) != 0U);
    processLSP_setEnabled((s & SYSTEM_STATE_LSP) != 0U);
    // TODO when the LTP task is implemented: processLTP_setEnabled((s & SYSTEM_STATE_LTP) != 0U).
    // Like LSP it has to resend its heading target when turned on, or pidHeading keeps the operator's heading.

    // TODO kill switch: also cut servo power through the MOSFET once it has a pin

    state.state = s;
}

static ArbiterMode_e arbiter_mode_for(uint8_t s, uint8_t pid_bit, uint8_t planner_bit) {
    if (s & SYSTEM_STATE_KILLED) {
        return ARBITER_MODE_SAFE;
    }
    if (!(s & pid_bit)) {
        return ARBITER_MODE_MANUAL;
    }
    return (s & planner_bit) ? ARBITER_MODE_AUTO : ARBITER_MODE_SEMI;
}

cmd_result_t rudder_sail_cntrl(ipc_cmd_t *cmd) {
    // Passsed a non rudder sail command, noop and return
    if (cmd->type >> 4 != 2U) {
        return CMD_RESULT_BAD_VALUE;
    }

    // Read the actuator's mode the same way the arbiters were set, so the two can never disagree
    ArbiterMode_e rudder = arbiter_mode_for(state.state, SYSTEM_STATE_RUDDER_PID, SYSTEM_STATE_LTP);
    ArbiterMode_e tail = arbiter_mode_for(state.state, SYSTEM_STATE_SAIL_PID, SYSTEM_STATE_LSP);

    // A servo cmd needs its PID off (manual) and a PID cmd needs its PID on with the planner off (semi).
    // Anything else is ignored. Neither changes the state, the ground station has to send the on/off commands first.
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

static bool state_needs_lora(uint8_t s) {
    return !(s & SYSTEM_STATE_KILLED) && (s != SYSTEM_STATE_AUTO);
}

uint8_t commander_getState(void) {
    return state.state;
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

// One command, then checks the state and waits so the tail can be watched
static void selftest_step(const char *name, uint8_t type, int16_t deg, cmd_result_t expected, uint8_t expected_state) {
    selftest_send(name, type, &deg, sizeof deg, expected);
    selftest_report("  state", state.state == expected_state, (unsigned)state.state, (unsigned)expected_state);
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

    const uint8_t rudder_auto = SYSTEM_STATE_RUDDER_PID | SYSTEM_STATE_LTP;

    commander_printf("SELFTEST start");
    apply_state(SYSTEM_STATE_AUTO); // the script assumes everything on, whatever the boot state is

    selftest_step("sail servo with everything on", COMMAND_SAIL_SERVO, 20, CMD_RESULT_WRONG_MODE, SYSTEM_STATE_AUTO);
    selftest_step("sail PID off while LSP on", COMMAND_SAIL_PID_OFF, 0, CMD_RESULT_WRONG_MODE, SYSTEM_STATE_AUTO);
    selftest_step("sail PID target while LSP on", COMMAND_SAIL_PID, 30, CMD_RESULT_WRONG_MODE, SYSTEM_STATE_AUTO);
    selftest_step("LSP off", COMMAND_LSP_OFF, 0, CMD_RESULT_OK, rudder_auto | SYSTEM_STATE_SAIL_PID);
    selftest_step("LSP off repeat", COMMAND_LSP_OFF, 0, CMD_RESULT_OK, rudder_auto | SYSTEM_STATE_SAIL_PID);
    selftest_step("sail servo with sail PID on", COMMAND_SAIL_SERVO, 20, CMD_RESULT_WRONG_MODE, rudder_auto | SYSTEM_STATE_SAIL_PID);
    selftest_step("sail PID 30", COMMAND_SAIL_PID, 30, CMD_RESULT_OK, rudder_auto | SYSTEM_STATE_SAIL_PID);
    selftest_report("  sail PID target", pidSail_getTarget() == 30, (unsigned)pidSail_getTarget(), 30U);
    selftest_step("sail PID 30 repeat", COMMAND_SAIL_PID, 30, CMD_RESULT_OK, rudder_auto | SYSTEM_STATE_SAIL_PID);
    selftest_step("sail PID 200 rejected", COMMAND_SAIL_PID, 200, CMD_RESULT_BAD_VALUE, rudder_auto | SYSTEM_STATE_SAIL_PID);
    selftest_step("rudder PID target while LTP on", COMMAND_RUDDER_PID, 90, CMD_RESULT_WRONG_MODE, rudder_auto | SYSTEM_STATE_SAIL_PID);
    selftest_step("sail PID off", COMMAND_SAIL_PID_OFF, 0, CMD_RESULT_OK, rudder_auto);
    selftest_step("sail servo +20", COMMAND_SAIL_SERVO, 20, CMD_RESULT_OK, rudder_auto);
    selftest_step("sail servo 200 rejected", COMMAND_SAIL_SERVO, 200, CMD_RESULT_BAD_VALUE, rudder_auto);
    selftest_step("LSP on with sail PID off", COMMAND_LSP_ON, 0, CMD_RESULT_WRONG_MODE, rudder_auto);
    selftest_step("rudder PID off while LTP on", COMMAND_RUDDER_PID_OFF, 0, CMD_RESULT_WRONG_MODE, rudder_auto);
    selftest_step("LTP off", COMMAND_LTP_OFF, 0, CMD_RESULT_OK, SYSTEM_STATE_RUDDER_PID);
    selftest_step("rudder PID 400 rejected", COMMAND_RUDDER_PID, 400, CMD_RESULT_BAD_VALUE, SYSTEM_STATE_RUDDER_PID);
    selftest_step("kill", COMMAND_KILL, 0, CMD_RESULT_OK, SYSTEM_STATE_KILLED);
    selftest_step("kill repeat", COMMAND_KILL, 0, CMD_RESULT_OK, SYSTEM_STATE_KILLED);
    selftest_step("sail servo while killed", COMMAND_SAIL_SERVO, 20, CMD_RESULT_WRONG_MODE, SYSTEM_STATE_KILLED);
    selftest_step("LTP on while killed", COMMAND_LTP_ON, 0, CMD_RESULT_WRONG_MODE, SYSTEM_STATE_KILLED);
    selftest_step("sail PID on leaves kill", COMMAND_SAIL_PID_ON, 0, CMD_RESULT_OK, SYSTEM_STATE_SAIL_PID);
    selftest_step("LSP on", COMMAND_LSP_ON, 0, CMD_RESULT_OK, SYSTEM_STATE_SAIL_PID | SYSTEM_STATE_LSP);
    selftest_step("rudder PID on", COMMAND_RUDDER_PID_ON, 0, CMD_RESULT_OK, SYSTEM_STATE_AUTO & (uint8_t)~SYSTEM_STATE_LTP);
    selftest_step("LTP on", COMMAND_LTP_ON, 0, CMD_RESULT_OK, SYSTEM_STATE_AUTO);
    selftest_step("unknown 0x0X type", 0x05, 0, CMD_RESULT_UNSUPPORTED, SYSTEM_STATE_AUTO);

    uint16_t crc = crc16_ccitt((const uint8_t *)route, sizeof route);
    selftest_route("route good CRC", route, crc, CMD_RESULT_OK);
    selftest_report("  route version", navigationRoute_version() == 1U, (unsigned)navigationRoute_version(), 1U);
    selftest_route("route bad CRC", route, (uint16_t)~crc, CMD_RESULT_BAD_CRC);
    selftest_report("  route version", navigationRoute_version() == 1U, (unsigned)navigationRoute_version(), 1U);

    commander_printf("SELFTEST done | %u failed", (unsigned)selftest_failures);
}
#endif
