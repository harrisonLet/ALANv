#include "commander.h"
#include "arbiterRudder.h"
#include "arbiterTail.h"
#include "pidHeading.h"
#include "pidSail.h"
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
    TickType_t result_tick;            // when a real command last set prev_cmd_result
    bool result_held;                  // prev_cmd_result is a real command's, heartbeats leave it alone for COMMANDER_RESULT_HOLD_MS
    uint8_t state_mode_gen;            // mode request number last applied from a COMMAND_STATE, STATE_MODE_GEN_NONE = none
} commander_state_t;

// state_mode_gen when no request number is remembered, so the next mode request applies (numbers are 0..15)
#define STATE_MODE_GEN_NONE 0xFFU

TaskHandle_t commander_task = NULL;

static commander_state_t state;

static volatile bool safe_requested; // set by commander_requestSafe from another task, cleared by the Commander

// Converts a system mode into the mode each arbiter runs in. No default case, so the compiler flags a new system mode missing here.
static bool arbiter_modes_for(system_mode_t mode, ArbiterMode_e *rudder, ArbiterMode_e *tail);

// Puts both arbiters in the modes for this system mode, and suspends each PID whose arbiter is in MANUAL or SAFE.
// The planners pause themselves from the arbiter mode: LSP skips any cycle where the tail is not in AUTO,
// and LTP will need the same check on the rudder.
// Returns false if the mode is not valid.
static bool apply_arbiter_modes(system_mode_t mode);

#ifdef COMMANDER_LINK_TIMEOUT_ENABLE
// True if the sail or the rudder follows the ground station (manual or semi-auto), so the mode needs lora.
// No default case, so the compiler flags a new system mode missing here.
static bool mode_needs_lora(system_mode_t mode);
#endif

// Decodes and range checks a sail or rudder servo angle or PID target. Returns CMD_RESULT_OK if deg can be used.
static cmd_result_t decode_setpoint(const ipc_cmd_t *cmd, int16_t *deg);

// One setpoint of a state packet, as the one-shot command of the same type: checked (apply false) with
// decode_setpoint, or applied (apply true) through execute_servo / execute_pid
static cmd_result_t state_setpoint(uint8_t type, int16_t deg, bool apply);

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
    // Start at 255 so the first command (seq 0) is not flagged as dropped, and so the boot report
    // (NONE) is not matched to the ground station's first command
    state.prev_cmd_seq = 255U;
    state.reported_cmd_seq = 255U;
    // After a reboot the ground station's current mode request applies, whatever its number
    state.state_mode_gen = STATE_MODE_GEN_NONE;

    return RTOS_INIT_SUCCESS;
}

void commander_handler(void *argument) {
    ipc_cm7_init();

    // Agree with the arbiters from the first cycle, whatever order they initialized in
    apply_arbiter_modes(COMMANDER_BOOT_MODE);

    #ifndef COMMANDER_LINK_TIMEOUT_ENABLE
    commander_printf("WARN: link timeout disabled, manual/semi modes will not drop to SAFE if the link is lost");
    #endif

    #ifdef COMMANDER_SELFTEST
    commander_selftest();
    #endif

    state.last_cmd_tick = xTaskGetTickCount();
    for(;;) {
        // Sleep until cm4 notifies with new command or every couple ms in case of missed cmd
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(COMMANDER_POLL_MS));

        // Checked first, so a command already queued behind it can still take the boat out of SAFE
        if (safe_requested) {
            safe_requested = false;
            commander_printf("SAFE requested by another task, mode was %u", (unsigned)state.mode);
            switch_mode(SYSTEM_MODE_SAFE);
        }

        // Drain every queued command, pop returns EMPTY once the ring is drained
        while (ipc_cm7_pop_command(&state.current_cmd) == IPC_STATUS_OK) {
            TickType_t now = xTaskGetTickCount();
            state.last_cmd_tick = now;

            // check for dropped commands (seq is uint8_t and wraps 255 -> 0)
            uint8_t expected_seq = (uint8_t)(state.prev_cmd_seq + 1U);
            if (state.current_cmd.seq != expected_seq) {
                commander_printf("WARN: command dropped | expected seq %u, popped %u",
                    expected_seq, state.current_cmd.seq);
            }

            cmd_result_t result = execute_cmd(&state.current_cmd);
            commander_printf("seq %u type 0x%02X -> result %u, mode %u",
                (unsigned)state.current_cmd.seq, (unsigned)state.current_cmd.type,
                (unsigned)result, (unsigned)state.mode);
            state.prev_cmd_seq = state.current_cmd.seq;

            if (state.current_cmd.type != COMMAND_NONE) {
                state.prev_cmd_result = result;
                state.reported_cmd_seq = state.current_cmd.seq;
                state.result_tick = now;
                state.result_held = true;
            }
            // After COMMANDER_RESULT_HOLD_MS of no cmd results, assume no heartbeat from lora link
            else if (!state.result_held || (now - state.result_tick) > pdMS_TO_TICKS(COMMANDER_RESULT_HOLD_MS)) {
                state.prev_cmd_result = CMD_RESULT_NONE;
                state.reported_cmd_seq = state.current_cmd.seq;
                state.result_held = false;
            }
        }

        #ifdef COMMANDER_LINK_TIMEOUT_ENABLE
        // AUTO keeps sailing its route without the radio, so only modes driven from the ground station time out
        if (mode_needs_lora(state.mode) &&
            (xTaskGetTickCount() - state.last_cmd_tick) > pdMS_TO_TICKS(COMMANDER_LINK_TIMEOUT_MS)) {
            commander_printf("WARN: no command for %u ms in mode %u, switching to SAFE",
                (unsigned)COMMANDER_LINK_TIMEOUT_MS, (unsigned)state.mode);
            switch_mode(SYSTEM_MODE_SAFE);
        }
        #endif
    }
}

cmd_result_t execute_cmd(ipc_cmd_t *cmd) {

    switch (cmd->type >> 4) {
        // 0x00 is the heartbeat, a no-op with no result. The rest of 0x0X is unassigned
        case (0U):
            return (cmd->type == COMMAND_NONE) ? CMD_RESULT_NONE : CMD_RESULT_UNSUPPORTED;

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

        // 0x4 is the ground station's desired state
        case (4U):
            return (cmd->type == COMMAND_STATE) ? apply_state(cmd) : CMD_RESULT_UNSUPPORTED;

        // 0x5 to 0xF are not handled yet
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

    // Arbiters first, so the servos stop listening to a PID before the values stop arriving if we are suspending
    arbiterRudder_setMode(rudder);
    arbiterTail_setMode(tail);

    // A PID only runs while its arbiter uses its value in SEMI or AUTO
    // In MANUAL and SAFE it is suspended -- avoids integral windup on pids
    pidHeading_setEnabled(rudder == ARBITER_MODE_SEMI || rudder == ARBITER_MODE_AUTO);
    pidSail_setEnabled(tail == ARBITER_MODE_SEMI || tail == ARBITER_MODE_AUTO);
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

        // The command is a turn (+= starboard) passed straight to the arbiter. 
        // The rudder servo is mounted upside down, so the arbiter's negation corrects that
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

#ifdef COMMANDER_LINK_TIMEOUT_ENABLE
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
#endif

void commander_requestSafe(void) {
    safe_requested = true;
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

cmd_result_t apply_state(const ipc_cmd_t *cmd) {
    if (cmd->type != COMMAND_STATE) {
        return CMD_RESULT_UNSUPPORTED;
    }
    ipc_cmd_state_t s;
    memcpy(&s, cmd->raw, sizeof s);

    // Check everything before changing anything, so a rejected packet leaves the boat as it was
    ArbiterMode_e rudder;
    ArbiterMode_e tail;
    if (s.mode != IPC_STATE_MODE_KEEP && !arbiter_modes_for((system_mode_t)s.mode, &rudder, &tail)) {
        return CMD_RESULT_UNSUPPORTED;
    }
    const struct { uint8_t type; int16_t deg; } setpoints[] = {
        { COMMAND_RUDDER_SERVO, s.rudder_deg },
        { COMMAND_SAIL_SERVO,   s.sail_deg },
        { COMMAND_RUDDER_PID,   s.heading },
        { COMMAND_SAIL_PID,     s.sail_target },
    };
    for (size_t i = 0; i < sizeof setpoints / sizeof setpoints[0]; i++) {
        if (setpoints[i].deg != IPC_STATE_KEEP && state_setpoint(setpoints[i].type, setpoints[i].deg, false) != CMD_RESULT_OK) {
            return CMD_RESULT_BAD_VALUE;
        }
    }

    // The mode only on a new request number, so resends do not undo a mode the boat chose itself
    // (LTP's SAFE). KEEP means the ground station has no mode request: forget the number, so its next
    // request applies even if it restarted and reused a number.
    uint8_t gen = IPC_STATE_MODE_GEN(s.flags);
    if (s.mode == IPC_STATE_MODE_KEEP) {
        state.state_mode_gen = STATE_MODE_GEN_NONE;
    } else if (gen != state.state_mode_gen) {
        cmd_result_t switched = switch_mode((system_mode_t)s.mode);
        if (switched != CMD_RESULT_OK) {
            return switched;
        }
        state.state_mode_gen = gen;
    }

    // Setpoints for the mode the boat is in now, which may not be the one asked for (LTP's SAFE uses
    // none). The others are ignored, so the state never overrides LTP or LSP. Each goes through the
    // one-shot command's path: the arbiter only moves if the angle changed, the PID target is only set
    // (restarting the PID) if it differs.
    if (!arbiter_modes_for(state.mode, &rudder, &tail)) {
        return CMD_RESULT_BAD_VALUE;   // system is in an undefined state, as in rudder_sail_cntrl
    }
    if (rudder == ARBITER_MODE_MANUAL && s.rudder_deg != IPC_STATE_KEEP) { state_setpoint(COMMAND_RUDDER_SERVO, s.rudder_deg, true); }
    if (rudder == ARBITER_MODE_SEMI   && s.heading    != IPC_STATE_KEEP) { state_setpoint(COMMAND_RUDDER_PID, s.heading, true); }
    if (tail   == ARBITER_MODE_MANUAL && s.sail_deg   != IPC_STATE_KEEP) { state_setpoint(COMMAND_SAIL_SERVO, s.sail_deg, true); }
    if (tail   == ARBITER_MODE_SEMI   && s.sail_target != IPC_STATE_KEEP) { state_setpoint(COMMAND_SAIL_PID, s.sail_target, true); }
    return CMD_RESULT_OK;
}

static cmd_result_t state_setpoint(uint8_t type, int16_t deg, bool apply) {
    ipc_cmd_t sub;
    memset(&sub, 0, sizeof sub);
    sub.type = type;
    memcpy(sub.raw, &deg, sizeof deg);
    if (!apply) {
        int16_t checked;
        return decode_setpoint(&sub, &checked);
    }
    return (type == COMMAND_RUDDER_SERVO || type == COMMAND_SAIL_SERVO) ? execute_servo(&sub) : execute_pid(&sub);
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

static void selftest_route(const char *name, const ipc_cmd_waypoint_t *route, uint16_t crc, cmd_result_t expected) {
    for (uint16_t i = 0; i < SELFTEST_ROUTE_LEN; i++) {
        selftest_send("  waypoint", COMMAND_NVS_LOAD, &route[i], sizeof route[i], CMD_RESULT_OK);
    }
    ipc_cmd_route_commit_t commit = { .count = SELFTEST_ROUTE_LEN, .crc16 = crc };
    selftest_send(name, COMMAND_NVS_LOAD_STOP, &commit, sizeof commit, expected);
}

// One COMMAND_STATE through execute_cmd, then checks the mode. gen is the mode request number
static void selftest_state(const char *name, uint8_t gen, uint8_t mode, int16_t rudder, int16_t sail, int16_t heading,
                           int16_t sail_target, cmd_result_t expected, system_mode_t expected_mode) {
    ipc_cmd_state_t s = { .flags = (uint8_t)(gen << 4), .mode = mode, .rudder_deg = rudder, .sail_deg = sail,
                          .heading = heading, .sail_target = sail_target };
    selftest_send(name, COMMAND_STATE, &s, sizeof s, expected);
    selftest_report("  mode", state.mode == expected_mode, (unsigned)state.mode, (unsigned)expected_mode);
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
    selftest_step("sail servo in FULL_SEMI", COMMAND_SAIL_SERVO, 20, CMD_RESULT_WRONG_MODE, SYSTEM_MODE_FULL_SEMI);
    selftest_step("sail PID 30", COMMAND_SAIL_PID, 30, CMD_RESULT_OK, SYSTEM_MODE_FULL_SEMI);
    selftest_report("  sail PID target", pidSail_getTarget() == 30, (unsigned)pidSail_getTarget(), 30U);
    selftest_step("sail PID 30 repeat", COMMAND_SAIL_PID, 30, CMD_RESULT_OK, SYSTEM_MODE_FULL_SEMI);
    selftest_step("sail PID 200 rejected", COMMAND_SAIL_PID, 200, CMD_RESULT_BAD_VALUE, SYSTEM_MODE_FULL_SEMI);
    selftest_step("rudder PID 400 rejected", COMMAND_RUDDER_PID, 400, CMD_RESULT_BAD_VALUE, SYSTEM_MODE_FULL_SEMI);
    selftest_step("mode SAFE", COMMAND_MODE_SAFE, 0, CMD_RESULT_OK, SYSTEM_MODE_SAFE);
    selftest_step("mode SAFE again", COMMAND_MODE_SAFE, 0, CMD_RESULT_OK, SYSTEM_MODE_SAFE);
    selftest_step("sail servo in SAFE", COMMAND_SAIL_SERVO, 20, CMD_RESULT_WRONG_MODE, SYSTEM_MODE_SAFE);
    selftest_step("sail PID in SAFE", COMMAND_SAIL_PID, 30, CMD_RESULT_WRONG_MODE, SYSTEM_MODE_SAFE);
    selftest_step("mode AUTO", COMMAND_MODE_AUTO, 0, CMD_RESULT_OK, SYSTEM_MODE_AUTO);

    uint16_t crc = crc16_ccitt((const uint8_t *)route, sizeof route);
    selftest_route("route good CRC", route, crc, CMD_RESULT_OK);
    selftest_report("  route version", navigationRoute_version() == 1U, (unsigned)navigationRoute_version(), 1U);
    selftest_route("route bad CRC", route, (uint16_t)~crc, CMD_RESULT_BAD_CRC);
    selftest_report("  route version", navigationRoute_version() == 1U, (unsigned)navigationRoute_version(), 1U);

    // COMMAND_STATE, the ground station's resent desired state
    const int16_t K = IPC_STATE_KEEP;
    selftest_state("state FULL_SEMI, sail target 40", 1, SYSTEM_MODE_FULL_SEMI, K, K, K, 40, CMD_RESULT_OK, SYSTEM_MODE_FULL_SEMI);
    selftest_report("  sail PID target", pidSail_getTarget() == 40, (unsigned)pidSail_getTarget(), 40U);
    uint32_t tail_changes = arbiterTail_getModeChanges();
    selftest_state("state resent unchanged", 1, SYSTEM_MODE_FULL_SEMI, K, K, K, 40, CMD_RESULT_OK, SYSTEM_MODE_FULL_SEMI);
    selftest_report("  no arbiter mode change", arbiterTail_getModeChanges() == tail_changes,
                    (unsigned)arbiterTail_getModeChanges(), (unsigned)tail_changes);
    selftest_report("  sail PID target", pidSail_getTarget() == 40, (unsigned)pidSail_getTarget(), 40U);
    selftest_state("state rudder servo angle while rudder is semi, ignored", 1, SYSTEM_MODE_FULL_SEMI, 30, K, K, 40, CMD_RESULT_OK, SYSTEM_MODE_FULL_SEMI);
    selftest_state("state heading 400 with sail 50, all rejected", 1, SYSTEM_MODE_FULL_SEMI, K, K, 400, 50, CMD_RESULT_BAD_VALUE, SYSTEM_MODE_FULL_SEMI);
    selftest_report("  sail PID target kept", pidSail_getTarget() == 40, (unsigned)pidSail_getTarget(), 40U);
    selftest_state("state invalid mode 0x0E", 2, 0x0E, K, K, K, K, CMD_RESULT_UNSUPPORTED, SYSTEM_MODE_FULL_SEMI);
    switch_mode(SYSTEM_MODE_SAFE);   // as LTP's commander_requestSafe at the end of a route
    selftest_state("state resent after the boat chose SAFE, SAFE kept", 1, SYSTEM_MODE_FULL_SEMI, K, K, K, 40, CMD_RESULT_OK, SYSTEM_MODE_SAFE);
    selftest_state("state new request for the same mode", 2, SYSTEM_MODE_FULL_SEMI, K, K, K, 40, CMD_RESULT_OK, SYSTEM_MODE_FULL_SEMI);
    switch_mode(SYSTEM_MODE_SAFE);
    selftest_state("state all KEEP, request number forgotten", 2, IPC_STATE_MODE_KEEP, K, K, K, K, CMD_RESULT_OK, SYSTEM_MODE_SAFE);
    selftest_state("state request number reused after KEEP", 2, SYSTEM_MODE_FULL_SEMI, K, K, K, K, CMD_RESULT_OK, SYSTEM_MODE_FULL_SEMI);
    selftest_state("state FULL_MAN, tail servo +25 (watch the tail)", 3, SYSTEM_MODE_FULL_MAN, K, 25, K, K, CMD_RESULT_OK, SYSTEM_MODE_FULL_MAN);
    vTaskDelay(pdMS_TO_TICKS(SELFTEST_STEP_MS));

    // Back to AUTO with no request number remembered, so the ground station's first request applies
    switch_mode(SYSTEM_MODE_AUTO);
    state.state_mode_gen = STATE_MODE_GEN_NONE;

    commander_printf("SELFTEST done | %u failed", (unsigned)selftest_failures);
}
#endif
