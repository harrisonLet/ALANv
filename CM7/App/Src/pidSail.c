#include "pidSail.h"
#include "pid.h"
#include "arbiterTail.h"
#include "telemetry.h"

#include <math.h>

#define PID_SAIL_NO_TARGET INT16_MIN

TaskHandle_t pidSail_task;

static PIDController pid;
static volatile int16_t target_angle;  // -180 to 179 degrees, or PID_SAIL_NO_TARGET until LSP sets one
static volatile bool pid_restart;      // set when the PID's history is no longer valid
static volatile bool pid_enabled;      // set by the Commander

HardwareInit_e pidSail_hardwareInit(void)
{
    pidSail_printf("pidSail_hardwareInit start");
    pidSail_printf("pidSail_hardwareInit complete");
    return HARDWARE_INIT_SUCCESS;
}

RtosInit_e pidSail_rtosInit(void)
{
    pidSail_printf("pidSail_rtosInit start");
    pid_init(&pid, PID_SAIL_KP, PID_SAIL_KI, PID_SAIL_KD, -PID_SAIL_TAIL_LIMIT, PID_SAIL_TAIL_LIMIT);
    target_angle = PID_SAIL_NO_TARGET;
    pid_restart = true;
    pid_enabled = true; // runs on its own when the Commander is not built in, which then sets it from its boot state
    pidSail_printf("pidSail_rtosInit complete");
    return RTOS_INIT_SUCCESS;
}

void pidSail_handler(void *argument)
{
    pidSail_printf("pidSail_handler start");
    TickType_t lastWakeup = xTaskGetTickCount();
    const float dt = 1.0f / PID_SAIL_FREQUENCY;
    uint32_t loop_count = 0;

    // loop that runs at 10 Hz, reading the main sail angle and updating the tail
    for (;;)
    {
        if (!xTaskDelayUntil(&lastWakeup, configTICK_RATE_HZ / PID_SAIL_FREQUENCY)) {
            continue;
        }

        // Turned off by the Commander, so send nothing
        if (!pid_enabled) {
            pid_restart = true;
            continue;
        }

        // No target from LSP yet, so send nothing and let the arbiter hold the safe angle
        if (target_angle == PID_SAIL_NO_TARGET) {
            pid_restart = true;
            continue;
        }

        // No encoder reading yet, so send nothing
        TelemetryMainSail_t main_sail;
        if (telemetry_getMainSail(&main_sail) != TELEMETRY_OK) {
            pid_restart = true;
            continue;
        }
        int16_t sail_angle = main_sail.ang_forward;

        // Old integral is wrong after a gap or a new target
        if (pid_restart) {
            pid_reset(&pid);
            pid_restart = false;
        }

        float tail = PID_SAIL_DIRECTION * pid_update(&pid, (float)target_angle, (float)sail_angle, dt);
        arbiterTail_request(ARBITER_TAIL_SOURCE_PID, (int16_t)lroundf(tail));

        // print everything once a second
        if (++loop_count % PID_SAIL_FREQUENCY == 0) {
            pidSail_printf("target %d sail %d tail %d", target_angle, sail_angle, (int)lroundf(tail));
        }
    }
}

void pidSail_setTarget(int16_t angle)
{
    target_angle = ANG_WRAP_180(angle);
    pid_restart = true;
}

int16_t pidSail_getTarget(void)
{
    return target_angle;
}

void pidSail_setEnabled(bool enabled)
{
    if (enabled && !pid_enabled) {
        pid_restart = true; // history from before it was off is stale
    }
    pid_enabled = enabled;
}
