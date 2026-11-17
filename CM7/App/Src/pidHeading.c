#include "pidHeading.h"
#include "pid.h"
#include "arbiterRudder.h"
#include "telemetry.h"

#include <math.h>

#define PID_HEADING_NO_TARGET -1

TaskHandle_t pidHeading_task;

static PIDController pid;
static volatile int16_t target_heading;  // 0-359 degrees, or PID_HEADING_NO_TARGET until one is set
static volatile bool pid_restart;        // set when the PID's history is no longer valid
static volatile bool pid_enabled;        // set by the Commander

HardwareInit_e pidHeading_hardwareInit(void)
{
    pidHeading_printf("pidHeading_hardwareInit start");
    pidHeading_printf("pidHeading_hardwareInit complete");
    return HARDWARE_INIT_SUCCESS;
}


RtosInit_e pidHeading_rtosInit(void)
{
    pidHeading_printf("pidHeading_rtosInit start");
    pid_init(&pid, PID_HEADING_KP, PID_HEADING_KI, PID_HEADING_KD,
        -PID_HEADING_RUDDER_LIMIT, PID_HEADING_RUDDER_LIMIT);
    target_heading = PID_HEADING_NO_TARGET;
    pid_restart = true;
    pid_enabled = true; // runs on its own when the Commander is not built in, which then sets it from its boot mode
    pidHeading_printf("pidHeading_rtosInit complete");
    return RTOS_INIT_SUCCESS;
}



void pidHeading_handler(void *argument)
{
    pidHeading_printf("pidHeading_handler start");
    TickType_t lastWakeup = xTaskGetTickCount();
    const float dt = 1.0f / PID_HEADING_FREQUENCY; // dt = 0.05s between updates at 20 Hz

    bool have_heading = false;
    TickType_t first_heading_tick = 0;
    uint16_t last_heading = 0;
    TickType_t last_change_tick = 0;
    uint32_t loop_count = 0;

    // loop that runs at 20 hz, reading the heading and updating the rudder
    for (;;)
    {
        // Turned off by the Commander: suspended until pidHeading_setEnabled(true) wakes it
        // Pausing at the top of the loop means it never stops partway
        if (!pid_enabled) {
            pidHeading_printf("suspended");
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            pid_restart = true;                 // history from before it was off is stale
            lastWakeup = xTaskGetTickCount();   // do not try to catch up on the missed cycles
            last_change_tick = lastWakeup;      // a steady heading while off is not a frozen IMU
            continue;                           // check again, it may have been turned off again already
        }

        if (!xTaskDelayUntil(&lastWakeup, configTICK_RATE_HZ / PID_HEADING_FREQUENCY)) {
            continue;
        }
        TickType_t now = xTaskGetTickCount();

        // No heading yet so send nothing and let the arbiter center
        TelemetryBoatGlobalOrientation_t orientation;
        if (telemetry_getBoatGlobalOrientation(&orientation) != TELEMETRY_OK) {
            pid_restart = true;
            continue;
        }
        uint16_t heading = orientation.angP_heading;

        if (!have_heading) {
            have_heading = true;
            first_heading_tick = now;
            last_heading = heading;
            last_change_tick = now;
            pidHeading_printf("First heading %u, holding in %u ms", heading, PID_HEADING_HOLD_DELAY);
        }

        // the heading will wobble a little so if it hasn't changed for a while, it is dead 
        if (heading != last_heading) {
            last_heading = heading;
            last_change_tick = now;
        }
        if ((now - last_change_tick) > pdMS_TO_TICKS(PID_HEADING_FROZEN)) {
            pid_restart = true;
            continue;
        }

        // With no target from LoRa, hold whatever heading the boat settles on after boot
        if (target_heading == PID_HEADING_NO_TARGET) {
            if ((now - first_heading_tick) < pdMS_TO_TICKS(PID_HEADING_HOLD_DELAY)) 
            {
                continue;
            }
            target_heading = heading;
            pid_restart = true;
            pidHeading_printf("Holding heading %u", heading);
        }

        // Old integeral or derivative is wrong and we need to reset
        if (pid_restart) {
            pid_reset(&pid);
            pid_restart = false;
        }

        float turn = pid_update(&pid, (float)target_heading, (float)heading, dt);
        arbiterRudder_request(ARBITER_RUDDER_SOURCE_PID, (int16_t)lroundf(turn));

        // print everything once a second
        if (++loop_count % PID_HEADING_FREQUENCY == 0) {
            pidHeading_printf("target %d heading %u turn %d", target_heading, heading, (int)lroundf(turn));
        }
    }
}


void pidHeading_setTarget(uint16_t heading)
{
    target_heading = (int16_t)(heading % 360);
    pid_restart = true;
}

int16_t pidHeading_getTarget(void)
{
    return target_heading;
}

void pidHeading_setEnabled(bool enabled)
{
    pid_enabled = enabled;
    // The notification is latched, so it is not lost if the task has not reached its wait yet
    // NULL when the PID is not built in
    if (enabled && pidHeading_task != NULL) {
        xTaskNotifyGive(pidHeading_task);
    }
}