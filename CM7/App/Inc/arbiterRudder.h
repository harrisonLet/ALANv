#ifndef ARBITER_RUDDER_H
#define ARBITER_RUDDER_H

#include "system.h"
#include "ipc.h"

#define ARBITER_RUDDER_TASK "[ARUDDER-]"
#define ARBITER_RUDDER_TASK_PRIORITY osPriorityAboveNormal
#define ARBITER_RUDDER_TASK_HEAP 300 // this is sized for printf's from servoSetRudder

#ifdef CM7_ARBITER_RUDDER_DEBUG
#define arbiterRudder_printf(fmt, ...) DEBUG_PRINTF(ARBITER_RUDDER_TASK, fmt, ##__VA_ARGS__)
#else
#define arbiterRudder_printf(...) ((void)0)
#endif

#define ARBITER_RUDDER_FREQUENCY 20u // Hz
#define ARBITER_RUDDER_STALE 200u // ms before a PID request is ignored and the rudder centers
#define ARBITER_RUDDER_LIMIT 90 // clamps to the maximum degrees on either side
#define ARBITER_RUDDER_SLEW 6 // max deg change per update (120 deg/s at 20 Hz)
#define ARBITER_RUDDER_SIGN 1

typedef enum {
    ARBITER_RUDDER_SOURCE_PID,
    ARBITER_RUDDER_SOURCE_MANUAL,
    ARBITER_RUDDER_SOURCE_COUNT  // number of sources
} ArbiterRudderSource_e;

extern TaskHandle_t arbiterRudder_task;

HardwareInit_e arbiterRudder_hardwareInit(void);
RtosInit_e arbiterRudder_rtosInit(void);
void arbiterRudder_handler(void *argument);

void arbiterRudder_request(ArbiterRudderSource_e source, int16_t turn);
void arbiterRudder_setMode(uint8_t mode);

#endif // ARBITER_RUDDER_H