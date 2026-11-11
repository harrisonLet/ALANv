#ifndef ARBITER_TAIL_H
#define ARBITER_TAIL_H

#include "system.h"
#include "ipc.h"

#define ARBITER_TAIL_TASK "[ATAIL---]"
#define ARBITER_TAIL_TASK_PRIORITY osPriorityAboveNormal

#ifdef CM7_ARBITER_TAIL_DEBUG
#define ARBITER_TAIL_TASK_HEAP (RTOS_MIN_STACK+256)
#define arbiterTail_printf(fmt, ...) DEBUG_PRINTF(ARBITER_TAIL_TASK, fmt, ##__VA_ARGS__)
#else
#define ARBITER_TAIL_TASK_HEAP (RTOS_MIN_STACK+256)
#define arbiterTail_printf(...) ((void)0)
#endif

#define ARBITER_TAIL_FREQUENCY 20u // Hz
#define ARBITER_TAIL_STALE 300u // ms before a PID request is ignored and the tail goes to ARBITER_TAIL_SAFE
#define ARBITER_TAIL_LIMIT 90 // clamps to the maximum degrees on either side
#define ARBITER_TAIL_MAX_STEP 6 // max deg change per update (120 deg/s at 20 Hz)
#define ARBITER_TAIL_SIGN 1
#define ARBITER_TAIL_SAFE 0 // deg, tail angle that feathers the sail, placeholder until the team confirms

typedef enum {
    ARBITER_TAIL_SOURCE_PID,
    ARBITER_TAIL_SOURCE_MANUAL,
    ARBITER_TAIL_SOURCE_COUNT  // number of sources
} ArbiterTailSource_e;

extern TaskHandle_t arbiterTail_task;

HardwareInit_e arbiterTail_hardwareInit(void);
RtosInit_e arbiterTail_rtosInit(void);
void arbiterTail_handler(void *argument);

void arbiterTail_request(ArbiterTailSource_e source, int16_t angle);
void arbiterTail_setMode(uint8_t mode);

#endif // ARBITER_TAIL_H