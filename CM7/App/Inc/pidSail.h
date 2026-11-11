#ifndef PID_SAIL_H
#define PID_SAIL_H

#include "system.h"

#define PID_SAIL_TASK "[PID_SAIL]"
#define PID_SAIL_TASK_PRIORITY osPriorityNormal

#ifdef CM7_PID_SAIL_DEBUG
#define PID_SAIL_TASK_HEAP (RTOS_MIN_STACK+128)
#define pidSail_printf(fmt, ...) DEBUG_PRINTF(PID_SAIL_TASK, fmt, ##__VA_ARGS__)
#else
#define PID_SAIL_TASK_HEAP (RTOS_MIN_STACK+128)
#define pidSail_printf(...) ((void)0)
#endif

#define PID_SAIL_FREQUENCY 10u  // Hz
#define PID_SAIL_KP 2.0f        // degrees of tail per degrees of main sail error
#define PID_SAIL_KI 0.5f       // removes the steady offset from the wind
#define PID_SAIL_KD 0.5f        
#define PID_SAIL_TAIL_LIMIT 90.0f // max sail tail angle in either direction
#define PID_SAIL_DIRECTION 1

extern TaskHandle_t pidSail_task;

HardwareInit_e pidSail_hardwareInit(void);
RtosInit_e pidSail_rtosInit(void);
void pidSail_handler(void *argument);

void pidSail_setTarget(int16_t angle);

#endif // PID_SAIL_H