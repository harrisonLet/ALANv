#ifndef FUSION_ORIENTATION_H
#define FUSION_ORIENTATION_H

#include "system.h"
#include "sharedBoat.h"

#define FUSION_ORIENTATION_TASK "[FORIENT-]"
#define FUSION_ORIENTATION_TASK_PRIORITY osPriorityNormal

#ifdef CM7_FUSION_ORIENTATION_DEBUG
#define FUSION_ORIENTATION_TASK_HEAP (RTOS_MIN_STACK+256)
#define fusionOrientation_printf(fmt, ...) DEBUG_PRINTF(FUSION_ORIENTATION_TASK, fmt, ##__VA_ARGS__)
#else
#define FUSION_ORIENTATION_TASK_HEAP (RTOS_MIN_STACK+128)
#define fusionOrientation_printf(...) ((void)0)
#endif

extern TaskHandle_t fusionOrientation_task;

HardwareInit_e fusionOrientation_hardwareInit(void);
RtosInit_e fusionOrientation_rtosInit(void);
void fusionOrientation_handler(void *argument);

#endif // FUSION_ORIENTATION_H