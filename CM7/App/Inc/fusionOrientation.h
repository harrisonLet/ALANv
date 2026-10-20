#ifndef FUSION_ORIENTATION_H
#define FUSION_ORIENTATION_H

#include "system.h"
#include "sharedBoat.h"

#define FUSION_ORIENTATION_TASK "[FORIENT-]"
#define FUSION_ORIENTATION_TASK_PRIORITY osPriorityNormal

#ifdef CM7_FUSION_ORIENTATION_DEBUG
#define FUSION_ORIENTATION_TASK_HEAP 230 // ~64B of extra
#define fusionOrientation_printf(fmt, ...) DEBUG_PRINTF(FUSION_ORIENTATION_TASK, fmt, ##__VA_ARGS__)
#else
#define FUSION_ORIENTATION_TASK_HEAP 100 // ~32B of extra
#define fusionOrientation_printf(...) ((void)0)
#endif

#define SENSOR_BATTERY_FREQUENCY 1u // Hz
#define SENSOR_BATTERY_WINDOW 2u // num readings
#define SENSOR_BATTERY_STALE 5000u // ms before data is considered stale

extern TaskHandle_t fusionOrientation_task;

HardwareInit_e fusionOrientation_hardwareInit(void);
RtosInit_e fusionOrientation_rtosInit(void);
void fusionOrientation_handler(void *argument);

#endif // FUSION_ORIENTATION_H