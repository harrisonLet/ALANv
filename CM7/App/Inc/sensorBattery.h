#ifndef SENSOR_BATTERY_H
#define SENSOR_BATTERY_H

#include "system.h"
#include "sharedBattery.h"
#include "sharedSensor.h"

#define SENSOR_BATTERY_TASK "[SBATTERY]"
#define SENSOR_BATTERY_TASK_PRIORITY osPriorityLow

#ifdef CM7_SENSOR_BATTERY_DEBUG
#define SENSOR_BATTERY_TASK_HEAP 230 // ~64B of extra
#define sensorBattery_printf(fmt, ...) DEBUG_PRINTF(SENSOR_BATTERY_TASK, fmt, ##__VA_ARGS__)
#else
#define SENSOR_BATTERY_TASK_HEAP 130 // ~64B of extra
#define sensorBattery_printf(...) ((void)0)
#endif

#define SENSOR_BATTERY_FREQUENCY 1u // Hz
#define SENSOR_BATTERY_WINDOW 2u // num readings
#define SENSOR_BATTERY_STALE 5000u // ms before data is considered stale

extern TaskHandle_t sensorBattery_task;

HardwareInit_e sensorBattery_hardwareInit(void);
RtosInit_e sensorBattery_rtosInit(void);
void sensorBattery_handler(void *argument);

bool sensorBattery_getMetadata(MetadataBattery_t *metadata);

#endif // SENSOR_BATTERY_H