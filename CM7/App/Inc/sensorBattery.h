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
#define SENSOR_BATTERY_TASK_HEAP 100 // ~32B of extra
#define sensorBattery_printf(...) ((void)0)
#endif

#define SENSOR_BATTERY_FREQUENCY 1u // Hz
#define SENSOR_BATTERY_WINDOW 2u // num readings
#define SENSOR_BATTERY_STALE 5000u // ms before data is considered stale
_Static_assert(SENSOR_BATTERY_FREQUENCY > 0u && SENSOR_BATTERY_FREQUENCY <= 500u, "SENSOR_BATTERY_FREQUENCY must be 1-500Hz");
#if (SYSTEM_TICK_RATE_HZ % SENSOR_BATTERY_FREQUENCY) != 0u
#warning "SENSOR_BATTERY_FREQUENCY does not divide the tick rate, the period will be rounded down"
#endif

extern TaskHandle_t sensorBattery_task;

HardwareInit_e sensorBattery_hardwareInit(void);
RtosInit_e sensorBattery_rtosInit(void);
void sensorBattery_handler(void *argument);

bool sensorBattery_getMetadata(MetadataBattery_t *metadata);

#endif // SENSOR_BATTERY_H