#ifndef SENSOR_BATTERY_H
#define SENSOR_BATTERY_H

#include "sharedBattery.h"
#include "sharedSensor.h"

#define SENSOR_BATTERY_TASK "[SBATTERY]"
#define SENSOR_BATTERY_TASK_PRIORITY osPriorityNormal

#ifdef CM7_SENSOR_BATTERY_DEBUG
#define SENSOR_BATTERY_TASK_HEAP 350
#define sensorBattery_printf(fmt, ...) DEBUG_PRINTF(SENSOR_BATTERY_TASK, fmt, ##__VA_ARGS__)
#else
#define SENSOR_BATTERY_TASK_HEAP 100
#define sensorBattery_printf(...) ((void)0)
#endif

#define SENSOR_BATTERY_FREQUENCY 10u // Hz
#define SENSOR_BATTERY_WINDOW 4u // num readings
#define SENSOR_BATTERY_STALE 1000u // ms before data is considered stale

extern TaskHandle_t sensorBattery_task;

void sensorBattery_hardwareInit(void);
void sensorBattery_handler(void *argument);

bool sensorBattery_getTelemetry(TelemetryBattery_t *telemetry);
bool sensorBattery_getMetadata(MetadataBattery_t *metadata);

#endif // SENSOR_BATTERY_H