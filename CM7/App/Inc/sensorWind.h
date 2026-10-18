#ifndef SENSOR_WIND_H
#define SENSOR_WIND_H

#include "system.h"
#include "sharedSensor.h"

#define SENSOR_WIND_TASK "[SWIND---]"
#define SENSOR_WIND_TASK_PRIORITY osPriorityAboveNormal

#ifdef CM7_SENSOR_WIND_DEBUG
#define SENSOR_WIND_TASK_HEAP 300
#define sensorWind_printf(fmt, ...) DEBUG_PRINTF(SENSOR_WIND_TASK, fmt, ##__VA_ARGS__)
#else
#define SENSOR_WIND_TASK_HEAP 100
#define sensorWind_printf(...) ((void)0)
#endif

#define SENSOR_WIND_FREQUENCY 1u // Hz, This is mostly limited by the UART timeout, max is about 30Hz
#define SENSOR_WIND_WINDOW 10u // num readings
#define SENSOR_WIND_STALE 100u // ms before data is considered stale

extern TaskHandle_t sensorWind_task;

HardwareInit_e sensorWind_hardwareInit(void);
RtosInit_e sensorWind_rtosInit(void);
void sensorWind_handler(void *argument);

bool sensorWind_getMetadata(MetadataWind_t *metadata);

#endif // SENSOR_WIND_H