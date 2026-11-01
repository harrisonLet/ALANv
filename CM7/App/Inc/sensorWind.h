#ifndef SENSOR_WIND_H
#define SENSOR_WIND_H

#include "system.h"
#include "sharedSensor.h"

#define SENSOR_WIND_TASK "[SWIND---]"
#define SENSOR_WIND_TASK_PRIORITY osPriorityNormal

#ifdef CM7_SENSOR_WIND_DEBUG
#define SENSOR_WIND_TASK_HEAP 230 // ~64B of extra
#define sensorWind_printf(fmt, ...) DEBUG_PRINTF(SENSOR_WIND_TASK, fmt, ##__VA_ARGS__)
#else
#define SENSOR_WIND_TASK_HEAP 100 // ~32B of extra
#define sensorWind_printf(...) ((void)0)
#endif

#define SENSOR_WIND_FREQUENCY 10u // Hz, This is mostly limited by the UART timeout, max is about 40Hz
#define SENSOR_WIND_WINDOW 10u // num readings
#define SENSOR_WIND_STALE 100u // ms before data is considered stale
_Static_assert(SENSOR_WIND_FREQUENCY > 0u && SENSOR_WIND_FREQUENCY <= 40u, "SENSOR_WIND_FREQUENCY must be 1-30Hz");
#if (SYSTEM_TICK_RATE_HZ % SENSOR_WIND_FREQUENCY) != 0u
#warning "SENSOR_WIND_FREQUENCY does not divide the tick rate, the period will be rounded down"
#endif

extern TaskHandle_t sensorWind_task;

HardwareInit_e sensorWind_hardwareInit(void);
RtosInit_e sensorWind_rtosInit(void);
void sensorWind_handler(void *argument);

bool sensorWind_getMetadata(MetadataWind_t *metadata);

#endif // SENSOR_WIND_H