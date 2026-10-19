#ifndef SENSOR_GPS_H
#define SENSOR_GPS_H

#include "system.h"
#include "sharedSensor.h"

#define SENSOR_GPS_TASK "[SGPS----]"
#define SENSOR_GPS_TASK_PRIORITY osPriorityLow

#ifdef CM7_SENSOR_GPS_DEBUG
#define SENSOR_GPS_TASK_HEAP 350 // ~64B of extra, tested without a a GPS lock so it might be too small
#define sensorGPS_printf(fmt, ...) DEBUG_PRINTF(SENSOR_GPS_TASK, fmt, ##__VA_ARGS__)
#else
#define SENSOR_GPS_TASK_HEAP 170 // ~64B of extra, tested without a GPS lock so it might be too small
#define sensorGPS_printf(...) ((void)0)
#endif

#define SENSOR_GPS_FREQUENCY 1u // Hz, we don't actually use or set this value, the GPS module itself naturally functions at about 1Hz
#define SENSOR_GPS_WINDOW 3u // num readings
#define SENSOR_GPS_STALE 5000u // ms before data is considered stale

extern TaskHandle_t sensorGPS_task;

HardwareInit_e sensorGPS_hardwareInit(void);
RtosInit_e sensorGPS_rtosInit(void);
void sensorGPS_handler(void *argument);

bool sensorGPS_getMetadata(MetadataGPS_t *metadata);

#endif