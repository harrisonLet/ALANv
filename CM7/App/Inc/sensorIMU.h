#ifndef SENSOR_IMU_H
#define SENSOR_IMU_H

#include "system.h"
#include "sharedSensor.h"

#define SENSOR_IMU_TASK "[SIMU----]"
#define SENSOR_IMU_PRIORITY osPriorityAboveNormal

#ifdef CM7_SENSOR_IMU_DEBUG
#define SENSOR_IMU_TASK_HEAP 300
#define sensorIMU_printf(fmt, ...) DEBUG_PRINTF(SENSOR_IMU_TASK, fmt, ##__VA_ARGS__)
#else
#define SENSOR_IMU_TASK_HEAP 100
#define sensorIMU_printf(...) ((void)0)
#endif

#define SENSOR_IMU_FREQUENCY 100u // Hz, this value is primarily limited by the i2c bus speed, max: ~300Hz
#define SENSOR_IMU_WINDOW 10u // num readings
#define SENSOR_IMU_STALE 100u // ms before data is considered stale

extern TaskHandle_t sensorIMU_task;

HardwareInit_e sensorIMU_hardwareInit(void);
RtosInit_e sensorIMU_rtosInit(void);
void sensorIMU_handler(void *argument);

bool sensorIMU_getMetadata(MetadataIMU_t *metadata);

#endif // SENSOR_IMU_H