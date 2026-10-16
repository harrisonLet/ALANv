#ifndef SENSOR_ENCODER_H
#define SENSOR_ENCODER_H

#include "sharedSensor.h"
#include "sharedSail.h"

#define SENSOR_ENCODER_TASK "[SENCODER]"
#define SENSOR_ENCODER_TASK_PRIORITY osPriorityAboveNormal

#ifdef CM7_SENSOR_ENCODER_DEBUG
#define SENSOR_ENCODER_TASK_HEAP 400
#define sensorEncoder_printf(fmt, ...) DEBUG_PRINTF(SENSOR_ENCODER_TASK, fmt, ##__VA_ARGS__)
#else
#define SENSOR_ENCODER_TASK_HEAP 200
#define sensorEncoder_printf(...) ((void)0)
#endif

#define SENSOR_ENCODER_FREQUENCY 100u // Hz, this value is primarily limited by the i2c bus speed, max: ~300Hz
#define SENSOR_ENCODER_WINDOW 10u // num readings
#define SENSOR_ENCODER_STALE 100u // ms before data is considered stale

extern TaskHandle_t sensorEncoder_task;

void sensorEncoder_hardwareInit(void);
void sensorEncoder_handler(void *argument);

bool sensorEncoder_getMetadata(MetadataEncoder_t *metadata);

#endif // SENSOR_ENCODER_H
