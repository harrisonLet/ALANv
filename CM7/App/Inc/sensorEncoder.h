#ifndef SENSOR_ENCODER_H
#define SENSOR_ENCODER_H

#include "system.h"
#include "sharedSensor.h"
#include "sharedSail.h"

#define SENSOR_ENCODER_TASK "[SENCODER]"
#define SENSOR_ENCODER_TASK_PRIORITY osPriorityNormal

#ifdef CM7_SENSOR_ENCODER_DEBUG
#define SENSOR_ENCODER_TASK_HEAP 220 // ~64B of extra
#define sensorEncoder_printf(fmt, ...) DEBUG_PRINTF(SENSOR_ENCODER_TASK, fmt, ##__VA_ARGS__)
#else
#define SENSOR_ENCODER_TASK_HEAP 120 // ~32B of extra
#define sensorEncoder_printf(...) ((void)0)
#endif

#define SENSOR_ENCODER_FREQUENCY 100u // Hz, this value is primarily limited by the i2c bus speed, max: ~300Hz
#define SENSOR_ENCODER_WINDOW 10u // num readings
#define SENSOR_ENCODER_STALE 100u // ms before data is considered stale
_Static_assert(SENSOR_ENCODER_FREQUENCY > 0u && SENSOR_ENCODER_FREQUENCY <= 500u, "SENSOR_ENCODER_FREQUENCY must be 1-500Hz");
#if (SYSTEM_TICK_RATE_HZ % SENSOR_ENCODER_FREQUENCY) != 0u
#warning "SENSOR_ENCODER_FREQUENCY does not divide the tick rate, the period will be rounded down"
#endif

extern TaskHandle_t sensorEncoder_task;

HardwareInit_e sensorEncoder_hardwareInit(void);
RtosInit_e sensorEncoder_rtosInit(void);
void sensorEncoder_handler(void *argument);

bool sensorEncoder_getMetadata(MetadataEncoder_t *metadata);

#endif // SENSOR_ENCODER_H
