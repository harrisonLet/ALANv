#ifndef SENSOR_ENCODER_H
#define SENSOR_ENCODER_H

#include "sharedSensor.h"
#include "sharedSail.h"



extern TaskHandle_t task_sensorEncoder;

void sensorEncoder_hardwareInit();
void sensorEncoder_handler(void *argument);

#endif // SENSOR_ENCODER_H
