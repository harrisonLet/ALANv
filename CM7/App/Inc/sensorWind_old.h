#ifndef SENSORWIND_H
#define SENSORWIND_H

#define SENSOR_WIND_TASK "[SENSOR_WIND]"

extern TaskHandle_t task_sensorWind;

void sensorWind_hardwareInit();
void sensorWind_handler(void *argument);

#endif