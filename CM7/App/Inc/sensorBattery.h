#ifndef SENSOR_BATTERY_H
#define SENSOR_BATTERY_H

#define SENSOR_BATTERY_TASK "[SENSOR_BATTERY]"
#define SENSOR_BATTERY_FREQUENCY 1u // Hz
#define SENSOR_BATTERY_WINDOW 4u // num readings
#define SENSOR_BATTERY_STALE 1000u // ms before data is considered stale

extern TaskHandle_t sensorBattery_task;

void sensorBattery_hardwareInit(void);
void sensorBattery_handler(void *argument);

bool sensorBattery_getTelemetry(TelemetryBattery_t *telemetry);
bool sensorBattery_getMetadata(MetadataBattery_t *metadata);

#endif // SENSOR_BATTERY_H