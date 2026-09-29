
extern TaskHandle_t task_sensorMagnetometer;

void sensorMagnetometer_hardwareInit();
void sensorMagnetometer_handler(void *argument);
void sensorMagnetometer_scan();