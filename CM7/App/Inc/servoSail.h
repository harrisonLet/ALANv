
extern TaskHandle_t task_servoSail;

void servoSail_hardwareInit();
void servoSail_handler(void *argument);
void servoSail_setAngle(int16_t angle);