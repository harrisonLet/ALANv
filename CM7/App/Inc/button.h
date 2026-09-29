
extern TaskHandle_t task_button;
extern SemaphoreHandle_t semphr_button;

void button_hardwareInit();
void button_handler(void *argument);