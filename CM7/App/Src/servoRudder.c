#include "main.h"
#include "servoRudder.h"

TaskHandle_t task_servoRudder;

/**
  * Initialize the hardware.
  */
void servoRudder_hardwareInit() {}

/**
  * Handler for the task.
  */
void servoRudder_handler(void *argument) {
    for(;;) {
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for demonstration purposes
    }
}