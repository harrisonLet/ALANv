#include "main.h"
#include "system.h"
#include "controller.h"

#include "servoShared.h"
#include "servoSail.h"
#include "servoRudder.h"

#include "sensorBattery.h"
#include "sensorWind.h"
#include "sensorMagnetometer.h"
#include "sensorEncoder.h"
#include "sensorGPS.h"

#include "semphr.h"

TaskHandle_t task_live;

void live_hardwareInit();
void live_handler(void *argument);

/////////////////////////////////////////////////////////////////////////////////////////
// Init Functions
/////////////////////////////////////////////////////////////////////////////////////////

void hardware_init(void) {
    #ifdef CM7_LIVE_ENABLE
    live_hardwareInit();
    #endif

    #ifdef CM7_SENSORPOLL_BATTERY_ENABLE
    sensorBattery_hardwareInit();
    #endif

    #if defined(CM7_SERVO_SAIL_ENABLE) || defined(CM7_SERVO_RUDDER_ENABLE)
    servoShared_hardwareInit();
    #endif

    #ifdef CM7_SERVO_SAIL_ENABLE
    servoSail_hardwareInit();
    #endif
    
    #ifdef CM7_SERVO_RUDDER_ENABLE
    servoRudder_hardwareInit();
    #endif

    #ifdef CM7_SENSOR_WIND_ENABLE
    sensorWind_hardwareInit();
    #endif

    #ifdef CM7_SENSOR_MAGNETOMETER_ENABLE
    sensorMagnetometer_hardwareInit();
    #endif

    #ifdef CM7_SENSOR_ENCODER_ENABLE
    sensorEncoder_hardwareInit();
    #endif

    #ifdef CM7_SENSOR_GPS_ENABLE
    sensorGPS_hardwareInit();
    #endif
}

/**
  * @brief  Initialize the Real-Time Operating System and all of its components.
  * @retval None
  */
void rtos_init()
{
    #ifdef CM7_LIVE_ENABLE
    if (xTaskCreate(live_handler, LIVE_TASK, LIVE_TASK_HEAP, NULL, LIVE_TASK_PRIORITY, &task_live) != pdPASS) { Error_Handler(); }
    #endif
    #ifdef CM7_SENSORPOLL_BATTERY_ENABLE
    if (xTaskCreate(sensorBattery_handler, SENSOR_BATTERY_TASK, SENSOR_BATTERY_TASK_HEAP, NULL, SENSOR_BATTERY_TASK_PRIORITY, &sensorBattery_task) != pdPASS) { Error_Handler(); }
    #endif
}

/////////////////////////////////////////////////////////////////////////////////////////
// Live Functions
/////////////////////////////////////////////////////////////////////////////////////////

void live_hardwareInit()
{
    live_printf("Initializing the liveliness LED");

    live_printf("    - init GPIO pins");
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    live_printf("Initialization of live complete.");
}

void live_handler(void *argument)
{
    live_printf("Starting liveliness task.");
    for(;;)
    {
        #ifdef CM7_LIVE_DEBUG_STACK
        live_printf("StackHW (Live): %u", (unsigned)uxTaskGetStackHighWaterMark(NULL));
        #ifdef CM7_SENSORPOLL_BATTERY_ENABLE
        live_printf("StackHW (sensorBattery)=%u", (unsigned)uxTaskGetStackHighWaterMark(sensorBattery_task));
        #endif
        #endif

        #ifdef CM7_LIVE_DEBUG_TELEMETRY
        #ifdef CM7_SENSORPOLL_BATTERY_ENABLE
        {
            TelemetryBattery_t telemetry;
            sensorBattery_getTelemetry(&telemetry);
            char telemetryStr[TELEMETRY_BATTERY_STR_SIZE];
            live_printf("%s", TelemetryBattery_toString(&telemetry, telemetryStr));
        }
        #endif
        #endif

        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
        vTaskDelay(1000 * portTICK_PERIOD_MS);
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
// FreeRTOS Fault Hooks
/////////////////////////////////////////////////////////////////////////////////////////

// Inspect this in the debugger after a stack overflow to see which task caused it.
volatile char fault_taskName[configMAX_TASK_NAME_LEN];

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    taskDISABLE_INTERRUPTS();
    strncpy((char *)fault_taskName, pcTaskName, configMAX_TASK_NAME_LEN - 1);
    __BKPT(0);
    for (;;) { }
}

void vApplicationMallocFailedHook(void)
{
    taskDISABLE_INTERRUPTS();
    __BKPT(0);
    for (;;) { }
}
