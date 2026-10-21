#include "main.h"
#include "projdefs.h"
#include "system.h"
#include "controller.h"
#include "shared_configuration.h"
#include "ipc.h"
#include "telemetry.h"

#include "servoShared.h"
#include "servoSail.h"
#include "servoRudder.h"

#include "sensorBattery.h"
#include "sensorEncoder.h"
#include "sensorGPS.h"
#include "sensorIMU.h"
#include "sensorWind.h"
#include "fusionOrientation.h"
#include "ipcCM7.h"

#include "semphr.h"

TaskHandle_t task_live;

HardwareInit_e live_hardwareInit();
RtosInit_e live_rtosInit();
void live_handler(void *argument);

/////////////////////////////////////////////////////////////////////////////////////////
// Init Functions
/////////////////////////////////////////////////////////////////////////////////////////

void hardware_init(void) {
    #ifdef CM7_LIVE_ENABLE
    live_hardwareInit();
    #endif
    #ifdef CM7_TELEMETRY_ENABLE
    telemetry_hardwareInit();
    #endif

    #ifdef CM7_SENSOR_BATTERY_ENABLE
    sensorBattery_hardwareInit();
    #endif
    #ifdef CM7_SENSOR_ENCODER_ENABLE
    sensorEncoder_hardwareInit();
    #endif
    #ifdef CM7_SENSOR_GPS_ENABLE
    sensorGPS_hardwareInit();
    #endif
    #ifdef CM7_SENSOR_IMU_ENABLE
    sensorIMU_hardwareInit();
    #endif
    #ifdef CM7_SENSOR_WIND_ENABLE
    sensorWind_hardwareInit();
    #endif
    #if defined(CM7_FUSION_ORIENTATION_ENABLE) && defined(CM7_SENSOR_IMU_ENABLE) && defined(CM7_SENSOR_GPS_ENABLE)
    fusionOrientation_hardwareInit();
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

    #ifdef IPC_ENABLE
    ipc_init(); // Common Shared Memory (SRAM4) init
    #endif
}

/**
  * @brief  Initialize the Real-Time Operating System and all of its components.
  * @retval None
  */
void rtos_init()
{
    int32_t totalStack = 0;
    
    #ifdef CM7_LIVE_ENABLE
    if (live_rtosInit() != RTOS_INIT_SUCCESS) { Error_Handler(); }
    if (xTaskCreate(live_handler, LIVE_TASK, LIVE_TASK_HEAP, NULL, LIVE_TASK_PRIORITY, &task_live) != pdPASS) { Error_Handler(); }
    totalStack += LIVE_TASK_HEAP;
    #endif
    #ifdef CM7_TELEMETRY_ENABLE
    if (telemetry_rtosInit() != RTOS_INIT_SUCCESS) { Error_Handler(); }
    if (xTaskCreate(telemetry_handler, TELEMETRY_TASK, TELEMETRY_TASK_HEAP, NULL, TELEMETRY_TASK_PRIORITY, &telemetry_task) != pdPASS) { Error_Handler(); }
    totalStack += TELEMETRY_TASK_HEAP;
    #endif

    #ifdef CM7_SENSOR_BATTERY_ENABLE
    if (sensorBattery_rtosInit() != RTOS_INIT_SUCCESS) { Error_Handler(); }
    if (xTaskCreate(sensorBattery_handler, SENSOR_BATTERY_TASK, SENSOR_BATTERY_TASK_HEAP, NULL, SENSOR_BATTERY_TASK_PRIORITY, &sensorBattery_task) != pdPASS) { Error_Handler(); }
    totalStack += SENSOR_BATTERY_TASK_HEAP;
    #endif
    #ifdef CM7_SENSOR_ENCODER_ENABLE
    if (sensorEncoder_rtosInit() != RTOS_INIT_SUCCESS) { Error_Handler(); }
    if (xTaskCreate(sensorEncoder_handler, SENSOR_ENCODER_TASK, SENSOR_ENCODER_TASK_HEAP, NULL, SENSOR_ENCODER_TASK_PRIORITY, &sensorEncoder_task) != pdPASS) { Error_Handler(); }
    totalStack += SENSOR_ENCODER_TASK_HEAP;
    #endif
    #ifdef CM7_SENSOR_GPS_ENABLE
    if (sensorGPS_rtosInit() != RTOS_INIT_SUCCESS) { Error_Handler(); }
    if (xTaskCreate(sensorGPS_handler, SENSOR_GPS_TASK, SENSOR_GPS_TASK_HEAP, NULL, SENSOR_GPS_TASK_PRIORITY, &sensorGPS_task) != pdPASS) { Error_Handler(); }
    totalStack += SENSOR_GPS_TASK_HEAP;
    #endif
    #ifdef CM7_SENSOR_IMU_ENABLE
    if (sensorIMU_rtosInit() != RTOS_INIT_SUCCESS) { Error_Handler(); }
    if (xTaskCreate(sensorIMU_handler, SENSOR_IMU_TASK, SENSOR_IMU_TASK_HEAP, NULL, SENSOR_IMU_TASK_PRIORITY, &sensorIMU_task) != pdPASS) { Error_Handler(); }
    totalStack += SENSOR_IMU_TASK_HEAP;
    #endif
    #ifdef CM7_SENSOR_WIND_ENABLE
    if (sensorWind_rtosInit() != RTOS_INIT_SUCCESS) { Error_Handler(); }
    if (xTaskCreate(sensorWind_handler, SENSOR_WIND_TASK, SENSOR_WIND_TASK_HEAP, NULL, SENSOR_WIND_TASK_PRIORITY, &sensorWind_task) != pdPASS) { Error_Handler(); }
    totalStack += SENSOR_WIND_TASK_HEAP;
    #endif
    #if defined(CM7_FUSION_ORIENTATION_ENABLE) && defined(CM7_SENSOR_IMU_ENABLE) && defined(CM7_SENSOR_GPS_ENABLE)
    if (fusionOrientation_rtosInit() != RTOS_INIT_SUCCESS) { Error_Handler(); }
    if (xTaskCreate(fusionOrientation_handler, FUSION_ORIENTATION_TASK, FUSION_ORIENTATION_TASK_HEAP, NULL, FUSION_ORIENTATION_TASK_PRIORITY, &fusionOrientation_task) != pdPASS) { Error_Handler(); }
    totalStack += FUSION_ORIENTATION_TASK_HEAP;
    #endif

    #ifdef CM7_DEBUG
    printf("Total stack allocated: %u B / %u B\r\n", (unsigned)(totalStack * 4), (unsigned)configTOTAL_HEAP_SIZE);
    #endif
}

/////////////////////////////////////////////////////////////////////////////////////////
// Live Functions
/////////////////////////////////////////////////////////////////////////////////////////

HardwareInit_e live_hardwareInit()
{
    live_printf("live_hardwareInit start");

    live_printf("    - init GPIO pins");
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    live_printf("live_hardwareInit complete");
    return HARDWARE_INIT_SUCCESS;
}

RtosInit_e live_rtosInit()
{
    live_printf("live_rtosInit start");
    live_printf("live_rtosInit complete");
    return RTOS_INIT_SUCCESS;
}

void live_handler(void *argument)
{
    live_printf("live_handler start");
    for(;;)
    {
        #ifdef CM7_LIVE_DEBUG_STACK
        
        live_printf("StackHW (Live): %u", (unsigned)uxTaskGetStackHighWaterMark(NULL));
        #ifdef CM7_TELEMETRY_ENABLE
        live_printf("StackHW (telemetry)=%u", (unsigned)uxTaskGetStackHighWaterMark(telemetry_task));
        #endif

        #ifdef CM7_SENSOR_BATTERY_ENABLE
        live_printf("StackHW (sensorBattery)=%u", (unsigned)uxTaskGetStackHighWaterMark(sensorBattery_task));
        #endif
        #ifdef CM7_SENSOR_ENCODER_ENABLE
        live_printf("StackHW (sensorEncoder)=%u", (unsigned)uxTaskGetStackHighWaterMark(sensorEncoder_task));
        #endif
        #ifdef CM7_SENSOR_GPS_ENABLE
        live_printf("StackHW (sensorGPS)=%u", (unsigned)uxTaskGetStackHighWaterMark(sensorGPS_task));
        #endif
        #ifdef CM7_SENSOR_IMU_ENABLE
        live_printf("StackHW (sensorIMU)=%u", (unsigned)uxTaskGetStackHighWaterMark(sensorIMU_task));
        #endif
        #ifdef CM7_SENSOR_WIND_ENABLE
        live_printf("StackHW (sensorWind)=%u", (unsigned)uxTaskGetStackHighWaterMark(sensorWind_task));
        #endif
        #if defined(CM7_FUSION_ORIENTATION_ENABLE) && defined(CM7_SENSOR_IMU_ENABLE) && defined(CM7_SENSOR_GPS_ENABLE)
        live_printf("StackHW (fusionOrientation)=%u", (unsigned)uxTaskGetStackHighWaterMark(fusionOrientation_task));
        #endif

        #endif

        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
        vTaskDelay(pdMS_TO_TICKS(1000));
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
