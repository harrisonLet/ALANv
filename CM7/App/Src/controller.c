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
#include "arbiterRudder.h"
#include "pidHeading.h"
#include "arbiterTail.h"
#include "pidSail.h"
#include "processLSP.h"

#include "sensorBattery.h"
#include "sensorEncoder.h"
#include "sensorGPS.h"
#include "sensorIMU.h"
#include "sensorWind.h"
#include "fusionOrientation.h"
#include "ipcCM7.h"
#include "console.h"

#include "semphr.h"

#define MAX_TASKS 100

TaskHandle_t task_live;

static TaskHandle_t all_tasks[MAX_TASKS];
static uint16_t all_stack[MAX_TASKS]; // stack size in words, for reporting usage
static int task_count;
#ifdef CM7_LIVE_DEBUG_STACK
volatile uint16_t stackHWM[MAX_TASKS];
#endif

HardwareInit_e live_hardwareInit();
RtosInit_e live_rtosInit();
void live_handler(void *argument);

/////////////////////////////////////////////////////////////////////////////////////////
// Init Functions
/////////////////////////////////////////////////////////////////////////////////////////

void hardware_init(void) {
    #ifdef CM7_LIVE_ENABLE
    cm7_printf("live_hardwareInit");
    live_hardwareInit();
    #endif
    #ifdef CM7_TELEMETRY_ENABLE
    cm7_printf("telemetry_hardwareInit");
    telemetry_hardwareInit();
    #endif

    #ifdef CM7_SENSOR_BATTERY_ENABLE
    cm7_printf("sensorBattery_hardwareInit");
    sensorBattery_hardwareInit();
    #endif
    #ifdef CM7_SENSOR_ENCODER_ENABLE
    cm7_printf("sensorEncoder_hardwareInit");
    sensorEncoder_hardwareInit();
    #endif
    #ifdef CM7_SENSOR_GPS_ENABLE
    cm7_printf("sensorGPS_hardwareInit");
    sensorGPS_hardwareInit();
    #endif
    #ifdef CM7_SENSOR_IMU_ENABLE
    cm7_printf("sensorIMU_hardwareInit");
    sensorIMU_hardwareInit();
    #endif
    #ifdef CM7_SENSOR_WIND_ENABLE
    cm7_printf("sensorWind_hardwareInit");
    sensorWind_hardwareInit();
    #endif
    #if defined(CM7_FUSION_ORIENTATION_ENABLE) && defined(CM7_SENSOR_IMU_ENABLE) && defined(CM7_SENSOR_GPS_ENABLE)
    cm7_printf("fusionOrientation_hardwareInit");
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

    #if defined(CM7_ARBITER_RUDDER_ENABLE) && defined(CM7_SERVO_RUDDER_ENABLE)
    arbiterRudder_hardwareInit();
    #endif

    #if defined(CM7_PID_HEADING_ENABLE) && defined(CM7_ARBITER_RUDDER_ENABLE)
    pidHeading_hardwareInit();
    #endif

    #if defined(CM7_ARBITER_TAIL_ENABLE) && defined(CM7_SERVO_SAIL_ENABLE)
    arbiterTail_hardwareInit();
    #endif

    #if defined(CM7_PID_SAIL_ENABLE) && defined(CM7_ARBITER_TAIL_ENABLE)
    pidSail_hardwareInit();
    #endif

    #if defined(CM7_PROCESS_LSP_ENABLE) && defined(CM7_PID_SAIL_ENABLE)
    processLSP_hardwareInit();
    #endif

    #ifdef CM7_CONSOLE_ENABLE
    console_hardwareInit();
    #endif

    #ifdef IPC_ENABLE
    cm7_printf("ipc_init");
    ipc_init(); // Common Shared Memory (SRAM4) init
    #endif
}

/**
  * @brief  Initialize the Real-Time Operating System and all of its components.
  * @retval None
  */
static int32_t totalStack; // words, for the allocation report

// Creates a task and records it for the stack/heap monitor; halts if creation fails.
static void create_task(TaskFunction_t fn, const char *name, uint16_t stackWords, UBaseType_t priority, TaskHandle_t *handle)
{
    cm7_printf("create_task %s", name);
    if (task_count >= MAX_TASKS) { Error_Handler(); }
    if (xTaskCreate(fn, name, stackWords, NULL, priority, handle) != pdPASS) { Error_Handler(); }
    totalStack += stackWords;
    all_tasks[task_count] = *handle;
    all_stack[task_count] = stackWords;
    task_count++;
}

void rtos_init()
{
    #ifdef CM7_LIVE_ENABLE
    cm7_printf("live_rtosInit");
    if (live_rtosInit() != RTOS_INIT_SUCCESS) { Error_Handler(); }
    create_task(live_handler, LIVE_TASK, LIVE_TASK_HEAP, LIVE_TASK_PRIORITY, &task_live);
    #endif
    #ifdef CM7_TELEMETRY_ENABLE
    cm7_printf("telemetry_rtosInit");
    if (telemetry_rtosInit() != RTOS_INIT_SUCCESS) { Error_Handler(); }
    create_task(telemetry_handler, TELEMETRY_TASK, TELEMETRY_TASK_HEAP, TELEMETRY_TASK_PRIORITY, &telemetry_task);
    #endif

    #ifdef CM7_SENSOR_BATTERY_ENABLE
    cm7_printf("sensorBattery_rtosInit");
    if (sensorBattery_rtosInit() != RTOS_INIT_SUCCESS) { Error_Handler(); }
    create_task(sensorBattery_handler, SENSOR_BATTERY_TASK, SENSOR_BATTERY_TASK_HEAP, SENSOR_BATTERY_TASK_PRIORITY, &sensorBattery_task);
    #endif
    #ifdef CM7_SENSOR_ENCODER_ENABLE
    cm7_printf("sensorEncoder_rtosInit");
    if (sensorEncoder_rtosInit() != RTOS_INIT_SUCCESS) { Error_Handler(); }
    create_task(sensorEncoder_handler, SENSOR_ENCODER_TASK, SENSOR_ENCODER_TASK_HEAP, SENSOR_ENCODER_TASK_PRIORITY, &sensorEncoder_task);
    #endif
    #ifdef CM7_SENSOR_GPS_ENABLE
    cm7_printf("sensorGPS_rtosInit");
    if (sensorGPS_rtosInit() != RTOS_INIT_SUCCESS) { Error_Handler(); }
    create_task(sensorGPS_handler, SENSOR_GPS_TASK, SENSOR_GPS_TASK_HEAP, SENSOR_GPS_TASK_PRIORITY, &sensorGPS_task);
    #endif
    #ifdef CM7_SENSOR_IMU_ENABLE
    cm7_printf("sensorIMU_rtosInit");
    if (sensorIMU_rtosInit() != RTOS_INIT_SUCCESS) { Error_Handler(); }
    create_task(sensorIMU_handler, SENSOR_IMU_TASK, SENSOR_IMU_TASK_HEAP, SENSOR_IMU_TASK_PRIORITY, &sensorIMU_task);
    #endif
    #ifdef CM7_SENSOR_WIND_ENABLE
    cm7_printf("sensorWind_rtosInit");
    if (sensorWind_rtosInit() != RTOS_INIT_SUCCESS) { Error_Handler(); }
    create_task(sensorWind_handler, SENSOR_WIND_TASK, SENSOR_WIND_TASK_HEAP, SENSOR_WIND_TASK_PRIORITY, &sensorWind_task);
    #endif
    #if defined(CM7_FUSION_ORIENTATION_ENABLE) && defined(CM7_SENSOR_IMU_ENABLE) && defined(CM7_SENSOR_GPS_ENABLE)
    cm7_printf("fusionOrientation_rtosInit");
    if (fusionOrientation_rtosInit() != RTOS_INIT_SUCCESS) { Error_Handler(); }
    create_task(fusionOrientation_handler, FUSION_ORIENTATION_TASK, FUSION_ORIENTATION_TASK_HEAP, FUSION_ORIENTATION_TASK_PRIORITY, &fusionOrientation_task);
    #endif

    #if defined(CM7_ARBITER_RUDDER_ENABLE) && defined(CM7_SERVO_RUDDER_ENABLE)
    cm7_printf("arbiterRudder_rtosInit");
    if (arbiterRudder_rtosInit() != RTOS_INIT_SUCCESS) { Error_Handler(); }
    create_task(arbiterRudder_handler, ARBITER_RUDDER_TASK, ARBITER_RUDDER_TASK_HEAP, ARBITER_RUDDER_TASK_PRIORITY, &arbiterRudder_task);
    #endif

    #if defined(CM7_PID_HEADING_ENABLE) && defined(CM7_ARBITER_RUDDER_ENABLE)
    cm7_printf("pidHeading_rtosInit");
    if (pidHeading_rtosInit() != RTOS_INIT_SUCCESS) { Error_Handler(); }
    create_task(pidHeading_handler, PID_HEADING_TASK, PID_HEADING_TASK_HEAP, PID_HEADING_TASK_PRIORITY, &pidHeading_task);
    #endif

    #if defined(CM7_ARBITER_TAIL_ENABLE) && defined(CM7_SERVO_SAIL_ENABLE)
    cm7_printf("arbiterTail_rtosInit");
    if (arbiterTail_rtosInit() != RTOS_INIT_SUCCESS) { Error_Handler(); }
    create_task(arbiterTail_handler, ARBITER_TAIL_TASK, ARBITER_TAIL_TASK_HEAP, ARBITER_TAIL_TASK_PRIORITY, &arbiterTail_task);
    #endif

    #if defined(CM7_PID_SAIL_ENABLE) && defined(CM7_ARBITER_TAIL_ENABLE)
    cm7_printf("pidSail_rtosInit");
    if (pidSail_rtosInit() != RTOS_INIT_SUCCESS) { Error_Handler(); }
    create_task(pidSail_handler, PID_SAIL_TASK, PID_SAIL_TASK_HEAP, PID_SAIL_TASK_PRIORITY, &pidSail_task);
    #endif

    #if defined(CM7_PROCESS_LSP_ENABLE) && defined(CM7_PID_SAIL_ENABLE)
    cm7_printf("processLSP_rtosInit");
    if (processLSP_rtosInit() != RTOS_INIT_SUCCESS) { Error_Handler(); }
    create_task(processLSP_handler, PROCESS_LSP_TASK, PROCESS_LSP_TASK_HEAP, PROCESS_LSP_TASK_PRIORITY, &processLSP_task);
    #endif

    #ifdef CM7_CONSOLE_ENABLE
    create_task(console_handler, CONSOLE_TASK, CONSOLE_TASK_HEAP, CONSOLE_TASK_PRIORITY, &task_console);
    #endif

    #ifdef CM7_DEBUG
    cm7_printf("Total stack allocated: %u B / %u B\r\n", (unsigned)(totalStack * 4), (unsigned)configTOTAL_HEAP_SIZE);
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
        live_printf("\x1b[2J\x1b[H");
        for (int i = 0; i < task_count; i++) {
            stackHWM[i] = (uint16_t)uxTaskGetStackHighWaterMark(all_tasks[i]);
            live_printf("%s stack used (peak): %u / %u words", pcTaskGetName(all_tasks[i]),
                        (unsigned)(all_stack[i] - stackHWM[i]), (unsigned)all_stack[i]);
        }
        live_printf("min free heap: %u B", (unsigned)xPortGetMinimumEverFreeHeapSize());
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
