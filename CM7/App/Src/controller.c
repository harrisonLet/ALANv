#include "system.h"
#include "controller.h"

#include "servoSail.h"
#include "servoRudder.h"
#include "sensorWind.h"
#include "sensorMagnetometer.h"
#include "sensorEncoder.h"
#include "sensorGPS.h"

#include "semphr.h"

#define LIVE_ENABLE
#define SERVO_SAIL_ENABLE
#define SERVO_RUDDER_ENABLE
#define SENSOR_WIND_ENABLE
#define SENSOR_MAGNETOMETER_ENABLE
#define SENSOR_ENCODER_ENABLE
#define SENSOR_GPS_ENABLE

TaskHandle_t task_live;
TaskHandle_t task_button;
SemaphoreHandle_t semphr_button;

void live_hardwareInit();
void live_handler(void *argument);
void button_hardwareInit();
void button_handler(void *argument);

/////////////////////////////////////////////////////////////////////////////////////////
// Init Functions
/////////////////////////////////////////////////////////////////////////////////////////

void hardware_init(void) {
    #ifdef LIVE_ENABLE
    live_hardwareInit();
    button_hardwareInit();
    #endif

    #ifdef SERVO_SAIL_ENABLE
    servoSail_hardwareInit();
    #endif
    
    #ifdef SERVO_RUDDER_ENABLE
    servoRudder_hardwareInit();
    #endif

    #ifdef SENSOR_WIND_ENABLE
    sensorWind_hardwareInit();
    #endif

    #ifdef SENSOR_MAGNETOMETER_ENABLE
    sensorMagnetometer_hardwareInit();
    #endif

    #ifdef SENSOR_ENCODER_ENABLE
    sensorEncoder_hardwareInit();
    #endif

    #ifdef SENSOR_GPS_ENABLE
    sensorGPS_hardwareInit();
    #endif
}

/**
  * @brief  Initialize the Real-Time Operating System and all of its components.
  * @retval None
  */
void rtos_init()
{
    #ifdef LIVE_ENABLE
    if ((semphr_button = xSemaphoreCreateBinary()) == NULL) { Error_Handler(); }
    if (xTaskCreate(live_handler,               "liveTask",               64,  NULL, osPriorityNormal,      &task_live)               != pdPASS) { Error_Handler(); }
    if (xTaskCreate(button_handler,             "buttonTask",             64,  NULL, osPriorityNormal,      &task_button)             != pdPASS) { Error_Handler(); }
    #endif
    
    #ifdef SERVO_SAIL_ENABLE 
    if (xTaskCreate(servoSail_handler,          "servoSailTask",          128, NULL, osPriorityNormal,      &task_servoSail)          != pdPASS) { Error_Handler(); }
    #endif
    
    #ifdef SERVO_RUDDER_ENABLE
    if (xTaskCreate(servoRudder_handler,        "servoRudderTask",        128, NULL, osPriorityNormal,      &task_servoRudder)        != pdPASS) { Error_Handler(); }
    #endif

    #ifdef SENSOR_WIND_ENABLE
    if (xTaskCreate(sensorWind_handler,         "sensorWindTask",         512, NULL, osPriorityAboveNormal, &task_sensorWind)         != pdPASS) { Error_Handler(); }
    #endif

    #ifdef SENSOR_MAGNETOMETER_ENABLE
    if (xTaskCreate(sensorMagnetometer_handler, "sensorMagnetometerTask", 128, NULL, osPriorityAboveNormal, &task_sensorMagnetometer) != pdPASS) { Error_Handler(); }
    #endif

    #ifdef SENSOR_ENCODER_ENABLE
    if (xTaskCreate(sensorEncoder_handler,      "sensorEncoderTask",      256, NULL, osPriorityAboveNormal, &task_sensorEncoder)      != pdPASS) { Error_Handler(); }
    #endif

    #ifdef SENSOR_GPS_ENABLE
    if (xTaskCreate(sensorGPS_handler,          "sensorGPSTask",          512, NULL, osPriorityAboveNormal, &task_sensorGPS)          != pdPASS) { Error_Handler(); }
    #endif
}

/////////////////////////////////////////////////////////////////////////////////////////
// Live Functions
/////////////////////////////////////////////////////////////////////////////////////////

void live_hardwareInit()
{
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void live_handler(void *argument)
{
    for(;;)
    {
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
        vTaskDelay(1000 * portTICK_PERIOD_MS);
    }
}



void button_hardwareInit()
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 15, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void button_handler(void *argument)
{
    for(;;)
    {
        if (xSemaphoreTake(semphr_button, portMAX_DELAY) == pdTRUE)
        {
            printf("Button pressed!\r\n");
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); 
            vTaskDelay(1000 * portTICK_PERIOD_MS);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
        }
    }
}

void EXTI15_10_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_13)
  {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(semphr_button, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}