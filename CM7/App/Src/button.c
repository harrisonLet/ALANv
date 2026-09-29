#include "main.h"
#include "button.h"

TaskHandle_t task_button;
SemaphoreHandle_t semphr_button;

/**
  * Initialize the hardware.
  */
void button_hardwareInit()
{
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

/**
  * Handler for the task.
  */
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