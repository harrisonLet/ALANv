
#include "main.h"
#include "servoSail.h"

#define SERVO_CLOCK_FREQUENCY_HZ 1000000
#define SERVO_PWM_FREQUENCY_HZ 50

#define SERVO_MIN_ANGLE -135
#define SERVO_MAX_ANGLE 135
#define SERVO_MIN_PULSE 500
#define SERVO_MAX_PULSE 2500

TaskHandle_t task_servoSail;
TIM_HandleTypeDef servo_tim1;

/**
  * Initialize the hardware.
  */
void servoSail_hardwareInit()
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pin = GPIO_PIN_9; 
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  // The board's clock rate is 64MHz, and we want a 50Hz signal,
  // so we set the prescaler to 63 (64MHz/64 = 1MHz) and the
  // period to 19999 (1MHz/20,000 = 50Hz)
  servo_tim1.Instance = TIM1;
  servo_tim1.Init.Prescaler = (STM32H755_CLOCK_RATE / SERVO_CLOCK_FREQUENCY_HZ) - 1;
  servo_tim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  servo_tim1.Init.Period = (SERVO_CLOCK_FREQUENCY_HZ / SERVO_PWM_FREQUENCY_HZ) - 1;
  servo_tim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  servo_tim1.Init.RepetitionCounter = 0;
  servo_tim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&servo_tim1) != HAL_OK) { Error_Handler(); }

  TIM_OC_InitTypeDef sConfigOC = {0};
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = (SERVO_MAX_PULSE+SERVO_MIN_PULSE)/2;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&servo_tim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) { Error_Handler(); }

  if (HAL_TIM_PWM_Start(&servo_tim1, TIM_CHANNEL_1) != HAL_OK) { Error_Handler(); }
}

void servoSail_handler(void *argument)
{
    for(;;)
    {
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for demonstration purposes
    }
}

void servoSail_setAngle(int16_t angle)
{
    if (angle < SERVO_MIN_ANGLE) angle = SERVO_MIN_ANGLE;
    if (angle > SERVO_MAX_ANGLE) angle = SERVO_MAX_ANGLE;
    uint16_t pulse_length = SERVO_MIN_PULSE + ((SERVO_MAX_PULSE - SERVO_MIN_PULSE) * (angle - SERVO_MIN_ANGLE)) / (SERVO_MAX_ANGLE - SERVO_MIN_ANGLE);
    __HAL_TIM_SET_COMPARE(&servo_tim1, TIM_CHANNEL_1, pulse_length);
    printf("Setting Servo Angle To: %d\r\n", angle);
}