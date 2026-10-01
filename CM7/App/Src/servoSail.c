#include "system.h"

#include "servoSail.h"
#include "servoShared.h"



#define SERVO_MIN_ANGLE -135
#define SERVO_MAX_ANGLE 135
#define SERVO_MIN_PULSE 500
#define SERVO_MAX_PULSE 2500
#define SERVO_CENTER_PULSE ((SERVO_MIN_PULSE + SERVO_MAX_PULSE) / 2)



/**
  * Initialize the hardware.
  */
void servoSail_hardwareInit()
{
  __HAL_RCC_GPIOE_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pin = GPIO_PIN_9; 
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  TIM_OC_InitTypeDef sConfigOC = {0};
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = (SERVO_MAX_PULSE+SERVO_MIN_PULSE)/2;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&servo_tim, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) { Error_Handler(); }

  if (HAL_TIM_PWM_Start(&servo_tim, TIM_CHANNEL_1) != HAL_OK) { Error_Handler(); }

  /* Sweep to extremes on startup so you can see if servo responds */
  __HAL_TIM_SET_COMPARE(&servo_tim, TIM_CHANNEL_1, SERVO_MIN_PULSE);
  HAL_Delay(1000);
  __HAL_TIM_SET_COMPARE(&servo_tim, TIM_CHANNEL_1, SERVO_MAX_PULSE);
  HAL_Delay(1000);
  __HAL_TIM_SET_COMPARE(&servo_tim, TIM_CHANNEL_1, SERVO_CENTER_PULSE);

  printf("[SERVO] Sail init OK, centered at %d us\r\n", SERVO_CENTER_PULSE);
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
    __HAL_TIM_SET_COMPARE(&servo_tim, TIM_CHANNEL_1, pulse_length);
    printf("Setting Servo Angle To: %d\r\n", angle);
}