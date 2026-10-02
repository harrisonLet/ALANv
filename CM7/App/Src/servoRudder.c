#include "system.h"

#include "servoRudder.h"
#include "servoShared.h"


#define SERVO_MIN_ANGLE -90
#define SERVO_MAX_ANGLE 90
#define SERVO_MIN_PULSE 500
#define SERVO_MAX_PULSE 2500
#define SERVO_CENTER_PULSE ((SERVO_MIN_PULSE + SERVO_MAX_PULSE) / 2)



uint16_t servoRudder_angleToPulse(int16_t angle);
int16_t servoRudder_pulseToAngle(uint16_t pulse);



void servoRudder_hardwareInit(void)
{
  __HAL_RCC_GPIOE_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pin = GPIO_PIN_11; 
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  TIM_OC_InitTypeDef sConfigOC = {0};
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = SERVO_CENTER_PULSE;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&servo_tim, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) { Error_Handler(); }

  if (HAL_TIM_PWM_Start(&servo_tim, TIM_CHANNEL_2) != HAL_OK) { Error_Handler(); }

  /* Sweep to extremes on startup so you can see if servo responds */
  printf("%s Starting motion sweep\r\n", SERVO_RUDDER_TASK);
  int16_t test_angles[] = {-5, 5, 0};
  for (int i = 0; i < 3; i++) {
    servoRudder_setAngle(test_angles[i]);
    HAL_Delay(1000);
  }

  printf("%s Initialized, centered at %d degrees\r\n", SERVO_RUDDER_TASK, servoRudder_pulseToAngle(SERVO_CENTER_PULSE));
}



void servoRudder_setAngle(int16_t angle)
{
  uint16_t pulse_length = servoRudder_angleToPulse(angle);
  __HAL_TIM_SET_COMPARE(&servo_tim, TIM_CHANNEL_2, pulse_length);
  printf("%s Setting Servo Angle To: %d\r\n", SERVO_RUDDER_TASK, angle);
}

uint16_t servoRudder_angleToPulse(int16_t angle) {
  if (angle < SERVO_MIN_ANGLE) angle = SERVO_MIN_ANGLE;
  if (angle > SERVO_MAX_ANGLE) angle = SERVO_MAX_ANGLE;
  return SERVO_MIN_PULSE + ((SERVO_MAX_PULSE - SERVO_MIN_PULSE) * (angle - SERVO_MIN_ANGLE)) / (SERVO_MAX_ANGLE - SERVO_MIN_ANGLE);
}

int16_t servoRudder_pulseToAngle(uint16_t pulse) {
  if (pulse < SERVO_MIN_PULSE) pulse = SERVO_MIN_PULSE;
  if (pulse > SERVO_MAX_PULSE) pulse = SERVO_MAX_PULSE;
  return SERVO_MIN_ANGLE + ((SERVO_MAX_ANGLE - SERVO_MIN_ANGLE) * (pulse - SERVO_MIN_PULSE)) / (SERVO_MAX_PULSE - SERVO_MIN_PULSE);
}
