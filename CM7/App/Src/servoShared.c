#include "system.h"

#include "servoShared.h"

#define SERVO_CLOCK_FREQUENCY_HZ 1000000
#define SERVO_PWM_FREQUENCY_HZ 50

TIM_HandleTypeDef servo_tim;

void servoShared_hardwareInit(void)
{
    __HAL_RCC_TIM1_CLK_ENABLE();

    // The board's clock rate is 64MHz, and we want a 50Hz signal,
    // so we set the prescaler to 63 (64MHz/64 = 1MHz) and the
    // period to 19999 (1MHz/20,000 = 50Hz)
    servo_tim.Instance = TIM1;
    servo_tim.Init.Prescaler = (STM32H755_CLOCK_RATE / SERVO_CLOCK_FREQUENCY_HZ) - 1;
    servo_tim.Init.CounterMode = TIM_COUNTERMODE_UP;
    servo_tim.Init.Period = (SERVO_CLOCK_FREQUENCY_HZ / SERVO_PWM_FREQUENCY_HZ) - 1;
    servo_tim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    servo_tim.Init.RepetitionCounter = 0;
    servo_tim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&servo_tim) != HAL_OK) { Error_Handler(); }
}