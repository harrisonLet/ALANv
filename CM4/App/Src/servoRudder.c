#include "servoRudder.h"
#include "main.h"

/* PWM output: PB9, TIM4 Channel 4, CN7 morpho connector
 * 64 MHz HSI -> prescaler 63 -> 1 MHz tick -> period 19999 -> 50 Hz
 * Pulse range: 1000-2000 us maps to -135..+135 degrees */

#define SERVO_MIN_ANGLE  -135
#define SERVO_MAX_ANGLE   135
#define SERVO_MIN_PULSE  1000
#define SERVO_MAX_PULSE  2000
#define SERVO_CENTER_PULSE ((SERVO_MIN_PULSE + SERVO_MAX_PULSE) / 2)

static TIM_HandleTypeDef htim4;

void servoRudder_init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin       = GPIO_PIN_9;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB, &gpio);

    htim4.Instance               = TIM4;
    htim4.Init.Prescaler         = 63;
    htim4.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim4.Init.Period            = 19999;
    htim4.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim4) != HAL_OK) { Error_Handler(); }

    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode        = TIM_OCMODE_PWM1;
    oc.Pulse         = SERVO_CENTER_PULSE;
    oc.OCPolarity    = TIM_OCPOLARITY_HIGH;
    oc.OCNPolarity   = TIM_OCNPOLARITY_HIGH;
    oc.OCFastMode    = TIM_OCFAST_DISABLE;
    oc.OCIdleState   = TIM_OCIDLESTATE_RESET;
    oc.OCNIdleState  = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim4, &oc, TIM_CHANNEL_4) != HAL_OK) { Error_Handler(); }

    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

    /* Sweep to extremes on startup so you can see if servo responds */
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, SERVO_CENTER_PULSE-100);
    HAL_Delay(1000);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, SERVO_CENTER_PULSE+100);
    HAL_Delay(1000);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, SERVO_CENTER_PULSE);

    printf("[SERVO] Rudder init OK, centered at %d us\r\n", SERVO_CENTER_PULSE);
}

void servoRudder_setAngle(int16_t angle)
{
    if (angle < SERVO_MIN_ANGLE) angle = SERVO_MIN_ANGLE;
    if (angle > SERVO_MAX_ANGLE) angle = SERVO_MAX_ANGLE;

    uint32_t pulse = SERVO_MIN_PULSE +
        ((uint32_t)(angle - SERVO_MIN_ANGLE) * (SERVO_MAX_PULSE - SERVO_MIN_PULSE))
        / (SERVO_MAX_ANGLE - SERVO_MIN_ANGLE);

    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, pulse);
    printf("[SERVO] Rudder -> %d deg (%lu us)\r\n", angle, pulse);
}
