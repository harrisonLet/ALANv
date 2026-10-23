#pragma once

#include "main.h"

#include "stm32h7xx.h"
#include "stm32h755xx.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_def.h"
#include "stm32h7xx_hal_gpio.h"
#include "stm32h7xx_hal_i2c.h"
#include "stm32h7xx_hal_uart.h"
#include "stm32h7xx_nucleo.h"

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "shared_configuration.h"

#define STM32H755_CLOCK_RATE 64000000
#define RAD2DEG 57.29577951308232f
#define DEG2RAD 0.017453292519943295f

#define DEBUG_PRINTF(tag, fmt, ...) printf("%s " fmt "\r\n", tag, ##__VA_ARGS__)

#define ANG_WRAP_180(angle) ((int16_t)(((((angle) + 180) % 360 + 360) % 360) - 180))
#define ANG_WRAP_360(angle) ((uint16_t)((((angle) % 360) + 360) % 360))

typedef enum {
    HARDWARE_INIT_SUCCESS,
    HARDWARE_INIT_FAIL
} HardwareInit_e;

typedef enum {
    RTOS_INIT_SUCCESS,
    RTOS_INIT_FAIL
} RtosInit_e;