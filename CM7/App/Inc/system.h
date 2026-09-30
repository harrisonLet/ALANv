#include "stm32h7xx.h"
#include "stm32h755xx.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_def.h"
#include "stm32h7xx_hal_gpio.h"
#include "stm32h7xx_hal_i2c.h"
#include "stm32h7xx_hal_uart.h"
#include "stm32h7xx_nucleo.h"

#include "task.h"

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>



#define STM32H755_CLOCK_RATE 64000000
