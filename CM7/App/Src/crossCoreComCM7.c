#include <crossCoreComCM7.h>

#include "shared_configuration.h"

#include "portmacro.h"

#include "stm32h7xx_hal_conf.h"
#include "stm32h7xx_hal_gpio.h"
#include "stm32h7xx_hal_rcc.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_nucleo.h"

// Check and define Hardware Semaphore for Data Signaling
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
#ifndef HSEM_ID_1
#define HSEM_ID_1 (1U)
#endif
#ifndef HSEM_ID_2
#define HSEM_ID_2 (2U)
#endif
#endif

void 


