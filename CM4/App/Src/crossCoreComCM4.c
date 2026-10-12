#include <crossCoreComCM4.h>

#include "shared_configuration.h"
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