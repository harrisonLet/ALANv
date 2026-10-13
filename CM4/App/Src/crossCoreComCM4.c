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

bool crossCoreComCM4_attach(uint32_t timeout_ms) {
    return true;
}
bool crossCoreComCM4_readTelemetry(ipc_telemetry_t *telem, uint32_t *seq) {
    return true;
}
bool crossCoreComCM4_pushCommand(ipc_cmd_t *cmd) {
    return true;
}