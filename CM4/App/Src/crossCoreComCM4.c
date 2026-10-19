#include <crossCoreComCM4.h>

#include "shared_configuration.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_nucleo.h"

bool crossCoreComCM4_attach(uint32_t timeout_ms) {
    uint32_t start_time_ms = HAL_GetTick();

    while ((HAL_GetTick() - start_time_ms) < timeout_ms) {
        if (ipc_shared.magic == IPC_MAGIC) {
            __DMB();
            if (ipc_shared.version == IPC_VERSION) {
                // IPC Init Complete and Version Verified
                return true;
            } 
            else {
                // ERROR: IPC setup but version mismatch
                return false;
            }
        }
    }
    return false;
}
bool crossCoreComCM4_readTelemetry(ipc_telemetry_t *telem, uint32_t *seq) {
    return ipc_telem_read(telem, seq);
}
bool crossCoreComCM4_pushCommand(ipc_cmd_t *cmd) {
    return ipc_cmd_push(cmd);
}