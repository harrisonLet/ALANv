#include "ipcCM4.h"

#include "shared_configuration.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_nucleo.h"

bool ipc_cm4_attach(uint32_t timeout_ms) {
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
bool ipc_cm4_read_telemetry(ipc_telemetry_t *telem, uint32_t *seq) {
    return ipc_telem_read(telem, seq);
}
bool ipc_cm4_push_command(ipc_cmd_t *cmd) {
    return ipc_cmd_push(cmd);
}