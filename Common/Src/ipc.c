#include "ipc.h"
#include "sharedCommand.h"
#include <string.h>
#include "stm32h7xx_hal.h"

// Allocate both memory structs into SRAM4 (.ipc_shared section in both .ld files handles actual allocation)
ipc_shared_t ipc_shared __attribute__((section(".ipc_shared")));

const char *ipc_status_str(ipc_status_t status) {
    switch (status) {
        case IPC_STATUS_OK:               return "OK";
        case IPC_STATUS_BUSY:             return "BUSY";
        case IPC_STATUS_EMPTY:            return "EMPTY";
        case IPC_STATUS_FULL:             return "FULL";
        case IPC_STATUS_TIMEOUT:          return "TIMEOUT";
        case IPC_STATUS_VERSION_MISMATCH: return "VERSION_MISMATCH";
    }
    return "UNKNOWN";
}

bool ipc_lock(uint32_t hsem_id) {
    if (HAL_HSEM_FastTake(hsem_id) != HAL_OK) {
        return false;
    }
    __DMB(); // Prevents memory operations before HSEM is taken
    return true;
}

void ipc_unlock(uint32_t hsem_id) {
    __DMB(); // Finish all memory operation before releasing
    HAL_HSEM_Release(hsem_id, IPC_PROCESS_ID);
}

#ifdef CORE_CM4 // Block these functions if CM4 is not configured
ipc_status_t ipc_cmd_push(const ipc_cmd_t *cmd) {
    uint32_t head = ipc_shared.cmd.head;
    uint32_t tail = ipc_shared.cmd.tail;

    if ((head - tail) >= IPC_CMD_RING_SLOTS) {
        return IPC_STATUS_FULL;
    }
    __DMB(); // Read tail before overwriting the slot the CM7 just freed

    // Write command to SRAM slot
    ipc_shared.cmd.slot[head & IPC_CMD_RING_MASK] = *cmd;

    __DMB(); // Complete the slot write before publishing it with head
    ipc_shared.cmd.head = head + 1U;

    // Trigger CM7s task interrupt (ring the doorbell)
    ipc_lock(IPC_HSEM_CMD_BELL);
    ipc_unlock(IPC_HSEM_CMD_BELL);
    return IPC_STATUS_OK;
}

ipc_status_t ipc_telem_read(LoRaTelemetry_t *telem, uint32_t *telem_seq) {
    if (ipc_lock(IPC_HSEM_TELEM)) {
        *telem_seq = ipc_shared.telem_seq;
        memcpy(telem, &ipc_shared.telem, sizeof(LoRaTelemetry_t));
        ipc_unlock(IPC_HSEM_TELEM);
        return IPC_STATUS_OK;
    }
    return IPC_STATUS_BUSY;
}
#endif

#ifdef CORE_CM7

static const MPU_Region_InitTypeDef SRAM4_Config = {
    MPU_REGION_ENABLE, 
    MPU_REGION_NUMBER0, 
    IPC_SHARED_BASE, 
    MPU_REGION_SIZE_64KB, 
    .SubRegionDisable = 0x00, // Min_Data 
    MPU_TEX_LEVEL1,
    MPU_REGION_FULL_ACCESS,
    MPU_INSTRUCTION_ACCESS_DISABLE,
    MPU_ACCESS_SHAREABLE,
    MPU_ACCESS_NOT_CACHEABLE,
    MPU_ACCESS_NOT_BUFFERABLE
};

void ipc_init(void) {
    // Enable Hardware Semaphore Clock
    __HAL_RCC_HSEM_CLK_ENABLE();

    // Diable MPU to configure SRAM4
    HAL_MPU_Disable();
    // Cofigure CM7 so it doesn't cache SRAM4 (CM4 doesn't cache)
    HAL_MPU_ConfigRegion(&SRAM4_Config);
    // Reenable the rest of the MPU with defaults
    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

    // Clear SRAM4
    memset((void *)&ipc_shared, 0, sizeof(ipc_shared));
    
    // Write IPC Version to SRAM4
    ipc_shared.version = IPC_VERSION;  
    // Report the Commander boot state until the first telemetry publish
    ipc_shared.telem.mode = SYSTEM_STATE_AUTO;

    // Make sure everything is cleared/written before magic is set
    __DMB();

    // Write Magic (this signals to CM4 that SRAM4 is initialized)
    ipc_shared.magic = IPC_MAGIC;
}

ipc_status_t ipc_cmd_pop(ipc_cmd_t *cmd) {
    uint32_t head = ipc_shared.cmd.head;
    uint32_t tail = ipc_shared.cmd.tail;

    if (head == tail) {
        return IPC_STATUS_EMPTY;
    }
    __DMB(); // Read head before reading the slot it published

    // Read from command slot in SRAM
    *cmd = ipc_shared.cmd.slot[tail & IPC_CMD_RING_MASK];

    __DMB(); // Finish reading the slot before handing it back to the CM4
    ipc_shared.cmd.tail = tail + 1U;
    return IPC_STATUS_OK;
}

ipc_status_t ipc_telem_write(LoRaTelemetry_t *telem) {
    if (ipc_lock(IPC_HSEM_TELEM)) {
        ipc_shared.telem_seq++;
        memcpy(&ipc_shared.telem, telem, sizeof(LoRaTelemetry_t));
        ipc_unlock(IPC_HSEM_TELEM);
        return IPC_STATUS_OK;
    }
    return IPC_STATUS_BUSY;
}
#endif
