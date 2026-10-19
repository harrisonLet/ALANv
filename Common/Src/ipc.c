#include "ipc.h"
#include <string.h>
#include "stm32h7xx_hal.h"

/* NOTE: The ring buffer used to store commands will rarely exceed 1 command due to the LORA send rate 
being much slower than the CM7 task that will drain the entire ring buffer on every interrupt. 
The buffer is implemented incase the CM7 gets caught or slightly lags. */

// Allocate both memory structs into SRAM4 (.ld IPC_SHARED section handles actual allocation)
ipc_shared_t ipc_shared __attribute__((section(".ipc_shared")));

const char *ipc_status_str(ipc_status_t status) {
    switch (status) {
        case IPC_STATUS_OK:               return "OK";
        case IPC_STATUS_BUSY:             return "BUSY";
        case IPC_STATUS_EMPTY:            return "EMPTY";
        case IPC_STATUS_FULL:             return "FULL";
        case IPC_STATUS_BAD_LEN:          return "BAD_LEN";
        case IPC_STATUS_CORRUPT:          return "CORRUPT";
        case IPC_STATUS_TIMEOUT:          return "TIMEOUT";
        case IPC_STATUS_VERSION_MISMATCH: return "VERSION_MISMATCH";
    }
    return "UNKNOWN";
}

// Attempt to take hardware semaphore
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
// pos is a free-running byte count, wraps to the start of data[] when needed
static void ipc_buf_write(uint32_t pos, const void *src, uint32_t num_bytes) {
    // MOD 2048 to get the data's 'index' in the buffer
    uint32_t buf_index = pos & (IPC_CMD_RING_SIZE - 1U);

    // Check if amount of data to be written will overflow (needs to split across the end and start of buffer)
    uint32_t remaining_space = IPC_CMD_RING_SIZE - buf_index;
    if (remaining_space > num_bytes) {
        remaining_space = num_bytes;
    }

    // First write using the amount of space left (won't overflow most of the time)
    memcpy(&ipc_shared.cmd.data[buf_index], src, remaining_space);

    // If overflowed the amount of bytes to copy to beginning of array will be > 0 here
    memcpy(&ipc_shared.cmd.data[0], (const uint8_t *)src + remaining_space, num_bytes - remaining_space);
}

ipc_status_t ipc_cmd_push(const ipc_cmd_t *cmd) {
    uint32_t head = ipc_shared.cmd.head;
    uint32_t tail = ipc_shared.cmd.tail;
    uint32_t space = IPC_CMD_RING_SIZE - (head - tail);
    uint32_t record_size = IPC_CMD_RECORD_SIZE(cmd->hdr.len);

    if (cmd->hdr.len > IPC_CMD_MAX_PAYLOAD) {
        return IPC_STATUS_BAD_LEN;
    }
    if (record_size > space) {
        return IPC_STATUS_FULL;
    }

    ipc_buf_write(head, cmd, sizeof(ipc_cmd_hdr_t) + cmd->hdr.len);
    __DMB(); // Complete all write operations before updating head and returning
    ipc_shared.cmd.head = head + record_size;
    return IPC_STATUS_OK;
}

ipc_status_t ipc_telem_read(ipc_telemetry_t *telem, uint32_t *telem_seq) {
    if (ipc_lock(IPC_HSEM_TELEM)) {
        *telem_seq = ipc_shared.telem_seq;
        memcpy(telem, &ipc_shared.telem, sizeof(ipc_telemetry_t));
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
    // Set default sail-boat mode in SRAM4 to AUTO
    ipc_shared.telem.mode = IPC_MODE_AUTO;

    // Make sure everything is cleared/written before magic is set
    __DMB();

    // Write Magic (this signals to CM4 that SRAM4 is initialized)
    ipc_shared.magic = IPC_MAGIC;
}

static void ipc_buf_read(uint32_t pos, void *dst, uint32_t num_bytes) {
    // Mod 2048 (struct is 2048 Bytes) to get its index
    uint32_t buf_index = pos & (IPC_CMD_RING_SIZE - 1U);

    // Check how much space is left in the ring buffer
    uint32_t remaining_space = IPC_CMD_RING_SIZE - buf_index;
    if (remaining_space > num_bytes) {
        remaining_space = num_bytes;
    }

    // Copy data from SRAM4 into dst using the remaining_space
    memcpy(dst, &ipc_shared.cmd.data[buf_index], remaining_space);

    // Copy remaining data from the beginning of the buffer (it wrapped around due to no space)
    memcpy((uint8_t *)dst + remaining_space, &ipc_shared.cmd.data[0], num_bytes - remaining_space);
}

ipc_status_t ipc_cmd_pop(ipc_cmd_t *cmd) {
    uint32_t head = ipc_shared.cmd.head;
    uint32_t tail = ipc_shared.cmd.tail;

    if (head == tail) {
        return IPC_STATUS_EMPTY;
    }
    __DMB(); // Make sure reading is done with updated head/tail

    ipc_buf_read(tail, &cmd->hdr, sizeof(ipc_cmd_hdr_t));
    if (cmd->hdr.len > IPC_CMD_MAX_PAYLOAD) {
        // Corrupt record, drop everything currently in the ring
        ipc_shared.cmd.tail = head;
        return IPC_STATUS_CORRUPT;
    }
    ipc_buf_read(tail + sizeof(ipc_cmd_hdr_t), cmd->payload.raw, cmd->hdr.len);

    __DMB(); // Finish all buffer operations before updating tail and returning
    ipc_shared.cmd.tail = tail + IPC_CMD_RECORD_SIZE(cmd->hdr.len);
    return IPC_STATUS_OK;
}

ipc_status_t ipc_telem_write(ipc_telemetry_t *telem) {
    if (ipc_lock(IPC_HSEM_TELEM)) {
        ipc_shared.telem_seq++;
        memcpy(&ipc_shared.telem, telem, sizeof(ipc_telemetry_t));
        ipc_unlock(IPC_HSEM_TELEM);
        return IPC_STATUS_OK;
    }
    return IPC_STATUS_BUSY;
}
#endif
