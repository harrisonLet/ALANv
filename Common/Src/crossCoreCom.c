#include "crossCoreCom.h"

#include <string.h>

#include "stm32h7xx_hal.h"

// Allocate both memory structs into SRAM4 (linker handles actual allocation)
ipc_shared_t ipc_shared __attribute__((section(".ipc_shared")));

// Attempt to take hardware semaphore
bool ipc_lock(uint32_t hsem_id) {
    if (HAL_HSEM_FastTake(hsem_id) != HAL_OK) {
        return false;
    }
    __DMB(); // 
    return true;
}

void ipc_unlock(uint32_t hsem_id) {
    __DMB();
    HAL_HSEM_Release(hsem_id, IPC_PROCESS_ID);
}

#ifdef CORE_CM4 // Block these functions () if CM4 is not configured
// pos is a free-running byte count, wraps to the start of data[] when needed
static void ipc_buf_write(uint32_t pos, const void *src, uint32_t num_bytes) {
    uint32_t buf_index = pos & (IPC_CMD_RING_SIZE - 1U);
    uint32_t first = IPC_CMD_RING_SIZE - buf_index;

    if (first > num_bytes) {
        first = num_bytes;
    }
    memcpy(&ipc_shared.cmd.data[buf_index], src, first);
    memcpy(&ipc_shared.cmd.data[0], (const uint8_t *)src + first, num_bytes - first);
}

bool ipc_cmd_push(const ipc_cmd_t *cmd) {
    uint32_t head = ipc_shared.cmd.head;
    uint32_t tail = ipc_shared.cmd.tail;
    uint32_t space = IPC_CMD_RING_SIZE - (head - tail);
    uint32_t record_size = IPC_CMD_RECORD_SIZE(cmd->hdr.len);

    if (cmd->hdr.len > IPC_CMD_MAX_PAYLOAD || record_size > space) {
        return false;
    }

    ipc_buf_write(head, cmd, sizeof(ipc_cmd_hdr_t) + cmd->hdr.len);
    __DMB();
    ipc_shared.cmd.head = head + record_size;
    return true;
}

bool ipc_telem_read(ipc_telemetry_t *telem, uint32_t *telem_seq) {
    if (ipc_lock(IPC_HSEM_TELEM)) {
        *telem_seq = ipc_shared.telem_seq;
        memcpy(telem, &ipc_shared.telem, sizeof(ipc_telemetry_t));
        ipc_unlock(IPC_HSEM_TELEM);
        return true;
    }
    return false;
}
#endif

#ifdef CORE_CM7
static void ipc_buf_read(uint32_t pos, void *dst, uint32_t num_bytes) {
    uint32_t buf_index = pos & (IPC_CMD_RING_SIZE - 1U);
    uint32_t first = IPC_CMD_RING_SIZE - buf_index;

    if (first > num_bytes) {
        first = num_bytes;
    }
    memcpy(dst, &ipc_shared.cmd.data[buf_index], first);
    memcpy((uint8_t *)dst + first, &ipc_shared.cmd.data[0], num_bytes - first);
}

bool ipc_cmd_pop(ipc_cmd_t *cmd) {
    uint32_t head = ipc_shared.cmd.head;
    uint32_t tail = ipc_shared.cmd.tail;

    if (head == tail) {
        return false;
    }
    __DMB();

    ipc_buf_read(tail, &cmd->hdr, sizeof(ipc_cmd_hdr_t));
    if (cmd->hdr.len > IPC_CMD_MAX_PAYLOAD) {
        // Corrupt record, drop everything currently in the ring
        ipc_shared.cmd.tail = head;
        return false;
    }
    ipc_buf_read(tail + sizeof(ipc_cmd_hdr_t), cmd->payload.raw, cmd->hdr.len);

    __DMB();
    ipc_shared.cmd.tail = tail + IPC_CMD_RECORD_SIZE(cmd->hdr.len);
    return true;
}

bool ipc_telem_write(ipc_telemetry_t *telem) {
    if (ipc_lock(IPC_HSEM_TELEM)) {
        ipc_shared.telem_seq++;
        memcpy(&ipc_shared.telem, telem, sizeof(ipc_telemetry_t));
        ipc_unlock(IPC_HSEM_TELEM);
        return true;
    }
    return false;
}
#endif
