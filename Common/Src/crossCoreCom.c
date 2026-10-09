#include "crossCoreCom.h"

#include <string.h>

#include "stm32h7xx_hal.h"

// Placed at IPC_SHARED_BASE (SRAM4) by the linker, same address in both images
ipc_shared_t ipc_shared __attribute__((section(".ipc_shared")));

// Interrupts stay disabled while the lock is held, ipc_unlock restores them
bool ipc_lock(uint32_t hsem_id, uint32_t *primask) {
    *primask = __get_PRIMASK();
    __disable_irq();

    if (HAL_HSEM_FastTake(hsem_id) != HAL_OK) {
        __set_PRIMASK(*primask);
        return false;
    }
    __DMB();
    return true;
}

void ipc_unlock(uint32_t hsem_id, uint32_t primask) {
    __DMB();
    HAL_HSEM_Release(hsem_id, IPC_PROCESS_ID);
    __set_PRIMASK(primask);
}

#ifdef CORE_CM4
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
#endif
