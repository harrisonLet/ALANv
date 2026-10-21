#include "ipcCM7.h"

#include "shared_configuration.h"
#include "system.h"

// Check and define Hardware Semaphore for Data Signaling
#define HSEM_DOORBELL_MASK (__HAL_HSEM_SEMID_TO_MASK(IPC_HSEM_CMD_BELL))
#define DOORBELL_IRQ_PRIORITY (6U)

// How many times the CM7 should try and send telemetry before issuing a warning
#define TELEM_BUSY_WARN_COUNT (5U)

static TaskHandle_t command_task = NULL;
static uint32_t telem_busy_count = 0;

void ipc_cm7_init(void) {
    command_task = xTaskGetCurrentTaskHandle();
    
    // Set up Interrut Callbacks for HSEM 2 (CM4's command doorbell)
    __HAL_HSEM_CLEAR_FLAG(HSEM_DOORBELL_MASK);
    HAL_HSEM_ActivateNotification(HSEM_DOORBELL_MASK);
    HAL_NVIC_SetPriority(HSEM1_IRQn, DOORBELL_IRQ_PRIORITY, 0);
    HAL_NVIC_EnableIRQ(HSEM1_IRQn);
}

/**
  * @brief  CM7's HSEM interrupt line. HAL clears and disables the notifications that fired,
  *         then calls HAL_HSEM_FreeCallback().
  * @retval None
  */
void HSEM1_IRQHandler(void) {
    HAL_HSEM_IRQHandler();
}

/**
  * @brief  Re-arm the doorbell and wake the comms task. Runs in interrupt context.
  * @param  SemMask  Mask of the semaphores that were released.
  * @retval None
  */
void HAL_HSEM_FreeCallback(uint32_t SemMask) {
    if (SemMask & HSEM_DOORBELL_MASK) {
        HAL_HSEM_ActivateNotification(HSEM_DOORBELL_MASK);

        // Try and Unblock Command Handler Task
        BaseType_t cmdTaskUnblocked = pdFALSE;
        vTaskNotifyGiveFromISR(command_task, &cmdTaskUnblocked);

        // Request context switch from RTOS if task is unblocked
        portYIELD_FROM_ISR(cmdTaskUnblocked); 
    }
}

ipc_status_t ipc_cm7_pop_command(ipc_cmd_t *cmd) {
    ipc_status_t status = ipc_cmd_pop(cmd);

    // EMPTY is the normal end of a drain loop, only CORRUPT is an error
    if (status == IPC_STATUS_CORRUPT) {
        printf("%s ERROR: %s command record (len %u > max %u), command ring flushed\r\n",
        IPC_TASK, ipc_status_str(status), (unsigned)cmd->hdr.len, (unsigned)IPC_CMD_MAX_PAYLOAD);
    }
    return status;
}

ipc_status_t ipc_cm7_publish_telemetry(ipc_telemetry_t *telem) {
    ipc_status_t status = ipc_telem_write(telem);

    if (status == IPC_STATUS_OK) {
        if (telem_busy_count >= TELEM_BUSY_WARN_COUNT) {
            printf("%s Telemetry publish recovered after %lu skipped rounds\r\n",
                   IPC_TASK, (unsigned long)telem_busy_count);
        }
        telem_busy_count = 0;
    }
    else if (++telem_busy_count == TELEM_BUSY_WARN_COUNT) {
        printf("%s ERROR: Telemetry publish %s %lu times in a row, CM4 may be stuck holding HSEM %u\r\n",
               IPC_TASK, ipc_status_str(status), (unsigned long)telem_busy_count, (unsigned)IPC_HSEM_TELEM);
    }
    return status;
}
