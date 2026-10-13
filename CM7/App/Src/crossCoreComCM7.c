#include <crossCoreComCM7.h>

#include "shared_configuration.h"
#include "system.h"

// Check and define Hardware Semaphore for Data Signaling
#define HSEM_DOORBELL_MASK (__HAL_HSEM_SEMID_TO_MASK(IPC_HSEM_CMD_BELL))
#define DOORBELL_IRQ_PRIORITY (6U)

static TaskHandle_t command_task = NULL;

// MUST BE CALLED BY TASK THAT HANDLES CM7 COMS NOT HWINIT()
void crossCoreComCM7_init(void) {
    command_task = xTaskGetCurrentTaskHandle();
    
    // Set up Interrut Callbacks for HSEM 2 (CM4's command doorbell)
    __HAL_HSEM_CLEAR_FLAG(HSEM_DOORBELL_MASK);
    HAL_HSEM_ActivateNotification(HSEM_DOORBELL_MASK);
    HAL_NVIC_SetPriority(HSEM1_IRQn, DOORBELL_IRQ_PRIORITY, 0);
    HAL_NVIC_EnableIRQ(HSEM1_IRQn);
}

void HSEM1_IRQHandler(void) {
    HAL_HSEM_IRQHandler();
}

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

bool crossCoreComCM7_popCommand(ipc_cmd_t *cmd) {
    return ipc_cmd_pop(cmd);
}

bool crossCoreComCM7_publishTelemetry(ipc_telemetry_t *telem) {
    return ipc_telem_write(telem);
}



