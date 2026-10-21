/*
 * ipcCM7.h - CM7 side of the IPC (shared layout and status codes in ipc.h)
 *
 * Rules:
 *   - Call every function here from ONE task only (the comms task).
 *     HAL_HSEM_FastTake succeeds if this core already holds the HSEM, so it
 *     cannot keep two CM7 tasks apart.
 *   - ipc_init() (ipc.h) runs in hardware_init(). ipc_cm7_init() runs later,
 *     inside the comms task.
 *   - This module defines HSEM1_IRQHandler and HAL_HSEM_FreeCallback. Do not
 *     define HAL_HSEM_FreeCallback anywhere else in the CM7 image; extend this one.
 *   - Errors are printed with the IPC_TASK tag.
 */

#ifndef IPC_CM7_H
#define IPC_CM7_H

#include <stdbool.h>
#include "ipc.h"

#define IPC_TASK "[IPC]"

/**
  * @brief  Arm the HSEM 2 doorbell interrupt and record the calling task as the one to wake.
  * @retval None
  * @note   Call once from the comms task before it first waits.
  *         Not from hardware_init(): no task exists yet.
  */
void ipc_cm7_init(void);

/**
  * @brief  Publish telemetry for the CM4 to send.
  * @param  telem  Telemetry to publish.
  * @retval IPC_STATUS_OK    Published.
  * @retval IPC_STATUS_BUSY  Round skipped, the next publish catches up.
  * @note   Prints an error after 5 BUSY in a row, and again when it recovers.
  */
ipc_status_t ipc_cm7_publish_telemetry(LoRaTelemetry_t *telem);

/**
  * @brief  Pop the oldest command from the CM4.
  * @param  cmd  Destination for the command.
  * @retval IPC_STATUS_OK       Command copied into cmd.
  * @retval IPC_STATUS_EMPTY    Ring drained (normal, not an error).
  * @retval IPC_STATUS_CORRUPT  Error printed, all queued commands dropped.
  * @note   Drain the whole ring on each wake-up:
  *         while ((st = ipc_cm7_pop_command(&cmd)) != IPC_STATUS_EMPTY) {
  *             if (st == IPC_STATUS_OK) { dispatch(&cmd); }
  *         }
  */
ipc_status_t ipc_cm7_pop_command(ipc_cmd_t *cmd);

#endif
