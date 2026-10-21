/*
 * ipcCM4.h - CM4 side of the IPC (shared layout and status codes in ipc.h)
 *
 * Rules:
 *   - Call ipc_cm4_attach() first, after HAL_Init(). Only use the other
 *     functions if it returned OK.
 *   - Call push/read from the main loop only, never from an interrupt handler.
 *   - printf does not reach the UART while both cores run, so report
 *     failures with LEDs or the debugger.
 */

#ifndef IPC_CM4_H
#define IPC_CM4_H

#include <stdbool.h>
#include "ipc.h"

/**
  * @brief  Wait for the CM7's ipc_init() to finish and check both cores share the same layout.
  * @param  timeout_ms  How long to wait, in milliseconds (uses HAL_GetTick).
  * @retval IPC_STATUS_OK                Shared memory ready.
  * @retval IPC_STATUS_TIMEOUT           CM7 never initialized, or ipc_shared is not at 0x38000000 in both images.
  * @retval IPC_STATUS_VERSION_MISMATCH  Cores built from different ipc.h, reflash both.
  */
ipc_status_t ipc_cm4_attach(uint32_t timeout_ms);

/**
  * @brief  Copy the latest telemetry and its sequence number.
  * @param  telem  Destination for the telemetry.
  * @param  seq    Destination for the sequence number.
  * @retval IPC_STATUS_OK    Copied.
  * @retval IPC_STATUS_BUSY  CM7 is mid-write, try next tick.
  * @note   seq increases on every CM7 publish. If it stops changing, the CM7 has stalled.
  */
ipc_status_t ipc_cm4_read_telemetry(LoRaTelemetry_t *telem, uint32_t *seq);

/**
  * @brief  Queue a command for the CM7 and wake it with the HSEM 2 doorbell.
  * @param  cmd  Command to send. Fill in hdr.type, hdr.seq, hdr.len (payload bytes) and the payload first.
  * @retval IPC_STATUS_OK       Queued.
  * @retval IPC_STATUS_FULL     CM7 is not draining, command dropped.
  * @retval IPC_STATUS_BAD_LEN  hdr.len > IPC_CMD_MAX_PAYLOAD, command dropped.
  */
ipc_status_t ipc_cm4_push_command(ipc_cmd_t *cmd);

#endif
