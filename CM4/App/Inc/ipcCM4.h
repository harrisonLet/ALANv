#ifndef IPC_CM4_H
#define IPC_CM4_H

#include <stdbool.h>
#include "ipc.h"

bool ipc_cm4_attach(uint32_t timeout_ms);
bool ipc_cm4_read_telemetry(ipc_telemetry_t *telem, uint32_t *seq);
bool ipc_cm4_push_command(ipc_cmd_t *cmd);

#endif
