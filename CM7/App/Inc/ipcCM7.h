#ifndef IPC_CM7_H
#define IPC_CM7_H

#include <stdbool.h>
#include "ipc.h"

#define IPC_TASK "[IPC]"

void ipc_cm7_init(void);
ipc_status_t ipc_cm7_publish_telemetry(ipc_telemetry_t *telem);
ipc_status_t ipc_cm7_pop_command(ipc_cmd_t *cmd);

#endif
