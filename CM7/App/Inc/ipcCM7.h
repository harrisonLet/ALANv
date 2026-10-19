#ifndef IPC_CM7_H
#define IPC_CM7_H

#include <stdbool.h>
#include "ipc.h"

void ipc_cm7_init(void);
bool ipc_cm7_publish_telemetry(ipc_telemetry_t *telem);
bool ipc_cm7_pop_command(ipc_cmd_t *cmd);

#endif
