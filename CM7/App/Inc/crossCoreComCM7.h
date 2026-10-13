#ifndef CROSS_CORE_COM_CM7_H
#define CROSS_CORE_COM_CM7_H

#include <stdbool.h>
#include "crossCoreCom.h"

void crossCoreComCM7_init(void);
bool crossCoreComCM7_publishTelemetry(ipc_telemetry_t *telem);
bool crossCoreComCM7_popCommand(ipc_cmd_t *cmd);

#endif
