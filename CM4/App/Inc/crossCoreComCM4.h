#ifndef CROSS_CORE_COM_CM4_H
#define CROSS_CORE_COM_CM4_H

#include <stdbool.h>
#include "crossCoreCom.h"

bool crossCoreComCM4_attach(uint32_t timeout_ms);
bool crossCoreComCM4_readTelemetry(ipc_telemetry_t *telem, uint32_t *seq);
bool crossCoreComCM4_pushCommand(ipc_cmd_t *cmd);

#endif
