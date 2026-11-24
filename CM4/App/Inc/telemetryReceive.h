#ifndef TELEMETRY_RECEIVE_H
#define TELEMETRY_RECEIVE_H

#include "shared_configuration.h"
#include "ipcCM4.h"
#include "lora.h"
#include "loraTelemetry.h"
#include <stdint.h>

#define FLAG_STALE          (1u << 0) // bit 0
#define FLAG_BUSY_FALLBACK  (1u << 1) // bit 1
#define FLAG_NO_IPC         (1u << 2) // bit 2
#define FLAG_MASK           (FLAG_STALE | FLAG_BUSY_FALLBACK | FLAG_NO_IPC)

void telemetryRec_init(bool ipc_attached);
bool telemetryRec_read(uint8_t *flags, LoRaTelemetry_t *telem_packet);

#endif
