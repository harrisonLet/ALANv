#include "shared_configuration.h"
#include "ipcCM4.h"
#include "lora.h"
#include <stdint.h>

#define MAX_READ_ATTEMPTS 5
#define

#define FLAG_STALE          (1u << 0) // bit 0
#define FLAG_BUSY_FALLBACK  (1u << 1) // bit 1
#define FLAG_NO_IPC         (1u << 2) // bit 2
#define FLAG_MASK           (FLAG_STALE | FLAG_BUSY_FALLBACK | FLAG_NO_IPC)

#define FLAG_SET(f, m)   ((f) |= (m))
#define FLAG_RESET(f, m)   ((f) &= (uint8_t)~(m))
#define FLAG_CHK(f, m)   (((f) & (m)) != 0)

static volatile uint8_t telem_status_t = 0;

void telemetryRec_init(bool ipc_attached);                                                               │
bool telemetryRec_read(void);