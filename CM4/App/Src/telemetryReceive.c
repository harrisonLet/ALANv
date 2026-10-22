#include "telemetryReceive.h"

#define FLAG_SET(f, m)   ((f) |= (m))
#define FLAG_RESET(f, m)   ((f) &= (uint8_t)~(m))
#define FLAG_CHK(f, m)   (((f) & (m)) != 0)

#define MAX_BUSY_READ_ATTEMPTS (5U)

static uint8_t telem_status_flags = 0;
static uint32_t prev_telem_seq = 0;
static uint8_t busy_retries = 0;

void telemetryRec_init(bool ipc_attached) {
    // COnditionally set IPC flag based on attach cm4/cm7 handshake result
    if (ipc_attached) {
        FLAG_RESET(telem_status_flags, FLAG_NO_IPC);
    } else {
        FLAG_SET(telem_status_flags, FLAG_NO_IPC);
    }
    // Reset other 2 flags unconditionally
    FLAG_RESET(telem_status_flags, FLAG_STALE);
    FLAG_RESET(telem_status_flags, FLAG_BUSY_FALLBACK);

    prev_telem_seq = 0;
    busy_retries = 0;
}

bool telemetryRec_read(uint8_t *flags, LoRaTelemetry_t *telem_packet) 
{
    // Check if IPC is correctly setup and working
    if (FLAG_CHK(telem_status_flags, FLAG_NO_IPC)) {
        *flags = telem_status_flags;
        return true;
    }

    uint32_t telem_seq;
    ipc_status_t result = ipc_cm4_read_telemetry(telem_packet, &telem_seq);

    if (result == IPC_STATUS_OK) {
        // reset busy_flag
        FLAG_RESET(telem_status_flags, FLAG_BUSY_FALLBACK);
        busy_retries = 0;
        
        // Succesfully read but check if telem was updated by cm7
        if (telem_seq != prev_telem_seq) {
            prev_telem_seq = telem_seq;
            FLAG_RESET(telem_status_flags, FLAG_STALE);
        } else {
            FLAG_SET(telem_status_flags ,FLAG_STALE);
        }
        *flags = telem_status_flags; 
        return true;
    }

    // Has to be IPC_BUSY so check if read max amount of times
    else if (++busy_retries >= MAX_BUSY_READ_ATTEMPTS) {
        // Max attempts, set flag and return true so LoRa can send
        FLAG_SET(telem_status_flags, FLAG_BUSY_FALLBACK);
        busy_retries = 0;
        *flags = telem_status_flags;
        return true;
    }

    // Busy but not max attempts: set false so main tries again next systick
    *flags = telem_status_flags;
    return false;
}