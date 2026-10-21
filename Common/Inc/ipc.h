/*
 * ipc.h - Inter-processor communication between the CM7 and CM4
 *
 * Shared layout and low-level functions. Application code should use the
 * per-core APIs instead: ipcCM7.h on the CM7, ipcCM4.h on the CM4.
 *
 * Both channels live in one block, ipc_shared, in SRAM4 (0x38000000):
 *   - Telemetry (CM7 -> CM4): one latest-value copy guarded by HSEM 1. Only the
 *     newest value matters, so a BUSY read or write is skipped, not retried.
 *   - Commands (CM4 -> CM7): a lock-free byte ring (CM4 only writes, CM7 only
 *     reads) plus HSEM 2 as a doorbell interrupt that wakes the CM7. The ring
 *     rarely holds more than one command, since LoRa delivers commands far slower
 *     than the CM7 drains them. It only buffers if the CM7 lags.
 *
 * Rules:
 *   - HSEM 0 is the CubeMX boot handshake. Do not reuse it.
 *   - After changing ipc_shared_t, bump IPC_REV and reflash BOTH cores.
 *     IPC_VERSION catches size changes automatically, but not same-size ones.
 *   - Shared structs use fixed-width integer fields, never enum types
 *     (enum size depends on compiler flags).
 *   - ipc_shared must be at 0x38000000 in both images. The .ipc_shared section
 *     and its ASSERT in both .ld files enforce this.
 */

#ifndef IPC_H
#define IPC_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdio.h>

#define IPC_HSEM_TELEM          (1U)
#define IPC_HSEM_CMD_BELL       (2U)
#define IPC_PROCESS_ID          (0U)

#define IPC_SHARED_BASE         (0x38000000UL)
#define IPC_SHARED_SIZE         (64UL * 1024UL)
#define IPC_MAGIC               (0x414C414EUL) // ALAN in ASCII
#define IPC_REV                 (1U)

#define IPC_CMD_RING_SIZE       (2048U)
#define IPC_CMD_ALIGN           (4U)
#define IPC_CMD_MAX_PAYLOAD     (240U)

#define IPC_ALIGN_UP(n, a)          (((n) + ((a) - 1U)) & ~((a) - 1U))
#define IPC_CMD_RECORD_SIZE(len)    IPC_ALIGN_UP(sizeof(ipc_cmd_hdr_t) + (len), IPC_CMD_ALIGN)

enum { IPC_MODE_MANUAL, IPC_MODE_AUTO, IPC_MODE_SAFE };
enum { IPC_CMD_MANUAL_STEP = 1, IPC_CMD_SHUTDOWN, IPC_CMD_WAYPOINT, IPC_CMD_ROUTE_COMMIT };
enum { IPC_RESULT_NONE, IPC_RESULT_OK, IPC_RESULT_UNSUPPORTED, IPC_RESULT_BAD_LEN };

// Return codes for the IPC API. Separate from IPC_RESULT_*, which is telemetry data sent to the ground station
typedef enum {
    IPC_STATUS_OK = 0,
    IPC_STATUS_BUSY,                // Other core holds the HSEM, try next round
    IPC_STATUS_EMPTY,               // Nothing to pop (normal, not a failure)
    IPC_STATUS_FULL,                // Not enough ring space
    IPC_STATUS_BAD_LEN,             // Payload longer than IPC_CMD_MAX_PAYLOAD
    IPC_STATUS_CORRUPT,             // Bad record header, ring was flushed
    IPC_STATUS_TIMEOUT,             // Attach: magic never appeared
    IPC_STATUS_VERSION_MISMATCH,    // Attach: cores built with different layouts
} ipc_status_t;

typedef struct {
    uint32_t uptime_ms;
    int32_t  lat_e7;
    int32_t  lon_e7;
    int16_t  rudder_cmd_deg;
    int16_t  sail_cmd_deg;
    int16_t  sail_angle_deg;
    uint16_t heading_cdeg;
    uint16_t wind_speed_dkn;
    uint16_t wind_dir_deg;
    uint16_t last_cmd_seq;
    uint8_t  last_cmd_result;
    uint8_t  mode;
    uint8_t  gps_fix;
    uint8_t  battery_pct;
    uint8_t  reserved[2];
} ipc_telemetry_t;

typedef struct {
    uint16_t len;       // Payload bytes following the header
    uint16_t type;
    uint16_t seq;
    uint16_t reserved;
} ipc_cmd_hdr_t;

typedef struct {
    int16_t rudder_delta;
    int16_t sail_delta;
} ipc_cmd_manual_step_t;

typedef struct {
    uint16_t index;
    uint16_t margin_m;
    int32_t  lat_e7;
    int32_t  lon_e7;
} ipc_cmd_waypoint_t;

typedef struct {
    uint16_t count;
    uint16_t reserved;
} ipc_cmd_route_commit_t;

// Only hdr.len bytes of payload are stored in the ring
typedef struct {
    ipc_cmd_hdr_t hdr;
    union {
        ipc_cmd_manual_step_t  manual_step;
        ipc_cmd_waypoint_t     waypoint;
        ipc_cmd_route_commit_t route_commit;
        uint8_t                raw[IPC_CMD_MAX_PAYLOAD];
    } payload;
} ipc_cmd_t;

// Free-running byte offsets: used = head - tail, index = offset & (IPC_CMD_RING_SIZE - 1)
typedef struct {
    volatile uint32_t head;     // Written by CM4 only
    volatile uint32_t tail;     // Written by CM7 only
    uint8_t data[IPC_CMD_RING_SIZE] __attribute__((aligned(IPC_CMD_ALIGN)));
} ipc_cmd_ring_t;

typedef struct {
    volatile uint32_t magic;
    volatile uint32_t version;
    volatile uint32_t telem_seq;
    ipc_telemetry_t   telem;
    ipc_cmd_ring_t    cmd;
} ipc_shared_t;

#define IPC_VERSION (((uint32_t)IPC_REV << 16) | ((uint32_t)sizeof(ipc_shared_t) & 0xFFFFU))

extern ipc_shared_t ipc_shared;

_Static_assert((IPC_CMD_RING_SIZE & (IPC_CMD_RING_SIZE - 1U)) == 0, "IPC_CMD_RING_SIZE must be a power of two");
_Static_assert((IPC_CMD_ALIGN & (IPC_CMD_ALIGN - 1U)) == 0, "IPC_CMD_ALIGN must be a power of two");
_Static_assert(offsetof(ipc_cmd_t, payload) == sizeof(ipc_cmd_hdr_t), "payload must directly follow header");
_Static_assert(IPC_CMD_RECORD_SIZE(IPC_CMD_MAX_PAYLOAD) <= IPC_CMD_RING_SIZE, "max command does not fit in ring");
_Static_assert(sizeof(ipc_shared_t) <= IPC_SHARED_SIZE, "ipc_shared_t does not fit in SRAM4");

/**
  * @brief  Get a printable name for a status code, e.g. "CORRUPT".
  * @param  status  Status code to convert.
  * @retval Constant string, "UNKNOWN" for values outside ipc_status_t.
  */
const char *ipc_status_str(ipc_status_t status);

/**
  * @brief  Try once to take a hardware semaphore. Never spins.
  * @param  hsem_id  HSEM to take (IPC_HSEM_*).
  * @retval true   Taken.
  * @retval false  Held by the other core.
  * @note   Only locks out the other core: a second task on the same core would also succeed.
  */
bool ipc_lock(uint32_t hsem_id);

/**
  * @brief  Release a hardware semaphore taken with ipc_lock().
  * @param  hsem_id  HSEM to release (IPC_HSEM_*).
  * @retval None
  */
void ipc_unlock(uint32_t hsem_id);

#ifdef CORE_CM4
/**
  * @brief  Copy a command into the command ring and ring the HSEM 2 doorbell to wake the CM7.
  * @param  cmd  Command to queue. Only the header and hdr.len payload bytes are copied.
  * @retval IPC_STATUS_OK       Queued.
  * @retval IPC_STATUS_FULL     Not enough room, nothing queued.
  * @retval IPC_STATUS_BAD_LEN  hdr.len > IPC_CMD_MAX_PAYLOAD, nothing queued.
  */
ipc_status_t ipc_cmd_push(const ipc_cmd_t *cmd);

/**
  * @brief  Copy the latest telemetry and its sequence number.
  * @param  telem      Destination for the telemetry.
  * @param  telem_seq  Destination for the sequence number.
  * @retval IPC_STATUS_OK    Copied.
  * @retval IPC_STATUS_BUSY  CM7 is mid-write, nothing copied.
  */
ipc_status_t ipc_telem_read(ipc_telemetry_t *telem, uint32_t *telem_seq);
#endif

#ifdef CORE_CM7
/**
  * @brief  Prepare SRAM4: make it non-cacheable, clear ipc_shared, then write version and magic.
  * @retval None
  * @note   Call once at boot, before releasing the CM4 from HSEM 0.
  * @note   Uses MPU region 0.
  */
void ipc_init(void);

/**
  * @brief  Copy the oldest command out of the ring and free its space.
  * @param  cmd  Destination for the command.
  * @retval IPC_STATUS_OK       Command copied into cmd.
  * @retval IPC_STATUS_EMPTY    Nothing queued.
  * @retval IPC_STATUS_CORRUPT  Bad record header, all queued commands dropped.
  */
ipc_status_t ipc_cmd_pop(ipc_cmd_t *cmd);

/**
  * @brief  Copy telemetry into shared memory and increment telem_seq.
  * @param  telem  Telemetry to publish.
  * @retval IPC_STATUS_OK    Published.
  * @retval IPC_STATUS_BUSY  CM4 is mid-read, nothing written.
  */
ipc_status_t ipc_telem_write(ipc_telemetry_t *telem);
#endif

#endif
