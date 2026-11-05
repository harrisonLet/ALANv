/*
 * ipc.h - Inter-processor communication between the CM7 and CM4
 *
 * Shared layout and low-level functions. Application code should use the
 * per-core APIs instead: ipcCM7.h on the CM7, ipcCM4.h on the CM4.
 *
 * Both channels live in one block, ipc_shared, in SRAM4 (0x38000000):
 *   - Telemetry (CM7 -> CM4): one latest-value copy guarded by HSEM 1. Only the
 *     newest value matters, so a BUSY read or write is skipped, not retried.
 *   - Commands (CM4 -> CM7): a lock-free ring of fixed-size ipc_cmd_t slots
 *     (CM4 only writes, CM7 only reads) plus HSEM 2 as a doorbell interrupt that
 *     wakes the CM7. The ring rarely holds more than one command, since LoRa
 *     delivers commands far slower than the CM7 drains them. It only buffers if
 *     the CM7 lags.
 *   - ipc.h does not know command payload layouts. The CM4 copies payload bytes
 *     into ipc_cmd_t.raw unchanged, and the CM7 decodes them based on type.
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
#include "loraTelemetry.h"

#define IPC_HSEM_TELEM          (1U)
#define IPC_HSEM_CMD_BELL       (2U)
#define IPC_PROCESS_ID          (0U)

#define IPC_SHARED_BASE         (0x38000000UL)
#define IPC_SHARED_SIZE         (64UL * 1024UL)
#define IPC_MAGIC               (0x414C414EUL) // ALAN in ASCII
#define IPC_REV                 (3U)

#define IPC_CMD_DATA_SIZE       (16U)   // Largest payload: waypoint (12 B). Route commit is 8 B
#define IPC_CMD_RING_SLOTS      (64U)   // Must be a power of two
#define IPC_CMD_RING_MASK       (IPC_CMD_RING_SLOTS - 1U)

enum { IPC_MODE_MANUAL, IPC_MODE_AUTO, IPC_MODE_SAFE };
enum { IPC_RESULT_NONE, IPC_RESULT_OK, IPC_RESULT_UNSUPPORTED, IPC_RESULT_BAD_LEN, IPC_RESULT_BAD_VALUE, IPC_RESULT_TOO_MANY, IPC_RESULT_INCOMPLETE, IPC_RESULT_BAD_CRC };

// Return codes for the IPC API. Separate from IPC_RESULT_*, which is telemetry data sent to the ground station
typedef enum {
    IPC_STATUS_OK = 0,
    IPC_STATUS_BUSY,                // Other core holds the HSEM, try next round
    IPC_STATUS_EMPTY,               // Nothing to pop (normal, not a failure)
    IPC_STATUS_FULL,                // All ring slots in use
    IPC_STATUS_TIMEOUT,             // Attach: magic never appeared
    IPC_STATUS_VERSION_MISMATCH,    // Attach: cores built with different layouts
} ipc_status_t;

// One command. Every command uses the full record. The type sets how many raw bytes are used.
typedef struct {
    uint8_t type;                   // CommandId_e (sharedCommand.h), 0 = COMMAND_NONE
    uint8_t seq;                    // LoRa sequence number, echoed back as telemetry last_cmd_seq
    uint8_t raw[IPC_CMD_DATA_SIZE]; // Little-endian payload, decoded by the CM7. Unused bytes zeroed
} ipc_cmd_t;

// Payload layouts carried in ipc_cmd_t.raw, decoded by the CM7
typedef struct {
    uint16_t index;
    uint16_t margin_m;
    int32_t  lat_e7;
    int32_t  lon_e7;
} ipc_cmd_waypoint_t;

typedef struct {
    uint16_t count;            // number of waypoints up to 32. min 1
    uint16_t crc16;            // CRC-16 over the waypoints 0 to count-1
    uint8_t  arrive_m;         // arrival radius for all waypoints (0 = default of 8 meters)
    uint8_t  margin_warning_m; // warning zone beyond each paths's margin
    uint16_t reserved;         // 0
} ipc_cmd_route_commit_t;

// Free-running slot counts: used = head - tail, index = count & IPC_CMD_RING_MASK
typedef struct {
    volatile uint32_t head;         // Written by CM4 only
    volatile uint32_t tail;         // Written by CM7 only
    ipc_cmd_t slot[IPC_CMD_RING_SLOTS];
} ipc_cmd_ring_t;

typedef struct {
    volatile uint32_t magic;
    volatile uint32_t version;
    volatile uint32_t telem_seq;
    LoRaTelemetry_t   telem;
    ipc_cmd_ring_t    cmd;
} ipc_shared_t;

#define IPC_VERSION (((uint32_t)IPC_REV << 16) | ((uint32_t)sizeof(ipc_shared_t) & 0xFFFFU))

extern ipc_shared_t ipc_shared;

_Static_assert((IPC_CMD_RING_SLOTS & IPC_CMD_RING_MASK) == 0, "IPC_CMD_RING_SLOTS must be a power of two");
_Static_assert(sizeof(ipc_cmd_t) == 2U + IPC_CMD_DATA_SIZE, "ipc_cmd_t must have no padding");
_Static_assert(sizeof(ipc_cmd_waypoint_t)     <= IPC_CMD_DATA_SIZE, "waypoint payload must fit in ipc_cmd_t.raw");
_Static_assert(sizeof(ipc_cmd_route_commit_t) <= IPC_CMD_DATA_SIZE, "route commit payload must fit in ipc_cmd_t.raw");
_Static_assert(sizeof(ipc_shared_t) <= IPC_SHARED_SIZE, "ipc_shared_t does not fit in SRAM4");

/**
  * @brief  Get a printable name for a status code, e.g. "FULL".
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
  * @param  cmd  Command to queue, copied whole into one slot.
  * @retval IPC_STATUS_OK    Queued.
  * @retval IPC_STATUS_FULL  All slots in use, nothing queued.
  */
ipc_status_t ipc_cmd_push(const ipc_cmd_t *cmd);

/**
  * @brief  Copy the latest telemetry and its sequence number.
  * @param  telem      Destination for the telemetry.
  * @param  telem_seq  Destination for the sequence number.
  * @retval IPC_STATUS_OK    Copied.
  * @retval IPC_STATUS_BUSY  CM7 is mid-write, nothing copied.
  */
ipc_status_t ipc_telem_read(LoRaTelemetry_t *telem, uint32_t *telem_seq);
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
  * @brief  Copy the oldest command out of the ring and free its slot.
  * @param  cmd  Destination for the command.
  * @retval IPC_STATUS_OK     Command copied into cmd.
  * @retval IPC_STATUS_EMPTY  Nothing queued.
  */
ipc_status_t ipc_cmd_pop(ipc_cmd_t *cmd);

/**
  * @brief  Copy telemetry into shared memory and increment telem_seq.
  * @param  telem  Telemetry to publish.
  * @retval IPC_STATUS_OK    Published.
  * @retval IPC_STATUS_BUSY  CM4 is mid-read, nothing written.
  */
ipc_status_t ipc_telem_write(LoRaTelemetry_t *telem);
#endif

#endif
