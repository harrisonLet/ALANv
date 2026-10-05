#ifndef CROSS_CORE_COM_H
#define CROSS_CORE_COM_H

#include <stdint.h>
#include <stddef.h>

#define IPC_HSEM_TELEM          (1U)
#define IPC_HSEM_CMD_BELL       (2U)

#define IPC_SHARED_BASE         (0x38000000UL)
#define IPC_SHARED_SIZE         (64UL * 1024UL)
#define IPC_MAGIC               (0x49504331UL)
#define IPC_REV                 (1U)

#define IPC_CMD_RING_SIZE       (2048U)
#define IPC_CMD_ALIGN           (4U)
#define IPC_CMD_MAX_PAYLOAD     (240U)

#define IPC_ALIGN_UP(n, a)          (((n) + ((a) - 1U)) & ~((a) - 1U))
#define IPC_CMD_RECORD_SIZE(len)    IPC_ALIGN_UP(sizeof(ipc_cmd_hdr_t) + (len), IPC_CMD_ALIGN)

enum { IPC_MODE_MANUAL, IPC_MODE_AUTO, IPC_MODE_SAFE };
enum { IPC_CMD_MANUAL_STEP = 1, IPC_CMD_SHUTDOWN, IPC_CMD_WAYPOINT, IPC_CMD_ROUTE_COMMIT };
enum { IPC_RESULT_NONE, IPC_RESULT_OK, IPC_RESULT_UNSUPPORTED, IPC_RESULT_BAD_LEN };

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

#endif
