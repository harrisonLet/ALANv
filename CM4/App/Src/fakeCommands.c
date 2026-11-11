#include "shared_configuration.h"

#ifdef CM4_FAKE_COMMANDS

#include "fakeCommands.h"
#include "lora.h"
#include "ipc.h"
#include "sharedCommand.h"
#include <string.h>

_Static_assert(FAKE_CMD_PACKET_LEN == LORA_HDR_LEN_BYTES + IPC_CMD_DATA_SIZE,
               "a fake packet must match the length main.c accepts");

typedef struct {
    uint8_t type;   // CommandId_e
    int16_t value;  // little-endian in raw[0..1], degrees for servo and PID commands
    uint8_t expect; // cmd_result_t the CM7 should report, for reading the results by eye
} fake_cmd_t;

// The CM7 boots in AUTO (COMMANDER_BOOT_MODE). Step 0 puts it back there on every lap.
static const fake_cmd_t script[] = {
    { COMMAND_MODE_AUTO,                 0,   CMD_RESULT_OK          },
    { COMMAND_SAIL_SERVO,                20,  CMD_RESULT_WRONG_MODE  }, // sail is not manual in AUTO
    { COMMAND_MODE_AUTO_RUDDER_MAN_SAIL, 0,   CMD_RESULT_OK          },
    { COMMAND_SAIL_SERVO,                20,  CMD_RESULT_OK          },
    { COMMAND_SAIL_SERVO,                200, CMD_RESULT_BAD_VALUE   }, // past ARBITER_TAIL_LIMIT
    { COMMAND_NONE,                      0,   CMD_RESULT_NONE        }, // heartbeat: telemetry keeps the previous seq and result
    { COMMAND_MODE_FULL_SEMI,            0,   CMD_RESULT_OK          },
    { COMMAND_SAIL_PID,                  30,  CMD_RESULT_OK          },
    { 0x4F,                              0,   CMD_RESULT_UNSUPPORTED }, // no 0x4X commands exist
    { COMMAND_MODE_SAFE,                 0,   CMD_RESULT_OK          },
};
#define SCRIPT_LEN (sizeof(script) / sizeof(script[0]))

static uint8_t step = 0;
static uint8_t seq = 0; // the CM7 expects the first command to be seq 0

uint8_t fakeCmd_next(uint8_t *buf)
{
    const fake_cmd_t *cmd = &script[step];

    memset(buf, 0, FAKE_CMD_PACKET_LEN);
    buf[0] = ADDR_STM32;                 // dest
    buf[1] = ADDR_SAMD21;                // src
    buf[2] = seq++;                      // seq, wraps 255 -> 0 like the commander expects
    buf[3] = 1U + IPC_CMD_DATA_SIZE;     // len: type byte + raw payload (17)
    buf[4] = cmd->type;
    buf[5] = (uint8_t)((uint16_t)cmd->value & 0xFFU);
    buf[6] = (uint8_t)((uint16_t)cmd->value >> 8);

    step = (uint8_t)((step + 1U) % SCRIPT_LEN);
    return FAKE_CMD_PACKET_LEN;
}

#endif // CM4_FAKE_COMMANDS
