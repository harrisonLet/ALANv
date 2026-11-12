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

// Every system mode, with each actuator command tried where it is allowed and where it is not.
// Arbiter modes per system mode come from arbiter_modes_for() in CM7/App/Src/commander.c:
// a servo command needs its actuator on MANUAL, a PID command needs it on SEMI.
// Ranges: servo +-90 (ARBITER_*_LIMIT), sail PID -180..180, rudder PID 0..359.
// The CM7 boots in AUTO (COMMANDER_BOOT_MODE). Step 0 puts it back there on every lap.
// Steps run every CM4_FAKE_CMD_PERIOD_S (3 s), under the 5 s link timeout of the manual and semi modes.
static const fake_cmd_t script[] = {
    // AUTO: rudder AUTO, tail AUTO. Heading PID holds the boot heading on the first lap, then
    // whatever the semi steps below last set (0). LSP picks the sail target from the wind.
    { COMMAND_MODE_AUTO,                  0,   CMD_RESULT_OK          },
    { COMMAND_RUDDER_SERVO,               20,  CMD_RESULT_WRONG_MODE  },
    { COMMAND_SAIL_SERVO,                 20,  CMD_RESULT_WRONG_MODE  },
    { COMMAND_RUDDER_PID,                 0,   CMD_RESULT_WRONG_MODE  },
    { COMMAND_SAIL_PID,                   30,  CMD_RESULT_WRONG_MODE  },

    // FULL_MAN: rudder MANUAL, tail MANUAL
    { COMMAND_MODE_FULL_MAN,              0,   CMD_RESULT_OK          },
    { COMMAND_RUDDER_SERVO,               30,  CMD_RESULT_OK          },
    { COMMAND_SAIL_SERVO,                 -30, CMD_RESULT_OK          },
    { COMMAND_RUDDER_SERVO,               91,  CMD_RESULT_BAD_VALUE   }, // one past the limit
    { COMMAND_SAIL_PID,                   30,  CMD_RESULT_WRONG_MODE  },

    // AUTO_SAIL_MAN_RUDDER: rudder MANUAL, tail AUTO
    { COMMAND_MODE_AUTO_SAIL_MAN_RUDDER,  0,   CMD_RESULT_OK          },
    { COMMAND_RUDDER_SERVO,               -20, CMD_RESULT_OK          },
    { COMMAND_SAIL_SERVO,                 20,  CMD_RESULT_WRONG_MODE  },

    // AUTO_RUDDER_MAN_SAIL: rudder AUTO, tail MANUAL
    { COMMAND_MODE_AUTO_RUDDER_MAN_SAIL,  0,   CMD_RESULT_OK          },
    { COMMAND_SAIL_SERVO,                 45,  CMD_RESULT_OK          },
    { COMMAND_RUDDER_SERVO,               10,  CMD_RESULT_WRONG_MODE  },

    // FULL_SEMI: rudder SEMI, tail SEMI. Heading 0 = the default heading the AUTO steps then hold.
    { COMMAND_MODE_FULL_SEMI,             0,   CMD_RESULT_OK          },
    { COMMAND_RUDDER_PID,                 0,   CMD_RESULT_OK          },
    { COMMAND_SAIL_PID,                   30,  CMD_RESULT_OK          },
    { COMMAND_RUDDER_PID,                 400, CMD_RESULT_BAD_VALUE   }, // past 359
    { COMMAND_SAIL_SERVO,                 10,  CMD_RESULT_WRONG_MODE  },

    // SEMI_RUDDER_MAN_SAIL: rudder SEMI, tail MANUAL
    { COMMAND_MODE_SEMI_RUDDER_MAN_SAIL,  0,   CMD_RESULT_OK          },
    { COMMAND_RUDDER_PID,                 90,  CMD_RESULT_OK          },
    { COMMAND_SAIL_SERVO,                 -45, CMD_RESULT_OK          },

    // SEMI_RUDDER_AUTO_SAIL: rudder SEMI, tail AUTO
    { COMMAND_MODE_SEMI_RUDDER_AUTO_SAIL, 0,   CMD_RESULT_OK          },
    { COMMAND_RUDDER_PID,                 0,   CMD_RESULT_OK          }, // back to the default heading
    { COMMAND_SAIL_PID,                   30,  CMD_RESULT_WRONG_MODE  },

    // SEMI_SAIL_MAN_RUDDER: rudder MANUAL, tail SEMI
    { COMMAND_MODE_SEMI_SAIL_MAN_RUDDER,  0,   CMD_RESULT_OK          },
    { COMMAND_SAIL_PID,                   -30, CMD_RESULT_OK          },
    { COMMAND_RUDDER_SERVO,               15,  CMD_RESULT_OK          },

    // SEMI_SAIL_AUTO_RUDDER: rudder AUTO, tail SEMI
    { COMMAND_MODE_SEMI_SAIL_AUTO_RUDDER, 0,   CMD_RESULT_OK          },
    { COMMAND_SAIL_PID,                   0,   CMD_RESULT_OK          },
    { COMMAND_RUDDER_PID,                 90,  CMD_RESULT_WRONG_MODE  }, // target stays 0

    // Full AUTO again, holding heading 0 from the semi steps
    { COMMAND_MODE_AUTO,                  0,   CMD_RESULT_OK          },
    { COMMAND_NONE,                       0,   CMD_RESULT_NONE        }, // heartbeat: telemetry keeps the previous seq and result

    // STANDBY and SAFE: both actuators SAFE, every actuator command refused
    { COMMAND_MODE_STANDBY,               0,   CMD_RESULT_OK          },
    { COMMAND_RUDDER_SERVO,               10,  CMD_RESULT_WRONG_MODE  },
    { COMMAND_MODE_SAFE,                  0,   CMD_RESULT_OK          },
    { COMMAND_MODE_SAFE,                  0,   CMD_RESULT_OK          }, // already there: still OK
    { COMMAND_SAIL_PID,                   30,  CMD_RESULT_WRONG_MODE  },
    { 0x4F,                               0,   CMD_RESULT_UNSUPPORTED }, // no 0x4X commands exist
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
