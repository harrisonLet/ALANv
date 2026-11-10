#pragma once

#include "ipcCM7.h"
#include "system.h"
#include "sharedCommand.h"

#define COMMANDER_TASK "[COMMANDR]"
#define COMMANDER_TASK_PRIORITY osPriorityHigh

#ifdef CM7_COMMANDER_DEBUG
#define CM7_COMMANDER_HEAP 400 
#define commander_printf(fmt, ...) DEBUG_PRINTF(COMMANDER_TASK, fmt, ##__VA_ARGS__)
#else
#define CM7_COMMANDER_HEAP 256 // ~64B of extra
#define commander_printf(...) ((void)0)
#endif

// Mode the boat starts in. Keep AUTO while commands can't get past the CM4, otherwise the
// sail chain never runs on the bench. Change to SYSTEM_MODE_SAFE once the radio path works.
#define COMMANDER_BOOT_MODE SYSTEM_MODE_AUTO

#ifdef COMMANDER_SELFTEST
#ifndef CM7_COMMANDER_DEBUG
#error "COMMANDER_SELFTEST prints through commander_printf, enable CM7_COMMANDER_DEBUG"
#endif
#endif

// Modes with a manual or semi-auto part fall back to SAFE after this long with no command, including COMMAND_NONE heartbeats
#define COMMANDER_LINK_TIMEOUT_MS 5000U

#define COMMANDER_FREQ 100U // Hz, NEEDS TESTING
_Static_assert(COMMANDER_FREQ > 0u && COMMANDER_FREQ <= 1000u, "COMMANDER_FREQ must be 1Hz-1KHz");
#if (SYSTEM_TICK_RATE_HZ % COMMANDER_FREQ) != 0u
#warning "COMMANDER_FREQ does not divide the tick rate, the period will be rounded down"
#endif

extern TaskHandle_t commander_task;

HardwareInit_e commander_hardware_init(void);
RtosInit_e commander_rtos_init(void);

void commander_handler(void *argument);

cmd_result_t execute_cmd(ipc_cmd_t *cmd);
cmd_result_t switch_mode(system_mode_t mode);
cmd_result_t rudder_sail_cntrl(ipc_cmd_t *cmd);
cmd_result_t execute_servo(ipc_cmd_t *cmd);
cmd_result_t execute_pid(ipc_cmd_t *cmd);
cmd_result_t navigation_cmd(ipc_cmd_t *cmd);

// Single bytes, safe to read from another task
uint8_t commander_getMode(void);
uint8_t commander_getLastSeq(void);
uint8_t commander_getLastResult(void);
