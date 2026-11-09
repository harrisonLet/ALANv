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

#define COMMANDER_FREQ 100U // Hz, NEEDS TESTING
_Static_assert(COMMANDER_FREQ > 0u && COMMANDER_FREQ <= 1000u, "SENSOR_IMU_FREQUENCY must be 1Hz-1KHz");
#if (SYSTEM_TICK_RATE_HZ % COMMANDER_FREQ) != 0u
#warning "SENSOR_IMU_FREQUENCY does not divide the tick rate, the period will be rounded down"
#endif

extern TaskHandle_t commander_task;

HardwareInit_e commander_hardware_init(void);
RtosInit_e commander_rtos_init(void);

void commander_handler(void *argument);

cmd_result_t execute_cmd(ipc_cmd_t *cmd);
cmd_result_t switch_mode(system_mode_t mode);
cmd_result_t rudder_sail_cntrl(ipc_cmd_t *cmd);
cmd_result_t execute_servo(ipc_cmd_t *cmd);
