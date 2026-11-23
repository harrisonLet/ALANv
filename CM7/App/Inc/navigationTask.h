#ifndef NAVIGATION_TASK_H
#define NAVIGATION_TASK_H

// Live Tack Planning (LTP) task: gathers the inputs, runs navTack_step() and sends the heading
// to the heading PID. All of the planning is in navigationTack.c, which has no RTOS calls.

#include "system.h"
#include "sharedNavigation.h"

#define NAVIGATION_TASK "[LTP-----]"
#define NAVIGATION_TASK_PRIORITY osPriorityNormal
#define NAVIGATION_TASK_HEAP 512 // navTack_step works in doubles, plus the debug printf

#ifdef CM7_NAVIGATION_DEBUG
#define navigationTask_printf(fmt, ...) DEBUG_PRINTF(NAVIGATION_TASK, fmt, ##__VA_ARGS__)
#else
#define navigationTask_printf(...) ((void)0)
#endif

extern TaskHandle_t navigationTask_task;

RtosInit_e navigationTask_rtosInit(void);
void navigationTask_handler(void *argument);

/**
  * @brief  Latest LTP output.
  * @retval true   Copied.
  * @retval false  LTP is not built in, or has not run its first cycle yet. telemetry is unchanged.
  */
bool navigationTask_getTelemetry(TelemetryNavigation_t *telemetry);

#endif // NAVIGATION_TASK_H
