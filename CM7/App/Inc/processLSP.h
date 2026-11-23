#ifndef PROCESS_LSP_H
#define PROCESS_LSP_H

#include "system.h"

#define PROCESS_LSP_TASK "[LSP-----]"
#define PROCESS_LSP_TASK_PRIORITY osPriorityLow

#ifdef CM7_PROCESS_LSP_DEBUG
#define PROCESS_LSP_TASK_HEAP (RTOS_MIN_STACK+128)
#define processLSP_printf(fmt, ...) DEBUG_PRINTF(PROCESS_LSP_TASK, fmt, ##__VA_ARGS__)
#else
#define PROCESS_LSP_TASK_HEAP (RTOS_MIN_STACK+0)
#define processLSP_printf(...) ((void)0)
#endif

#define PROCESS_LSP_FREQUENCY 2u // Hz
#define PROCESS_LSP_DEADBAND 3   // degrees of oscillation thhat the LSP will ignore

// Ease the sail when the rudder is bearing away hard, so the sail stops pushing the bow up
#define PROCESS_LSP_BEARAWAY_ON  20  // deg of rudder turn away from the wind to start easing
#define PROCESS_LSP_BEARAWAY_OFF 10  // deg of rudder turn to stop easing (hysteresis)
#define PROCESS_LSP_BEARAWAY_MAX_WIND 105 // only ease when the wind is forward of a broad reach

// Angle of attack per sailing zone
#define PROCESS_LSP_AOA_IRONS 0
#define PROCESS_LSP_AOA_CLOSEHAULED 15
#define PROCESS_LSP_AOA_CLOSEREACH 20
#define PROCESS_LSP_AOA_BEAMREACH 30
#define PROCESS_LSP_AOA_BROADREACH 45
#define PROCESS_LSP_AOA_TRAININGRUN 55 // we likely won't be able to hold this angle of attack
#define PROCESS_LSP_AOA_RUNNING 90 // the tail physically can't hold this angle of attack but we will still try

extern TaskHandle_t processLSP_task;

HardwareInit_e processLSP_hardwareInit(void);
RtosInit_e processLSP_rtosInit(void);
void processLSP_handler(void *argument);

#endif // PROCESS_LSP_H