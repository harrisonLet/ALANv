#ifndef PROCESS_LSP_H
#define PROCESS_LSP_H

#include "system.h"

#define PROCESS_LSP_TASK "[LSP-----]"
#define PROCESS_LSP_TASK_PRIORITY osPriorityLow

#ifdef CM7_PROCESS_LSP_DEBUG
#define PROCESS_LSP_TASK_HEAP 300
#define processLSP_printf(fmt, ...) DEBUG_PRINTF(PROCESS_LSP_TASK, fmt, ##__VA_ARGS__)
#else
#define PROCESS_LSP_TASK_HEAP 300
#define processLSP_printf(...) ((void)0)
#endif

#define PROCESS_LSP_FREQUENCY 2u // Hz
#define PROCESS_LSP_DEADBAND 3   // degrees of oscillation thhat the LSP will ignore

// Angle of attack per sailing zone
#define PROCESS_LSP_AOA_IRONS 0
#define PROCESS_LSP_AOA_CLOSEHAULED 30
#define PROCESS_LSP_AOA_CLOSEREACH 35
#define PROCESS_LSP_AOA_BEAMREACH 40
#define PROCESS_LSP_AOA_BROADREACH 45
#define PROCESS_LSP_AOA_TRAININGRUN 55 // we likely won't be able to hold this angle of attack
#define PROCESS_LSP_AOA_RUNNING 90 // the tail physically can't hold this angle of attack but we will still try

extern TaskHandle_t processLSP_task;

HardwareInit_e processLSP_hardwareInit(void);
RtosInit_e processLSP_rtosInit(void);
void processLSP_handler(void *argument);

#endif // PROCESS_LSP_H