#ifndef PROCESS_LSP_H
  #define PROCESS_LSP_H

  #include "system.h"

  #define PROCESS_LSP_TASK "[PROLSP-]"
  #define PROCESS_LSP_TASK_PRIORITY osPriorityLow
  #define PROCESS_LSP_TASK_HEAP 300

  #ifdef CM7_PROCESS_LSP_DEBUG
  #define processLSP_printf(fmt, ...) DEBUG_PRINTF(PROCESS_LSP_TASK, fmt, ##__VA_ARGS__)
  #else
  #define processLSP_printf(...) ((void)0)
  #endif

  #define PROCESS_LSP_FREQUENCY 2u // Hz
  #define PROCESS_LSP_DEADBAND 3   // degrees of oscillation thhat the LSP will ignore

  // Angle of attack per sailing zone
  #define PROCESS_LSP_AOA_IRONS 0
  #define PROCESS_LSP_AOA_CLOSEHAULED 5
  #define PROCESS_LSP_AOA_CLOSEREACH 10
  #define PROCESS_LSP_AOA_BEAMREACH 15
  #define PROCESS_LSP_AOA_BROADREACH 25
  #define PROCESS_LSP_AOA_TRAININGRUN 45
  #define PROCESS_LSP_AOA_RUNNING 90

  extern TaskHandle_t processLSP_task;

  HardwareInit_e processLSP_hardwareInit(void);
  RtosInit_e processLSP_rtosInit(void);
  void processLSP_handler(void *argument);

  #endif // PROCESS_LSP_H