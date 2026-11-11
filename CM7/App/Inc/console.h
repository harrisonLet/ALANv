#ifndef CONSOLE_H
#define CONSOLE_H

#include "FreeRTOS.h"
#include "task.h"

#define CONSOLE_TASK "[CONSOLE-]"
#define CONSOLE_TASK_PRIORITY osPriorityLow

#ifdef CM7_CONSOLE_DEBUG
#define CONSOLE_TASK_HEAP 300
#define console_printf(fmt, ...) DEBUG_PRINTF(CONSOLE_TASK, fmt, ##__VA_ARGS__)
#else
#define CONSOLE_TASK_HEAP 300
#define console_printf(...) ((void)0)
#endif

extern TaskHandle_t task_console;

void console_hardwareInit(void);
void console_handler(void *argument);

#endif
