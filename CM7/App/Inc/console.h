#ifndef CONSOLE_H
#define CONSOLE_H

#include "FreeRTOS.h"
#include "task.h"

// Serial command console on the ST-LINK virtual COM port (USART3, 115200 baud).
// Type commands in a terminal (screen) to drive the servos by hand; see console.c for the list.
// Enabled with CM7_CONSOLE_ENABLE.

extern TaskHandle_t task_console;

void console_hardwareInit(void);
void console_handler(void *argument);

#endif
