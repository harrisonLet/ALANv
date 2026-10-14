#ifndef CONTROLLER_H
#define CONTROLLER_H

#define LIVE_TASK "[MLIVE---]"
#define LIVE_TASK_PRIORITY osPriorityLow

#ifdef CM7_LIVE_DEBUG
#define LIVE_TASK_HEAP 180
#define live_printf(fmt, ...) DEBUG_PRINTF(LIVE_TASK, fmt, ##__VA_ARGS__)
#else
#define LIVE_TASK_HEAP 32
#define live_printf(...) ((void)0)
#endif

void hardware_init(void);
void rtos_init(void);

#endif // CONTROLLER_H