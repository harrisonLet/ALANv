#ifndef CONTROLLER_H
#define CONTROLLER_H

#define LIVE_TASK "[LIVE----]"

#ifdef CM7_DEBUG
#define cm7_printf(fmt, ...) DEBUG_PRINTF("[CM7-----]", fmt, ##__VA_ARGS__)
#else
#define cm7_printf(...) ((void)0)
#endif

#ifdef CM7_LIVE_DEBUG
#define LIVE_TASK_PRIORITY osPriorityHigh
#define LIVE_TASK_HEAP 300
#define live_printf(fmt, ...) DEBUG_PRINTF(LIVE_TASK, fmt, ##__VA_ARGS__)
#else
#define LIVE_TASK_PRIORITY osPriorityLow
#define LIVE_TASK_HEAP 32
#define live_printf(...) ((void)0)
#endif

void hardware_init(void);
void rtos_init(void);

#endif // CONTROLLER_H