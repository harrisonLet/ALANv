#ifndef CONTROLLER_H
#define CONTROLLER_H

#define LIVE_TASK "[LIVE----]"

#ifdef CM7_DEBUG
#define cm7_printf(fmt, ...) DEBUG_PRINTF("[CM7-----]", fmt, ##__VA_ARGS__)
#else
#define cm7_printf(...) ((void)0)
#endif

#define LIVE_TASK_PRIORITY osPriorityLow

#ifdef CM7_LIVE_DEBUG
#define LIVE_TASK_HEAP (RTOS_MIN_STACK+512)
#define live_printf(fmt, ...) DEBUG_PRINTF(LIVE_TASK, fmt, ##__VA_ARGS__)
#define liveStack_printf(fmt, ...) DEBUG_PRINTF("[LIVE-ST-]", fmt, ##__VA_ARGS__)
#define liveCpu_printf(fmt, ...) DEBUG_PRINTF("[LIVE-CPU]", fmt, ##__VA_ARGS__)
#else
#define LIVE_TASK_HEAP (RTOS_MIN_STACK+0)
#define live_printf(...) ((void)0)
#define liveStack_printf(...) ((void)0)
#define liveCpu_printf(...) ((void)0)
#endif

void hardware_init(void);
void rtos_init(void);

#endif // CONTROLLER_H