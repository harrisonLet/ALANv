#ifndef PID_HEADING_H
#define PID_HEADING_H

#include "system.h"

#define PID_HEADING_TASK "[PID_HEAD]"
#define PID_HEADING_TASK_PRIORITY osPriorityNormal
#define PID_HEADING_TASK_HEAP 300

#ifdef CM7_PID_HEADING_DEBUG
#define pidHeading_printf(fmt, ...) DEBUG_PRINTF(PID_HEADING_TASK, fmt, ##__VA_ARGS__)
#else
#define pidHeading_printf(...) ((void)0)
#endif

#define PID_HEADING_FREQUENCY 20u // Hz
#define PID_HEADING_KP 1.0f   // deg of rudder per deg of heading error
#define PID_HEADING_KI 0.05f
#define PID_HEADING_KD 0.0f
#define PID_HEADING_RUDDER_LIMIT 35.0f // degree limit for either side of the rudder
#define PID_HEADING_HOLD_DELAY 5000u  // ms after the first heading before locking the target
#define PID_HEADING_FROZEN 3000u   // ms without any heading change before the IMU is treated as dead

extern TaskHandle_t pidHeading_task;

HardwareInit_e pidHeading_hardwareInit(void);
RtosInit_e pidHeading_rtosInit(void);
void pidHeading_handler(void *argument);

void pidHeading_setTarget(uint16_t heading);
int16_t pidHeading_getTarget(void); // 0 to 359, or -1 before a target is set or held
// Commander turns the PID on and off. While off it sends nothing, and it restarts when turned back on.
void pidHeading_setEnabled(bool enabled);

#endif // PID_HEADING_H