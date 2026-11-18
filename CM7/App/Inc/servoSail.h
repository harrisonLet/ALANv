#ifndef SERVO_SAIL_H
#define SERVO_SAIL_H

#define SERVO_SAIL_TASK "[SERVSAIL]"

#ifdef CM7_SERVO_SAIL_DEBUG
#define servoSail_printf(fmt, ...) DEBUG_PRINTF(SERVO_SAIL_TASK, fmt, ##__VA_ARGS__)
#else
#define servoSail_printf(...) ((void)0)
#endif

void servoSail_hardwareInit(void);
void servoSail_setAngle(int16_t angle);

#endif