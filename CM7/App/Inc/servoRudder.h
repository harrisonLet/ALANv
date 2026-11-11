#ifndef SERVO_RUDDER_H
#define SERVO_RUDDER_H

#define SERVO_RUDDER_TASK "[SERVRUDD]"

#ifdef CM7_SERVO_RUDDER_DEBUG
#define servoRudder_printf(fmt, ...) DEBUG_PRINTF(SERVO_RUDDER_TASK, fmt, ##__VA_ARGS__)
#else
#define servoRudder_printf(...) ((void)0)
#endif

void servoRudder_hardwareInit(void);
void servoRudder_setAngle(int16_t angle);

#endif
