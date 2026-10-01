#ifndef SERVO_RUDDER_H
#define SERVO_RUDDER_H

#define SERVO_RUDDER_TASK "[SERVO_RUDDER]"

void servoRudder_hardwareInit(void);
void servoRudder_setAngle(int16_t angle);

#endif
