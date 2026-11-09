#ifndef PID_H
#define PID_H

#include <stdbool.h>

/// Generic PID controller used by the heading and sail loops.

typedef struct {
    // settings
    float kp;
    float ki;
    float kd;
    float out_min;          // output clamp
    float out_max;

    // state, changes every update
    float integral;         // running I total
    float prev_measurement; // last measurement, for the D term
    bool  first_run;        // true until the first update, so D is skipped once
} PIDController;

void  pid_init(PIDController *pid, float kp, float ki, float kd, float out_min, float out_max);
void  pid_reset(PIDController *pid);
void  pid_setGains(PIDController *pid, float kp, float ki, float kd);
float pid_wrap180(float angle);
float pid_update(PIDController *pid, float setpoint, float measurement, float dt);

#endif // PID_H