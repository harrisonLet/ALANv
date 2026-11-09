#include "pid.h"

static float clamp(float value, float low, float high)
{
    if (value < low)  { return low; }
    if (value > high) { return high; }
    return value;
}

void pid_init(PIDController *pid, float kp, float ki, float kd, float out_min, float out_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->out_min = out_min;
    pid->out_max = out_max;
    pid_reset(pid);
}

void pid_reset(PIDController *pid)
{
    pid->integral = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->first_run = true;
}

void pid_setGains(PIDController *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

float pid_wrap180(float angle)
{
    while (angle > 180.0f)  { angle -= 360.0f; }
    while (angle < -180.0f) { angle += 360.0f; }
    return angle;
}

/**
  * Run one step of the controller and return the clamped output.
  *
  * setpoint and measurement are angles in degrees, dt is the time since the last
  * update in seconds.
  */
float pid_update(PIDController *pid, float setpoint, float measurement, float dt)
{
    float error = pid_wrap180(setpoint - measurement);

    // Push in proportion to the error
    float p_term = pid->kp * error;

    // Slows down how fast the measurement is changing
    float d_term = 0.0f;
    if (!pid->first_run && dt > 0.0f) {
        float change = pid_wrap180(measurement - pid->prev_measurement);
        d_term = -pid->kd * change / dt;
    }

    // Only keep integrating if the output isn't already pinned in the same direction
    float test = p_term + pid->integral + d_term;
    bool stuck_high = (test >= pid->out_max) && (error > 0.0f);
    bool stuck_low  = (test <= pid->out_min) && (error < 0.0f);

    if (!stuck_high && !stuck_low) {
        pid->integral += pid->ki * error * dt;
    }
    pid->integral = clamp(pid->integral, pid->out_min, pid->out_max);

    pid->prev_measurement = measurement;
    pid->first_run = false;

    return clamp(p_term + pid->integral + d_term, pid->out_min, pid->out_max);
}
