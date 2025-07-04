/**
 * @file PID_lib.c
 * @brief Implementation of a floating point PID controller using explicit pointers.
 */

#include "lib/PID_lib/PID_lib.h"
#include "pico/time.h"

void pid_create(pid_controller_t *pid, float *in, uint16_t *out, float set, float kp, float ki, float kd, float omin, float omax)
{
    pid->input = in;
    pid->output = out;
    pid->setpoint = set;

    pid->automode = true;
    pid->sampletime = 100; // Default sample time (ms)
    pid->omin = omin;
    pid->omax = omax;

    pid_direction(pid, E_PID_DIRECT);
    pid_tune(pid, kp, ki, kd);

    pid->iterm = 0.0f;
    pid->lastin = *in;
}

void pid_compute(pid_controller_t *pid)
{

    if (!pid->automode)
        return;

    float input = *(pid->input);                                // Read the current input value
    float error = pid->setpoint - input;                         // Calculate the error (setpoint - input)

    pid->iterm += pid->Ki * error;                              // Update the integral term
    if (pid->iterm > pid->omax) pid->iterm = pid->omax;
    else if (pid->iterm < pid->omin) pid->iterm = pid->omin;

    float dinput = input - pid->lastin;                              // Calculate the change in input for the derivative term
    float output = pid->Kp * error + pid->iterm - pid->Kd * dinput;  // Calculate the PID output

    if (output > pid->omax) output = pid->omax;
    else if (output < pid->omin) output = pid->omin;

    *(pid->output) = output;
    pid->lastin = input;
}

void pid_tune(pid_controller_t *pid, float kp, float ki, float kd)
{
    if (kp < 0 || ki < 0 || kd < 0)
        return;

    float dt = (float)(pid->sampletime) / 1000.0f;

    pid->Kp = kp;
    pid->Ki = ki * dt;
    pid->Kd = kd / dt;

    if (pid->direction == E_PID_REVERSE) {
        pid->Kp = -pid->Kp;
        pid->Ki = -pid->Ki;
        pid->Kd = -pid->Kd;
    }
}

void pid_sample(pid_controller_t *pid, uint32_t time_ms)
{
    if (time_ms > 0) {
        float ratio = (float)time_ms / (float)pid->sampletime;
        pid->Ki *= ratio;
        pid->Kd /= ratio;
        pid->sampletime = time_ms;
    }
}

void pid_limits(pid_controller_t *pid, float min, float max)
{
    if (min >= max) return;

    pid->omin = min;
    pid->omax = max;

    if (pid->automode) {
        if (*(pid->output) > max) *(pid->output) = max;
        else if (*(pid->output) < min) *(pid->output) = min;

        if (pid->iterm > max) pid->iterm = max;
        else if (pid->iterm < min) pid->iterm = min;
    }
}

void pid_auto(pid_controller_t *pid)
{
    if (!pid->automode) {
        pid->iterm = *(pid->output);
        pid->lastin = *(pid->input);

        if (pid->iterm > pid->omax) pid->iterm = pid->omax;
        else if (pid->iterm < pid->omin) pid->iterm = pid->omin;

        pid->automode = true;
    }
}

void pid_manual(pid_controller_t *pid)
{
    pid->automode = false;
}

void pid_direction(pid_controller_t *pid, pid_control_direction_t dir)
{
    if (pid->automode && pid->direction != dir) {
        pid->Kp = -pid->Kp;
        pid->Ki = -pid->Ki;
        pid->Kd = -pid->Kd;
    }
    pid->direction = dir;
}
