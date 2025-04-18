#include "pid.h"
#include "mixer.h"

/**
 * Global PID objects for roll, pitch, yaw
 */
PID_t rollPID;
PID_t pitchPID;
PID_t yawPID;

/**
 * @brief Initialize default PID gains, zero integrators, etc.
 */
void pidInit(void)
{
    // Example default gains; tune as necessary for your system
    pidSetGains(&rollPID,  3.0f, 0.0f, 0.0f, 300.0f);
    pidSetGains(&pitchPID, 3.0f, 0.0f, 0.0f, 300.0f);
    pidSetGains(&yawPID,   3.0f, 0.0f, 0.0f, 300.0f);

    // Reset integrators and lastError
    rollPID.integrator  = 0.0f;
    rollPID.lastError   = 0.0f;
    pitchPID.integrator = 0.0f;
    pitchPID.lastError  = 0.0f;
    yawPID.integrator   = 0.0f;
    yawPID.lastError    = 0.0f;
}

/**
 * @brief Helper function to set gains on a PID object
 */
void pidSetGains(PID_t *pid, float kp, float ki, float kd, float integratorLimit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integratorLimit = integratorLimit;
}

/**
 * @brief Compute one PID axis. Output is 1000..2000, with 1500 meaning "zero correction"
 */
float pidComputeRate(PID_t *pid, float setpoint,
                     float measured, float dt)
{
    /* 1. error and derivative */
    const float error   = setpoint - measured;
    const float dError  = (error - pid->lastError) / dt;
    pid->lastError      = error;

    /* 2. integrator with anti‑wind‑up */
    pid->integrator += error * pid->ki * dt;
    if (pid->integrator >  pid->integratorLimit) pid->integrator =  pid->integratorLimit;
    if (pid->integrator < -pid->integratorLimit) pid->integrator = -pid->integratorLimit;

    /* 3. return physical output (e.g. deg/s or dps → servo mix later) */
    return  (pid->kp * error) +
            pid->integrator   +
            (pid->kd * dError);
}
