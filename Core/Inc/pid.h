#ifndef PID_H
#define PID_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Simple PID object for one axis.
 */
typedef struct {
    float kp;               ///< Proportional gain
    float ki;               ///< Integral gain
    float kd;               ///< Derivative gain

    float integrator;       ///< Integrator state
    float lastError;        ///< For derivative calculation

    float integratorLimit;  ///< Maximum absolute value of the integrator
} PID_t;

/**
 * Global PID structures for Roll, Pitch, Yaw.
 */
extern PID_t rollPID;
extern PID_t pitchPID;
extern PID_t yawPID;

/**
 * @brief Initializes the PID controllers (sets default gains, zeroes integrators).
 */
void pidInit(void);

/**
 * @brief Sets the PID gains and integrator limit.
 * @param pid pointer to PID_t instance
 * @param kp  proportional gain
 * @param ki  integral gain
 * @param kd  derivative gain
 * @param integratorLimit max absolute value for integrator
 */
void pidSetGains(PID_t *pid, float kp, float ki, float kd, float integratorLimit);

/**
 * @brief Computes one PID output for a single axis.
 *        Output is clamped to [1000..2000], with 1500 meaning "no correction."
 * @param pid       pointer to PID_t instance
 * @param setpoint  desired angle (e.g. 0 deg)
 * @param measured  actual angle (e.g. from ahrs->roll, ahrs->pitch, or ahrs->yaw)
 * @param dt        delta time (seconds)
 * @return          16-bit integer in range [1000..2000]
 */
float pidComputeRate(PID_t *pid, float setpoint, float measured, float dt);

#ifdef __cplusplus
}
#endif

#endif // PID_H
