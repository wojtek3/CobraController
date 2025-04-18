/*
 * ahrs.h
 *
 *  Created on: Apr 2, 2025
 *      Author: wwisz
 */

#ifndef INC_AHRS_H_
#define INC_AHRS_H_


#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    // Orientation Quaternion
    float q0, q1, q2, q3;

    // Rotation matrix
    float rotMat[3][3];

    // Euler angles (degrees)
    float roll;   // -180..180
    float pitch;  // -90..+90
    float yaw;    // 0..360

    // integral element filter
    float exInt;
    float eyInt;
    float ezInt;

    // Filter multipliers
    float kp;
    float ki;

    // Dynamic modified Kp multiplier
    float dynamicKp;

    // Timer since boot
    float timeSinceStartup;

    // Variables to check time without move
    float timeWithoutMotion;
    bool  wasArmed;
    bool  justArmed;

} ahrs_t;

/**
 * AHRS initialization.
 *  - Sets quaternion to (1,0,0,0),
 *  - Sets rotation matrix to I,
 *  - Resets euler angles and integral element.
 *  - Sets multipliers to default values.
 */
void ahrsInit(ahrs_t *ahrs);

/**
 * Main AHRS compute function.
 *  @param ahrs       - pointer to ahrs_t
 *  @param gx, gy, gz - gyro data in [rad/s]
 *  @param ax, ay, az - accelerometer data in [g]
 *  @param accHealthy - Checks if acc data is valid for correction (betweel ACC_MIN and ACC_MAX)
 *  @param armed      - drone armed
 *  @param dt         - time between executions
 */
void ahrsUpdate(ahrs_t *ahrs,
                float gx, float gy, float gz,
                float ax, float ay, float az,
                bool accHealthy,
                bool armed,
                float dt);

#ifdef __cplusplus
}
#endif


#endif /* INC_AHRS_H_ */
