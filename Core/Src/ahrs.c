/*
 * ahrs.c
 *
 *  Created on: Apr 2, 2025
 *      Author: wwisz
 */

#include "ahrs.h"
#include <math.h>
#include <float.h>
#include "config.h"

#ifndef RAD2DEG
#define RAD2DEG (57.295779513082320876f)
#endif



// time since boot with applied higher Kp
#define INITIAL_HIGH_KP_DURATION 20.0f

// boot time kP multiplier
#define HIGH_KP_FACTOR 10.0f

#define GYRO_SPIN_THRESHOLD (20.0f * (float)M_PI / 180.0f)


static float invSqrtf(float x)
{
    return 1.0f / sqrtf(x);
}


//Converts quaternion (q0..q3) to rotation matrix 3x3 and saves in ahrs->rotMat.
static void buildRotationMatrix(ahrs_t *ahrs)
{
    float q0q0 = ahrs->q0 * ahrs->q0;
    float q0q1 = ahrs->q0 * ahrs->q1;
    float q0q2 = ahrs->q0 * ahrs->q2;
    float q0q3 = ahrs->q0 * ahrs->q3;
    float q1q1 = ahrs->q1 * ahrs->q1;
    float q1q2 = ahrs->q1 * ahrs->q2;
    float q1q3 = ahrs->q1 * ahrs->q3;
    float q2q2 = ahrs->q2 * ahrs->q2;
    float q2q3 = ahrs->q2 * ahrs->q3;
    float q3q3 = ahrs->q3 * ahrs->q3;

    ahrs->rotMat[0][0] = q0q0 + q1q1 - q2q2 - q3q3;
    ahrs->rotMat[0][1] = 2.0f * (q1q2 - q0q3);
    ahrs->rotMat[0][2] = 2.0f * (q1q3 + q0q2);

    ahrs->rotMat[1][0] = 2.0f * (q1q2 + q0q3);
    ahrs->rotMat[1][1] = q0q0 - q1q1 + q2q2 - q3q3;
    ahrs->rotMat[1][2] = 2.0f * (q2q3 - q0q1);

    ahrs->rotMat[2][0] = 2.0f * (q1q3 - q0q2);
    ahrs->rotMat[2][1] = 2.0f * (q2q3 + q0q1);
    ahrs->rotMat[2][2] = q0q0 - q1q1 - q2q2 + q3q3;
}


//Computes Euler angles (roll, pitch, yaw) from rotation matrix and saves in ahrs->roll,pitch,yaw.
static void computeEulerAngles(ahrs_t *ahrs)
{

    float rollRad = atan2f(ahrs->rotMat[2][1], ahrs->rotMat[2][2]);

    float pitchRad = -asinf(ahrs->rotMat[2][0]);

    float yawRad = -atan2f(ahrs->rotMat[1][0], ahrs->rotMat[0][0]);

    //Conversion to degrees
    float rollDeg  = rollRad  * RAD2DEG;
    float pitchDeg = pitchRad * RAD2DEG;
    float yawDeg   = yawRad   * RAD2DEG;

    //Yaw normalization to 0...360deg
    while (yawDeg < 0.0f) {
        yawDeg += 360.0f;
    }
    while (yawDeg >= 360.0f) {
        yawDeg -= 360.0f;
    }

    ahrs->roll  = rollDeg;
    ahrs->pitch = pitchDeg;
    ahrs->yaw   = yawDeg;
}

/**
 * Dynamically sets Kp (dynamicKp) depending on:
 *  - arm state,
 *  - time from boot,
 *  - other factors (quiet drone).
 *
 * W Betaflight ta logika jest rozbudowana (np. 250ms spokoju + 500ms superKp).
 * Tu pokazujemy uproszczenie w stylu "20s x10, potem normalne" + "jeśli stoi nieruchomo".
 */
static void updateDynamicKp(ahrs_t *ahrs, bool armed, float gx, float gy, float gz, float dt)
{
    // Sets default kp:
    float kpNow = ahrs->kp;

    if (!armed && (ahrs->timeSinceStartup < INITIAL_HIGH_KP_DURATION)) {
        kpNow *= HIGH_KP_FACTOR;
    }

    const float spinSq = gx*gx + gy*gy + gz*gz;
    const float smallSpinThreshold = (5.0f * (float)M_PI/180.0f) * (5.0f * (float)M_PI/180.0f);

    if (!armed) {
        if (spinSq < smallSpinThreshold) {
            ahrs->timeWithoutMotion += dt;
        } else {
            ahrs->timeWithoutMotion = 0.0f;
        }

        if (ahrs->timeWithoutMotion > 0.5f) {
            kpNow *= 20.0f;
        }
    } else {
        ahrs->timeWithoutMotion = 0.0f;
    }
    ahrs->dynamicKp = kpNow;
}


void ahrsInit(ahrs_t *ahrs)
{
    // resetting everything
    ahrs->q0 = 1.0f;
    ahrs->q1 = 0.0f;
    ahrs->q2 = 0.0f;
    ahrs->q3 = 0.0f;

    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            ahrs->rotMat[r][c] = (r == c) ? 1.0f : 0.0f;
        }
    }

    ahrs->roll  = 0.0f;
    ahrs->pitch = 0.0f;
    ahrs->yaw   = 0.0f;

    ahrs->exInt = 0.0f;
    ahrs->eyInt = 0.0f;
    ahrs->ezInt = 0.0f;

    // Sets multipliers to default values
    ahrs->kp = DEFAULT_KP;
    ahrs->ki = DEFAULT_KI;
    ahrs->dynamicKp = ahrs->kp;

    // Variables to czeck time without move
    ahrs->timeSinceStartup = 0.0f;
    ahrs->timeWithoutMotion = 0.0f;
    ahrs->wasArmed = false;
    ahrs->justArmed = false;
}

//Main AHRS compute function
void ahrsUpdate(ahrs_t *ahrs,
                float gx, float gy, float gz,
                float ax, float ay, float az,
                bool accHealthy,
                bool armed,
                float dt)
{
    ahrs->timeSinceStartup += dt;

    // 1. Calculate dynamicKp
    updateDynamicKp(ahrs, armed, gx, gy, gz, dt);

    // 2. Compute radiant valocity
    const float spinRateSq = gx*gx + gy*gy + gz*gz;
    if (spinRateSq > (GYRO_SPIN_THRESHOLD * GYRO_SPIN_THRESHOLD)) {
        // If drone spins quickly, reset integral
        ahrs->exInt = 0.0f;
        ahrs->eyInt = 0.0f;
        ahrs->ezInt = 0.0f;
    }

    // 3. Init orientation error variables
    float ex = 0.0f, ey = 0.0f, ez = 0.0f;

    // 4. roll/pith correction if acc data is valid
    if (accHealthy) {
        float accNorm = ax*ax + ay*ay + az*az;
        if (accNorm > FLT_EPSILON) {
            accNorm = invSqrtf(accNorm);
            ax *= accNorm;
            ay *= accNorm;
            az *= accNorm;
        }

        // Read estimate 'ground' vector from rotation matrix
        float vx = ahrs->rotMat[2][0];
        float vy = ahrs->rotMat[2][1];
        float vz = ahrs->rotMat[2][2];

        // compute error
        ex += (ay * vz - az * vy);
        ey += (az * vx - ax * vz);
        ez += (ax * vy - ay * vx);
    }

    // 5. proportionam and integral element
    //    (ex,ey,ez is complete error from acc).
    //    W Betaflight normalnie jest: g += Kp*e + Ki*(integral e)
    const float kp = ahrs->dynamicKp;
    const float ki = ahrs->ki;

    // Update integral element if dorne isn't spinning and acc works.
    if ((spinRateSq < (GYRO_SPIN_THRESHOLD*GYRO_SPIN_THRESHOLD)) && accHealthy && (ki > 0.0f)) {
        ahrs->exInt += ex * ki * dt;
        ahrs->eyInt += ey * ki * dt;
        ahrs->ezInt += ez * ki * dt;
    }

    // correction sum:
    gx += kp * ex + ahrs->exInt;
    gy += kp * ey + ahrs->eyInt;
    gz += kp * ez + ahrs->ezInt;

    // 6. Quaternion integration q dot = 0.5 * q * (0, gx, gy, gz)
    float q0 = ahrs->q0;
    float q1 = ahrs->q1;
    float q2 = ahrs->q2;
    float q3 = ahrs->q3;

    float halfDt = 0.5f * dt;

    // q = q + dq
    ahrs->q0 += halfDt * (-q1 * gx - q2 * gy - q3 * gz);
    ahrs->q1 += halfDt * ( q0 * gx + q2 * gz - q3 * gy);
    ahrs->q2 += halfDt * ( q0 * gy - q1 * gz + q3 * gx);
    ahrs->q3 += halfDt * ( q0 * gz + q1 * gy - q2 * gx);

    // 7. Quaternion normalization
    float normQ = ahrs->q0 * ahrs->q0 + ahrs->q1 * ahrs->q1
                + ahrs->q2 * ahrs->q2 + ahrs->q3 * ahrs->q3;
    if (normQ > FLT_EPSILON) {
        float invN = invSqrtf(normQ);
        ahrs->q0 *= invN;
        ahrs->q1 *= invN;
        ahrs->q2 *= invN;
        ahrs->q3 *= invN;
    }

    // 8. Build rotation matrix from quaternion
    buildRotationMatrix(ahrs);

    // 9. Calculate Euler angles
    computeEulerAngles(ahrs);

    // 10. Save arming state
    if (!ahrs->wasArmed && armed) {
        ahrs->justArmed = true;
    } else {
        ahrs->justArmed = false;
    }
    ahrs->wasArmed = armed;
}
