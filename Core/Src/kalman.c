// kalman.c
#include "kalman.h"
#include <math.h>

IMU_Angles imu_angles = {0, 0, 0};

Kalman kalmanRoll, kalmanPitch;

void updateIMU(float accX, float accY, float accZ, float gyroX, float gyroY, float gyroZ, float dt) {
    float norm = sqrt(accX * accX + accY * accY + accZ * accZ);
    float threshold = 0.01f;

    float scale = 1.0f;
    if (fabs(norm - 1.0f) > threshold) {
        scale = 1.0f + (fabs(norm - 1.0f) - threshold) * 100.0f;
    }
    kalmanRoll.R_measure = 0.05f * scale;
    kalmanPitch.R_measure = 0.05f * scale;

    float rollAcc  = atan2(accY, accZ) * 180.0f / M_PI;
    float pitchAcc = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 180.0f / M_PI;

    float roll = Kalman_GetAngle(&kalmanRoll, rollAcc, gyroX, dt);
    float pitch = Kalman_GetAngle(&kalmanPitch, pitchAcc, gyroY, dt);

    imu_angles.roll = roll;
    imu_angles.pitch = pitch;

    imu_angles.yaw += gyroZ * dt;
}

void Kalman_Init(Kalman *kalman) {
    kalman->Q_angle = 0.1f;
    kalman->Q_bias = 0.01f;
    kalman->R_measure = 0.1f;
    kalman->angle = 0.0f;
    kalman->bias = 0.0f;
    kalman->P[0][0] = 1.0f;
    kalman->P[0][1] = 0.0f;
    kalman->P[1][0] = 0.0f;
    kalman->P[1][1] = 1.0f;
}

float Kalman_GetAngle(Kalman *kalman, float newAngle, float newRate, float dt) {
    kalman->rate = newRate - kalman->bias;
    kalman->angle += dt * kalman->rate;

    kalman->P[0][0] += dt * (dt * kalman->P[1][1] - kalman->P[0][1] - kalman->P[1][0] + kalman->Q_angle);
    kalman->P[0][1] -= dt * kalman->P[1][1];
    kalman->P[1][0] -= dt * kalman->P[1][1];
    kalman->P[1][1] += kalman->Q_bias * dt;

    float S = kalman->P[0][0] + kalman->R_measure;
    float K[2];
    K[0] = kalman->P[0][0] / S;
    K[1] = kalman->P[1][0] / S;

    float y = newAngle - kalman->angle;

    kalman->angle += K[0] * y;
    kalman->bias  += K[1] * y;

    float P00_temp = kalman->P[0][0];
    float P01_temp = kalman->P[0][1];

    kalman->P[0][0] -= K[0] * P00_temp;
    kalman->P[0][1] -= K[0] * P01_temp;
    kalman->P[1][0] -= K[1] * P00_temp;
    kalman->P[1][1] -= K[1] * P01_temp;

    return kalman->angle;
}
