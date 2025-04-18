// kalman.h
#ifndef KALMAN_H
#define KALMAN_H

typedef struct {
    float Q_angle;
    float Q_bias;
    float R_measure;
    float angle;
    float bias;
    float rate;
    float P[2][2];
} Kalman;

typedef struct {
    float roll;
    float pitch;
    float yaw;
} IMU_Angles;

extern IMU_Angles imu_angles;

extern Kalman kalmanRoll, kalmanPitch;

void updateIMU(float accX, float accY, float accZ, float gyroX, float gyroY, float gyroZ, float dt);
void Kalman_Init(Kalman *kalman);
float Kalman_GetAngle(Kalman *kalman, float newAngle, float newRate, float dt);

#endif // KALMAN_H
