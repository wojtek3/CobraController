/*
 * config.h
 *
 *  Created on: Apr 1, 2025
 *      Author: wwisz
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

////////////////// Mahony AHRS  /////////////////


#define ICM42688_GYRO_SCALE   (1.0f / 65.536f)
#define ICM42688_ACCEL_SCALE  (1.0f / 2048.0f)

#define ACC_MIN_TRUST 0.99f
#define ACC_MAX_TRUST 1.01f

#define DEFAULT_KP 20.00f
#define DEFAULT_KI 0.00f

#define ACC_MIN 0.95f
#define ACC_MAX 1.05f

////////////////// ICM42688 ///////////////////

#define ICM_ACC_RANGE   ICM42688_ACCEL_RANGE_4G
#define ICM_GYRO_RANGE  ICM42688_GYRO_RANGE_1000DPS

///////////////////// RC //////////////////////

#define NUM_RC_CHANNELS 16
#define NUM_HELPER_CHANNELS 8
#define TOTAL_CHANNELS (NUM_RC_CHANNELS + NUM_HELPER_CHANNELS)


///////////////////// PWM /////////////////////
#define NUM_PWM_OUTPUTS         12


#endif /* INC_CONFIG_H_ */
