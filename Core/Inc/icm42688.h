#ifndef ICM42688_H
#define ICM42688_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stddef.h>

/**
 * @brief  ICM42688 register definitions
 *         Please refer to the ICM-42688 datasheet for the complete register map.
 */
#define ICM42688_REG_WHO_AM_I            0x75U
#define ICM42688_REG_DEVICE_CONFIG       0x11U
#define ICM42688_REG_INT_CONFIG          0x14U
#define ICM42688_REG_PWR_MGMT_0          0x4EU
#define ICM42688_REG_ACCEL_CONFIG0       0x50U
#define ICM42688_REG_GYRO_CONFIG0        0x4FU
#define ICM42688_REG_ACCEL_CONFIG1       0x53U
#define ICM42688_REG_GYRO_CONFIG1        0x51U
#define ICM42688_REG_TEMP_DATA1          0x1DU
#define ICM42688_REG_TEMP_DATA0          0x1EU
#define ICM42688_REG_ACCEL_DATA_X1       0x1FU
#define ICM42688_REG_ACCEL_DATA_X0       0x20U
#define ICM42688_REG_ACCEL_DATA_Y1       0x21U
#define ICM42688_REG_ACCEL_DATA_Y0       0x22U
#define ICM42688_REG_ACCEL_DATA_Z1       0x23U
#define ICM42688_REG_ACCEL_DATA_Z0       0x24U
#define ICM42688_REG_GYRO_DATA_X1        0x25U
#define ICM42688_REG_GYRO_DATA_X0        0x26U
#define ICM42688_REG_GYRO_DATA_Y1        0x27U
#define ICM42688_REG_GYRO_DATA_Y0        0x28U
#define ICM42688_REG_GYRO_DATA_Z1        0x29U
#define ICM42688_REG_GYRO_DATA_Z0        0x2AU

#define ICM42688_DEVICE_ID               0x47U

/**
 * @brief  Full-scale range values. Adjust as per your needs.
 *         FSR (Full-Scale Range) for Accelerometer and Gyroscope might differ.
 */
typedef enum
{
    ICM42688_ACCEL_RANGE_2G  = 0x00,  /*!< ±2g  */
    ICM42688_ACCEL_RANGE_4G  = 0x01,  /*!< ±4g  */
    ICM42688_ACCEL_RANGE_8G  = 0x02,  /*!< ±8g  */
    ICM42688_ACCEL_RANGE_16G = 0x03,  /*!< ±16g */
} ICM42688_AccelRange_t;

typedef enum
{
    ICM42688_GYRO_RANGE_250DPS  = 0x00, /*!< ±250 dps  */
    ICM42688_GYRO_RANGE_500DPS  = 0x01, /*!< ±500 dps  */
    ICM42688_GYRO_RANGE_1000DPS = 0x02, /*!< ±1000 dps */
    ICM42688_GYRO_RANGE_2000DPS = 0x03, /*!< ±2000 dps */
} ICM42688_GyroRange_t;

typedef enum
{
    ICM42688_ACCEL_BW_ODR_DIV_2   = 0x00,  /*!< ~ODR/2   */
    ICM42688_ACCEL_BW_ODR_DIV_4   = 0x01,  /*!< ~ODR/4   */
    ICM42688_ACCEL_BW_ODR_DIV_5   = 0x02,  /*!< ~ODR/5   */
    ICM42688_ACCEL_BW_ODR_DIV_8   = 0x03,  /*!< ~ODR/8   */
    ICM42688_ACCEL_BW_ODR_DIV_10  = 0x04,  /*!< ~ODR/10  */
    ICM42688_ACCEL_BW_ODR_DIV_16  = 0x05,  /*!< ~ODR/16  */
    ICM42688_ACCEL_BW_ODR_DIV_32  = 0x06,  /*!< ~ODR/32  */
    ICM42688_ACCEL_BW_ODR_DIV_64  = 0x07,  /*!< ~ODR/64  */
} ICM42688_AccelBw_t;

typedef enum
{
    ICM42688_GYRO_BW_ODR_DIV_2    = 0x00,  /*!< ~ODR/2   */
    ICM42688_GYRO_BW_ODR_DIV_4    = 0x01,  /*!< ~ODR/4   */
    ICM42688_GYRO_BW_ODR_DIV_5    = 0x02,  /*!< ~ODR/5   */
    ICM42688_GYRO_BW_ODR_DIV_8    = 0x03,  /*!< ~ODR/8   */
    ICM42688_GYRO_BW_ODR_DIV_10   = 0x04,  /*!< ~ODR/10  */
    ICM42688_GYRO_BW_ODR_DIV_16   = 0x05,  /*!< ~ODR/16  */
    ICM42688_GYRO_BW_ODR_DIV_32   = 0x06,  /*!< ~ODR/32  */
    ICM42688_GYRO_BW_ODR_DIV_64   = 0x07,  /*!< ~ODR/64  */
} ICM42688_GyroBw_t;

/**
 * @brief  Structure to hold raw sensor data
 */
typedef struct
{
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;

    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;

    int16_t temperature;
} ICM42688_Data_t;

typedef struct
{
    float x_g, y_g, z_g;
    float x_rad, y_rad, z_rad;
    int16_t temperature;
} ICM42688_Solution_t;

/**
 * @brief  Driver handle structure
 */
typedef struct
{
    SPI_HandleTypeDef *hspi;  /*!< SPI handle from STM32 HAL */
    GPIO_TypeDef      *csPort;/*!< CS GPIO Port */
    uint16_t          csPin;  /*!< CS GPIO Pin */
} ICM42688_Handle_t;


/**
 * @brief  Function prototypes
 */

/**
 * @brief  Initialize the ICM42688 sensor (SPI mode).
 * @param  himu Pointer to ICM42688_Handle_t structure
 * @retval HAL status
 */
HAL_StatusTypeDef ICM42688_Init(ICM42688_Handle_t *himu);

/**
 * @brief  Read the WHO_AM_I register and validate the device ID.
 * @param  himu Pointer to ICM42688_Handle_t structure
 * @param  whoAmI Pointer to store the read WHO_AM_I value
 * @retval HAL status
 */
HAL_StatusTypeDef ICM42688_WhoAmI(ICM42688_Handle_t *himu, uint8_t *whoAmI);

/**
 * @brief  Get raw sensor data for accelerometer, gyroscope, and temperature.
 * @param  himu Pointer to ICM42688_Handle_t structure
 * @param  data Pointer to ICM42688_Data_t structure to store the data
 * @retval HAL status
 */
HAL_StatusTypeDef ICM42688_GetSensorData(ICM42688_Handle_t *himu, ICM42688_Data_t *data);

/**
 * @brief  Set accelerometer full-scale range and data rate
 * @param  himu         Pointer to ICM42688_Handle_t structure
 * @param  accelRange   Full-scale range for accelerometer
 * @retval HAL status
 */
HAL_StatusTypeDef ICM42688_SetAccelConfig(ICM42688_Handle_t *himu, ICM42688_AccelRange_t accelRange);

/**
 * @brief  Set gyroscope full-scale range and data rate
 * @param  himu         Pointer to ICM42688_Handle_t structure
 * @param  gyroRange    Full-scale range for gyroscope
 * @retval HAL status
 */
HAL_StatusTypeDef ICM42688_SetGyroConfig(ICM42688_Handle_t *himu, ICM42688_GyroRange_t gyroRange);

/**
 * @brief  Configure the built-in digital filter for accelerometer
 * @param  himu      Pointer to ICM42688 handle
 * @param  accelBw   Desired accelerometer filter bandwidth
 * @retval HAL status
 */
HAL_StatusTypeDef ICM42688_SetAccelFilter(ICM42688_Handle_t *himu, ICM42688_AccelBw_t accelBw);

/**
 * @brief  Configure the built-in digital filter for gyroscope
 * @param  himu      Pointer to ICM42688 handle
 * @param  gyroBw    Desired gyroscope filter bandwidth
 * @retval HAL status
 */
HAL_StatusTypeDef ICM42688_SetGyroFilter(ICM42688_Handle_t *himu, ICM42688_GyroBw_t gyroBw);

void calcRads_g(ICM42688_Data_t sensorData, ICM42688_Solution_t *solution);

#ifdef __cplusplus
}
#endif

#endif // ICM42688_H
