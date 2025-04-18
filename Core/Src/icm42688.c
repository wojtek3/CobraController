#include "icm42688.h"
#include "config.h"

ICM42688_Data_t sensorData;
// ---------------------------------------------------------------------------
//  Internal helper functions
// ---------------------------------------------------------------------------
static void ICM42688_Select(ICM42688_Handle_t *himu)
{
    HAL_GPIO_WritePin(himu->csPort, himu->csPin, GPIO_PIN_RESET);
}

static void ICM42688_Deselect(ICM42688_Handle_t *himu)
{
    HAL_GPIO_WritePin(himu->csPort, himu->csPin, GPIO_PIN_SET);
}

/**
 * @brief  Write to a register over SPI
 * @param  himu  Pointer to ICM42688 handle
 * @param  reg   Register address
 * @param  data  Byte to write
 * @retval HAL status
 */
static HAL_StatusTypeDef ICM42688_WriteReg(ICM42688_Handle_t *himu, uint8_t reg, uint8_t data)
{
    HAL_StatusTypeDef status;
    uint8_t txData[2];

    // For SPI write, bit7=0
    txData[0] = reg & 0x7F;  // Ensure MSB is 0 for write
    txData[1] = data;

    ICM42688_Select(himu);
    status = HAL_SPI_Transmit(himu->hspi, txData, 2, 100);
    ICM42688_Deselect(himu);

    return status;
}

/**
 * @brief  Read a register over SPI
 * @param  himu   Pointer to ICM42688 handle
 * @param  reg    Register address
 * @param  pData  Pointer to variable to store read data
 * @retval HAL status
 */
static HAL_StatusTypeDef ICM42688_ReadReg(ICM42688_Handle_t *himu, uint8_t reg, uint8_t *pData)
{
    HAL_StatusTypeDef status;
    uint8_t txData;
    uint8_t rxData;

    // For SPI read, bit7=1
    txData = reg | 0x80; // Set MSB to 1

    ICM42688_Select(himu);
    status = HAL_SPI_Transmit(himu->hspi, &txData, 1, 100);
    if (status != HAL_OK)
    {
        ICM42688_Deselect(himu);
        return status;
    }
    status = HAL_SPI_Receive(himu->hspi, &rxData, 1, 100);
    ICM42688_Deselect(himu);

    *pData = rxData;
    return status;
}

/**
 * @brief  Read multiple consecutive registers
 * @param  himu   Pointer to ICM42688 handle
 * @param  reg    Start register address
 * @param  pData  Pointer to buffer to store data
 * @param  length Number of bytes to read
 * @retval HAL status
 */
static HAL_StatusTypeDef ICM42688_ReadRegs(ICM42688_Handle_t *himu, uint8_t reg, uint8_t *pData, uint16_t length)
{
    HAL_StatusTypeDef status;
    uint8_t txData;

    txData = reg | 0x80; // MSB=1 for read

    ICM42688_Select(himu);
    status = HAL_SPI_Transmit(himu->hspi, &txData, 1, 100);
    if (status != HAL_OK)
    {
        ICM42688_Deselect(himu);
        return status;
    }

    status = HAL_SPI_Receive(himu->hspi, pData, length, 100);
    ICM42688_Deselect(himu);

    return status;
}

/**
 * @brief  Read-Modify-Write (RMW) helper
 * @param  himu  Pointer to ICM42688 handle
 * @param  reg   Register address
 * @param  mask  Bit mask to clear
 * @param  value Bits to set
 * @retval HAL status
 */
static HAL_StatusTypeDef ICM42688_RMW(ICM42688_Handle_t *himu, uint8_t reg, uint8_t mask, uint8_t value)
{
    HAL_StatusTypeDef status;
    uint8_t temp;

    status = ICM42688_ReadReg(himu, reg, &temp);
    if (status != HAL_OK)
    {
        return status;
    }
    temp &= ~mask;
    temp |= (value & mask);

    return ICM42688_WriteReg(himu, reg, temp);
}

// ---------------------------------------------------------------------------
//  Public functions
// ---------------------------------------------------------------------------

HAL_StatusTypeDef ICM42688_Init(ICM42688_Handle_t *himu)
{
    HAL_StatusTypeDef status;
    uint8_t whoAmI = 0;

    // 1) Check device ID
    status = ICM42688_WhoAmI(himu, &whoAmI);
    if (status != HAL_OK)
    {
        return status;
    }
    if (whoAmI != ICM42688_DEVICE_ID)
    {
        return HAL_ERROR; // Not the correct device
    }

    // 2) Reset the device
    status = ICM42688_WriteReg(himu, ICM42688_REG_DEVICE_CONFIG, 0x01); // Soft reset
    if (status != HAL_OK)
    {
        return status;
    }
    HAL_Delay(50); // Wait for reset

    // 3) Power on accelerometer and gyroscope in Low-Noise mode
    //    PWR_MGMT_0: GYRO_MODE=1, ACCEL_MODE=1
    //    bits: [2:1] = 01 => Accel LN mode, [0] = 1 => Gyro LN mode
    //    That equals 0x0F if we include all bits for LN mode. (Check datasheet for exact bits)
    status = ICM42688_WriteReg(himu, ICM42688_REG_PWR_MGMT_0, 0x0F);
    if (status != HAL_OK)
    {
        return status;
    }
    HAL_Delay(10);

    // 4) Default ranges (accel ±4g, gyro ±500dps)
    status = ICM42688_SetAccelConfig(himu, ICM_ACC_RANGE);
    if (status != HAL_OK) return status;

    status = ICM42688_SetGyroConfig(himu, ICM_GYRO_RANGE);
    if (status != HAL_OK) return status;

    // 5) Default filter settings (example: ODR/4)
    status = ICM42688_SetAccelFilter(himu, ICM42688_ACCEL_BW_ODR_DIV_4);
    if (status != HAL_OK) return status;

    status = ICM42688_SetGyroFilter(himu, ICM42688_GYRO_BW_ODR_DIV_4);
    if (status != HAL_OK) return status;

    // 6) Configure interrupt settings (INT_CONFIG)
    //    Example: push-pull, active high, latched
    status = ICM42688_WriteReg(himu, ICM42688_REG_INT_CONFIG, 0x10);
    if (status != HAL_OK) return status;

    return HAL_OK;
}

HAL_StatusTypeDef ICM42688_WhoAmI(ICM42688_Handle_t *himu, uint8_t *whoAmI)
{
    return ICM42688_ReadReg(himu, ICM42688_REG_WHO_AM_I, whoAmI);
}

HAL_StatusTypeDef ICM42688_GetSensorData(ICM42688_Handle_t *himu, ICM42688_Data_t *data)
{
    HAL_StatusTypeDef status;
    uint8_t buf[14];

    // Read 14 bytes starting from TEMP_DATA1:
    //   TEMP_DATA1, TEMP_DATA0
    //   ACCEL_X1, ACCEL_X0, ACCEL_Y1, ACCEL_Y0, ACCEL_Z1, ACCEL_Z0
    //   GYRO_X1, GYRO_X0, GYRO_Y1, GYRO_Y0, GYRO_Z1, GYRO_Z0
    status = ICM42688_ReadRegs(himu, ICM42688_REG_TEMP_DATA1, buf, 14);
    if (status != HAL_OK)
    {
        return status;
    }

    data->temperature = (int16_t)((buf[0] << 8) | buf[1]);
    data->accel_x     = (int16_t)((buf[2] << 8) | buf[3]);
    data->accel_y     = (int16_t)((buf[4] << 8) | buf[5]);
    data->accel_z     = (int16_t)((buf[6] << 8) | buf[7]);
    data->gyro_x      = (int16_t)((buf[8] << 8) | buf[9]);
    data->gyro_y      = (int16_t)((buf[10] << 8) | buf[11]);
    data->gyro_z      = (int16_t)((buf[12] << 8) | buf[13]);

    return HAL_OK;
}

/**
 * @brief  Set accelerometer full-scale range (and ODR)
 */
HAL_StatusTypeDef ICM42688_SetAccelConfig(ICM42688_Handle_t *himu, ICM42688_AccelRange_t accelRange)
{
    // ACCEL_CONFIG0 [3:2] = FS_SEL, [1:0] = ODR
    // Example: set ODR=0x03 => 1 kHz, FS_SEL=accelRange
    uint8_t regVal = 0;
    regVal |= ((accelRange & 0x03) << 2);
    regVal |= 0x03; // ODR=0x3 => ~1 kHz (example)

    return ICM42688_WriteReg(himu, ICM42688_REG_ACCEL_CONFIG0, regVal);
}

/**
 * @brief  Set gyroscope full-scale range (and ODR)
 */
HAL_StatusTypeDef ICM42688_SetGyroConfig(ICM42688_Handle_t *himu, ICM42688_GyroRange_t gyroRange)
{
    // GYRO_CONFIG0 [3:2] = FS_SEL, [1:0] = ODR
    // Example: set ODR=0x03 => 1 kHz, FS_SEL=gyroRange
    uint8_t regVal = 0;
    regVal |= ((gyroRange & 0x03) << 2);
    regVal |= 0x03; // ODR=0x3 => ~1 kHz (example)

    return ICM42688_WriteReg(himu, ICM42688_REG_GYRO_CONFIG0, regVal);
}

/**
 * @brief  Configure accelerometer digital filter
 *         Typically ACCEL_CONFIG1 [2:0] = ACCEL_UI_FILT_BW
 */
HAL_StatusTypeDef ICM42688_SetAccelFilter(ICM42688_Handle_t *himu, ICM42688_AccelBw_t accelBw)
{
    // ACCEL_CONFIG1 [2:0] => accelBw
    // We'll read-modify-write, since other bits in ACCEL_CONFIG1
    // (e.g. filter mode, averaging) might also exist
    return ICM42688_RMW(himu,
                        ICM42688_REG_ACCEL_CONFIG1,
                        0x07,                   // mask for bits [2:0]
                        (uint8_t)accelBw & 0x07 // new value
                       );
}

/**
 * @brief  Configure gyroscope digital filter
 *         Typically GYRO_CONFIG1 [2:0] = GYRO_UI_FILT_BW
 */
HAL_StatusTypeDef ICM42688_SetGyroFilter(ICM42688_Handle_t *himu, ICM42688_GyroBw_t gyroBw)
{
    // GYRO_CONFIG1 [2:0] => gyroBw
    return ICM42688_RMW(himu,
                        ICM42688_REG_GYRO_CONFIG1,
                        0x07,                  // mask for bits [2:0]
                        (uint8_t)gyroBw & 0x07 // new value
                       );
}

void calcRads_g(ICM42688_Data_t sensorData, ICM42688_Solution_t *solution){
	solution->x_g = sensorData.accel_x * ICM42688_ACCEL_SCALE;
	solution->y_g = sensorData.accel_y * ICM42688_ACCEL_SCALE;
	solution->z_g = sensorData.accel_z * ICM42688_ACCEL_SCALE;

	const float dps2rad = (float)M_PI / 180.0f;

	solution->x_rad = sensorData.gyro_x * ICM42688_GYRO_SCALE * dps2rad;
	solution->y_rad = sensorData.gyro_y * ICM42688_GYRO_SCALE * dps2rad;
	solution->z_rad = sensorData.gyro_z * ICM42688_GYRO_SCALE * dps2rad;
}
