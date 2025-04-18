/**
  ******************************************************************************
  * @file    dps310.h
  * @author
  * @brief   DPS310 barometric pressure/temperature sensor driver (I2C mode)
  ******************************************************************************
  */

#ifndef __DPS310_H
#define __DPS310_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include <stdint.h>

#define DPS310_I2C_ADDR (0x76 << 1)

/* DPS310 Registers */
#define DPS310_REG_PRS_B2 0x00
#define DPS310_REG_TMP_B2 0x03
#define DPS310_REG_PR_CFG 0x06
#define DPS310_REG_TMP_CFG 0x07
#define DPS310_REG_MEAS_CFG 0x08
#define DPS310_REG_CFG_REG 0x09
#define DPS310_REG_INT_STS 0x0A
#define DPS310_REG_FIFO_STS 0x0B
#define DPS310_REG_RESET 0x0C
#define DPS310_REG_PRODUCT_ID 0x0D
#define DPS310_REG_REV_ID 0x0E
#define DPS310_REG_COEF_START 0x10

#define DPS310_SOFT_RST_CMD 0x89

#define DPS310_MEAS_CFG_TEMP_RDY_MASK (1 << 4)
#define DPS310_MEAS_CFG_PRS_RDY_MASK (1 << 5)
#define DPS310_MEAS_CFG_SENSOR_RDY_MASK (1 << 6)
#define DPS310_MEAS_CFG_MEAS_CTRL_MASK (7 << 0)

typedef enum {
    DPS310_MODE_IDLE = 0x00,
    DPS310_MODE_COMMAND_T = 0x01,
    DPS310_MODE_COMMAND_P = 0x02,
    DPS310_MODE_BACKGROUND_ALL = 0x07,
} DPS310_Mode_t;

typedef struct {
    int16_t c0;
    int16_t c1;
    int32_t c00;
    int32_t c10;
    int16_t c01;
    int16_t c11;
    int16_t c20;
    int16_t c21;
    int16_t c30;
} DPS310_Coeffs_t;

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t address;
    DPS310_Coeffs_t coeffs;
    float temp_scaling;
    float prs_scaling;
    float temperature;
    float pressure;
} DPS310_Handle_t;

HAL_StatusTypeDef DPS310_Init(DPS310_Handle_t *hdps);
HAL_StatusTypeDef DPS310_SoftReset(DPS310_Handle_t *hdps);
HAL_StatusTypeDef DPS310_SetMode(DPS310_Handle_t *hdps, DPS310_Mode_t mode);
HAL_StatusTypeDef DPS310_ReadTempAndPressure(DPS310_Handle_t *hdps);
float DPS310_GetTemperature(DPS310_Handle_t *hdps);
float DPS310_GetPressure(DPS310_Handle_t *hdps);

#ifdef __cplusplus
}
#endif

#endif /* __DPS310_H */


