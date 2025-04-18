/**
  ******************************************************************************
  * @file    dps310.c
  * @brief   DPS310 driver implementation for STM32 (I2C mode)
  ******************************************************************************
  */

#include "dps310.h"
#include <math.h>

static HAL_StatusTypeDef dps310_read_coeffs(DPS310_Handle_t *hdps);
static int32_t dps310_get_raw(I2C_HandleTypeDef *hi2c, uint8_t dev_addr, uint8_t start_reg);
static float dps310_compensate_temperature(DPS310_Handle_t *hdps, int32_t raw_temp);
static float dps310_compensate_pressure(DPS310_Handle_t *hdps, int32_t raw_prs, float T);

HAL_StatusTypeDef DPS310_Init(DPS310_Handle_t *hdps) {
    HAL_StatusTypeDef status;
    uint8_t prod_id = 0;

    if (!hdps || !hdps->hi2c) return HAL_ERROR;

    status = DPS310_SoftReset(hdps);
    if (status != HAL_OK) return status;

    HAL_Delay(10);

    status = HAL_I2C_Mem_Read(hdps->hi2c, hdps->address, DPS310_REG_PRODUCT_ID, I2C_MEMADD_SIZE_8BIT, &prod_id, 1, 100);
    if (status != HAL_OK) return status;

    if (prod_id != 0x10) return HAL_ERROR;

    HAL_Delay(50);

    status = dps310_read_coeffs(hdps);
    if (status != HAL_OK) return status;

    uint8_t tmp = 0x43;
    status = HAL_I2C_Mem_Write(hdps->hi2c, hdps->address, DPS310_REG_PR_CFG, I2C_MEMADD_SIZE_8BIT, &tmp, 1, 100);
    if (status != HAL_OK) return status;

    tmp = 0xC3;
    status = HAL_I2C_Mem_Write(hdps->hi2c, hdps->address, DPS310_REG_TMP_CFG, I2C_MEMADD_SIZE_8BIT, &tmp, 1, 100);
    if (status != HAL_OK) return status;

    tmp = 0x00;
    status = HAL_I2C_Mem_Write(hdps->hi2c, hdps->address, DPS310_REG_CFG_REG, I2C_MEMADD_SIZE_8BIT, &tmp, 1, 100);
    if (status != HAL_OK) return status;

    hdps->temp_scaling = 7864320.0f;
    hdps->prs_scaling = 7864320.0f;

    return DPS310_SetMode(hdps, DPS310_MODE_IDLE);
}

HAL_StatusTypeDef DPS310_SoftReset(DPS310_Handle_t *hdps) {
    uint8_t rst = DPS310_SOFT_RST_CMD;
    return HAL_I2C_Mem_Write(hdps->hi2c, hdps->address, DPS310_REG_RESET, I2C_MEMADD_SIZE_8BIT, &rst, 1, 100);
}

HAL_StatusTypeDef DPS310_SetMode(DPS310_Handle_t *hdps, DPS310_Mode_t mode) {
    HAL_StatusTypeDef status;
    uint8_t meas_cfg = 0;

    status = HAL_I2C_Mem_Read(hdps->hi2c, hdps->address, DPS310_REG_MEAS_CFG, I2C_MEMADD_SIZE_8BIT, &meas_cfg, 1, 100);
    if (status != HAL_OK) return status;

    meas_cfg &= ~DPS310_MEAS_CFG_MEAS_CTRL_MASK;
    meas_cfg |= (mode & 0x07);

    return HAL_I2C_Mem_Write(hdps->hi2c, hdps->address, DPS310_REG_MEAS_CFG, I2C_MEMADD_SIZE_8BIT, &meas_cfg, 1, 100);
}

HAL_StatusTypeDef DPS310_ReadTempAndPressure(DPS310_Handle_t *hdps) {
    int32_t raw_temp = dps310_get_raw(hdps->hi2c, hdps->address, DPS310_REG_TMP_B2);
    if (raw_temp == 0x80000000) return HAL_ERROR;

    int32_t raw_prs = dps310_get_raw(hdps->hi2c, hdps->address, DPS310_REG_PRS_B2);
    if (raw_prs == 0x80000000) return HAL_ERROR;

    hdps->temperature = dps310_compensate_temperature(hdps, raw_temp);
    hdps->pressure = dps310_compensate_pressure(hdps, raw_prs, hdps->temperature);

    return HAL_OK;
}

float DPS310_GetTemperature(DPS310_Handle_t *hdps) {
    return hdps->temperature;
}

float DPS310_GetPressure(DPS310_Handle_t *hdps) {
    return hdps->pressure;
}

static HAL_StatusTypeDef dps310_read_coeffs(DPS310_Handle_t *hdps) {
    uint8_t buffer[18];
    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Read(hdps->hi2c, hdps->address, DPS310_REG_COEF_START, I2C_MEMADD_SIZE_8BIT, buffer, 18, 200);
    if (status != HAL_OK) return status;

    hdps->coeffs.c0 = (int16_t)((((int32_t)buffer[0]) << 4) | (buffer[1] >> 4));
    if (hdps->coeffs.c0 & (1 << 11)) hdps->coeffs.c0 -= 1 << 12;

    hdps->coeffs.c1 = (int16_t)((((int32_t)(buffer[1] & 0x0F)) << 8) | buffer[2]);
    if (hdps->coeffs.c1 & (1 << 11)) hdps->coeffs.c1 -= 1 << 12;

    hdps->coeffs.c00 = (int32_t)((((uint32_t)buffer[3]) << 12) | (((uint32_t)buffer[4]) << 4) | (buffer[5] >> 4));
    if (hdps->coeffs.c00 & (1 << 19)) hdps->coeffs.c00 -= 1 << 20;

    hdps->coeffs.c10 = (int32_t)((((uint32_t)(buffer[5] & 0x0F)) << 16) | (((uint32_t)buffer[6]) << 8) | (buffer[7]));
    if (hdps->coeffs.c10 & (1 << 19)) hdps->coeffs.c10 -= 1 << 20;

    hdps->coeffs.c01 = (int16_t)((((int32_t)buffer[8]) << 8) | buffer[9]);
    hdps->coeffs.c11 = (int16_t)((((int32_t)buffer[10]) << 8) | buffer[11]);
    hdps->coeffs.c20 = (int16_t)((((int32_t)buffer[12]) << 8) | buffer[13]);
    hdps->coeffs.c21 = (int16_t)((((int32_t)buffer[14]) << 8) | buffer[15]);
    hdps->coeffs.c30 = (int16_t)((((int32_t)buffer[16]) << 8) | buffer[17]);

    return HAL_OK;
}

static int32_t dps310_get_raw(I2C_HandleTypeDef *hi2c, uint8_t dev_addr, uint8_t start_reg) {
    uint8_t raw[3];
    if (HAL_I2C_Mem_Read(hi2c, dev_addr, start_reg, I2C_MEMADD_SIZE_8BIT, raw, 3, 100) != HAL_OK)
        return 0x80000000;

    int32_t val = ((int32_t)raw[0] << 16) | ((int32_t)raw[1] << 8) | (raw[2]);
    if (val & 0x800000)
        val -= 1 << 24;

    return val;
}

static float dps310_compensate_temperature(DPS310_Handle_t *hdps, int32_t raw_temp) {
    float Traw_sc = (float)raw_temp / hdps->temp_scaling;
    return ((float)hdps->coeffs.c0 * 0.5f) + ((float)hdps->coeffs.c1 * Traw_sc);
}

static float dps310_compensate_pressure(DPS310_Handle_t *hdps, int32_t raw_prs, float T) {
    float Praw_sc = (float)raw_prs / hdps->prs_scaling;
    float c0_half = (float)hdps->coeffs.c0 * 0.5f;
    float Traw_sc = (hdps->coeffs.c1 != 0) ? ((T - c0_half) / (float)hdps->coeffs.c1) : 0.0f;

    return hdps->coeffs.c00 +
           Praw_sc * (hdps->coeffs.c10 + Praw_sc * (hdps->coeffs.c20 + Praw_sc * hdps->coeffs.c30)) +
           Traw_sc * hdps->coeffs.c01 +
           Traw_sc * Praw_sc * (hdps->coeffs.c11 + Praw_sc * hdps->coeffs.c21);
}

