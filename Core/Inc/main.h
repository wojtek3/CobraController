/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED0_Pin GPIO_PIN_3
#define LED0_GPIO_Port GPIOE
#define LED1_Pin GPIO_PIN_4
#define LED1_GPIO_Port GPIOE
#define PWM11_Pin GPIO_PIN_5
#define PWM11_GPIO_Port GPIOE
#define PWM12_Pin GPIO_PIN_6
#define PWM12_GPIO_Port GPIOE
#define PWM3_Pin GPIO_PIN_0
#define PWM3_GPIO_Port GPIOA
#define PWM4_Pin GPIO_PIN_1
#define PWM4_GPIO_Port GPIOA
#define PWM5_Pin GPIO_PIN_2
#define PWM5_GPIO_Port GPIOA
#define PWM6_Pin GPIO_PIN_3
#define PWM6_GPIO_Port GPIOA
#define PWM1_Pin GPIO_PIN_0
#define PWM1_GPIO_Port GPIOB
#define PWM2_Pin GPIO_PIN_1
#define PWM2_GPIO_Port GPIOB
#define IMU_CS_Pin GPIO_PIN_11
#define IMU_CS_GPIO_Port GPIOE
#define PWM7_Pin GPIO_PIN_12
#define PWM7_GPIO_Port GPIOD
#define PWM8_Pin GPIO_PIN_13
#define PWM8_GPIO_Port GPIOD
#define PWM9_Pin GPIO_PIN_14
#define PWM9_GPIO_Port GPIOD
#define PWM10_Pin GPIO_PIN_15
#define PWM10_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
