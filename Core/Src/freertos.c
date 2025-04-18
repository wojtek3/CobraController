/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "kalman.h"
#include "dps310.h"
#include "icm42688.h"
#include "crsf.h"
#include "tim.h"
#include "pwm.h"
#include "config.h"
#include "ahrs.h"
#include "mixer.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

ahrs_t ahrs;

extern float temperature;
extern float pressure;

float roll, pitch, yaw;

float accel_x_g;
float accel_y_g;
float accel_z_g;

float gyro_x_dps;
float gyro_y_dps;
float gyro_z_dps;

float gyro_x_rad;
float gyro_y_rad;
float gyro_z_rad;

float accMag;


extern ICM42688_Data_t sensorData;
extern ICM42688_Handle_t hImu;
extern DPS310_Handle_t dps310;
extern BatteryData_t batteryData;


/* USER CODE END Variables */
/* Definitions for IMUTask */
osThreadId_t IMUTaskHandle;
const osThreadAttr_t IMUTask_attributes = {
  .name = "IMUTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh7,
};
/* Definitions for BaroTask */
osThreadId_t BaroTaskHandle;
const osThreadAttr_t BaroTask_attributes = {
  .name = "BaroTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for KalmanTask */
osThreadId_t KalmanTaskHandle;
const osThreadAttr_t KalmanTask_attributes = {
  .name = "KalmanTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh7,
};
/* Definitions for CSRFTask */
osThreadId_t CSRFTaskHandle;
const osThreadAttr_t CSRFTask_attributes = {
  .name = "CSRFTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for TelemetryTask */
osThreadId_t TelemetryTaskHandle;
const osThreadAttr_t TelemetryTask_attributes = {
  .name = "TelemetryTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for servoTask */
osThreadId_t servoTaskHandle;
const osThreadAttr_t servoTask_attributes = {
  .name = "servoTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for StatusLedTask */
osThreadId_t StatusLedTaskHandle;
const osThreadAttr_t StatusLedTask_attributes = {
  .name = "StatusLedTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartIMUTask(void *argument);
void StartBaroTask(void *argument);
void StartKalmanTask(void *argument);
void StartCSRFTask(void *argument);
void StartTelemetryTask(void *argument);
void StartServoTask(void *argument);
void StartStatusLedTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of IMUTask */
  IMUTaskHandle = osThreadNew(StartIMUTask, NULL, &IMUTask_attributes);

  /* creation of BaroTask */
  BaroTaskHandle = osThreadNew(StartBaroTask, NULL, &BaroTask_attributes);

  /* creation of KalmanTask */
  KalmanTaskHandle = osThreadNew(StartKalmanTask, NULL, &KalmanTask_attributes);

  /* creation of CSRFTask */
  CSRFTaskHandle = osThreadNew(StartCSRFTask, NULL, &CSRFTask_attributes);

  /* creation of TelemetryTask */
  TelemetryTaskHandle = osThreadNew(StartTelemetryTask, NULL, &TelemetryTask_attributes);

  /* creation of servoTask */
  servoTaskHandle = osThreadNew(StartServoTask, NULL, &servoTask_attributes);

  /* creation of StatusLedTask */
  StatusLedTaskHandle = osThreadNew(StartStatusLedTask, NULL, &StatusLedTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartIMUTask */

/**
  * @brief  Function implementing the IMUTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartIMUTask */
void StartIMUTask(void *argument)
{
  /* USER CODE BEGIN StartIMUTask */
  /* Infinite loop */
	for(;;)
	  {
	  	  if (ICM42688_GetSensorData(&hImu, &sensorData) == HAL_OK)
	  	  {
//	  	  temperature_degC = ((float)sensorData.temperature) / 132.48f + 25.0f;
	  		accel_x_g = (float)sensorData.accel_x * ICM42688_ACCEL_SCALE;
	  		accel_y_g = (float)sensorData.accel_y * ICM42688_ACCEL_SCALE;
	  		accel_z_g = (float)sensorData.accel_z * ICM42688_ACCEL_SCALE;

	  		gyro_x_dps = (float)sensorData.gyro_x * ICM42688_GYRO_SCALE;
	  		gyro_y_dps = (float)sensorData.gyro_y * ICM42688_GYRO_SCALE;
	  		gyro_z_dps = (float)sensorData.gyro_z * ICM42688_GYRO_SCALE;

	  		gyro_x_rad = gyro_x_dps * M_PI / 180.0;
	  		gyro_y_rad = gyro_y_dps * M_PI / 180.0;
	  		gyro_z_rad = gyro_z_dps * M_PI / 180.0;

	  		accMag = sqrtf(accel_x_g * accel_x_g + accel_y_g * accel_y_g + accel_z_g * accel_z_g);
	  	  }
	  	  osDelay(1);
	    }
  /* USER CODE END StartIMUTask */
}

/* USER CODE BEGIN Header_StartBaroTask */
/**
* @brief Function implementing the BaroTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBaroTask */
void StartBaroTask(void *argument)
{
  /* USER CODE BEGIN StartBaroTask */
  /* Infinite loop */
	for(;;)
	  {
	  	  if (DPS310_ReadTempAndPressure(&dps310) == HAL_OK){
	  	  }
	  	  osDelay(100);
	    }
  /* USER CODE END StartBaroTask */
}

/* USER CODE BEGIN Header_StartKalmanTask */
/**
* @brief Function implementing the KalmanTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartKalmanTask */
void StartKalmanTask(void *argument)
{
  /* USER CODE BEGIN StartKalmanTask */
  /* Infinite loop */
		TickType_t xLastWakeTime = xTaskGetTickCount();
		const TickType_t xFrequency = pdMS_TO_TICKS(1);
		ahrsInit(&ahrs);
		for(;;)
		{
			bool accHealthy = (accMag > ACC_MIN && accMag < ACC_MAX);
			bool isArmed = false;
			float dt = 0.001f;

			ahrsUpdate(&ahrs, gyro_x_rad, gyro_y_rad, gyro_z_rad, accel_x_g, accel_y_g, accel_z_g, accHealthy, isArmed, dt);

			vTaskDelayUntil(&xLastWakeTime, xFrequency);
		}
  /* USER CODE END StartKalmanTask */
}

/* USER CODE BEGIN Header_StartCSRFTask */
/**
* @brief Function implementing the CSRFTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCSRFTask */
void StartCSRFTask(void *argument)
{
  /* USER CODE BEGIN StartCSRFTask */
  /* Infinite loop */
	for(;;)
	{
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		processCRSFframe(frameBuf, frameLength);
	}
  /* USER CODE END StartCSRFTask */
}

/* USER CODE BEGIN Header_StartTelemetryTask */
/**
* @brief Function implementing the TelemetryTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTelemetryTask */
void StartTelemetryTask(void *argument)
{
  /* USER CODE BEGIN StartTelemetryTask */
  /* Infinite loop */
  for(;;)
  {
	  batteryData.remaining++;
	  if(batteryData.remaining > 100) batteryData.remaining = 0;
	  CRSF_SendBatteryData(&batteryData);

    osDelay(100);
  }
  /* USER CODE END StartTelemetryTask */
}

/* USER CODE BEGIN Header_StartServoTask */
/**
* @brief Function implementing the servoTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartServoTask */
void StartServoTask(void *argument)
{
  /* USER CODE BEGIN StartServoTask */
	uint8_t pwms[] = {1,2,3,4,5};
	startPWM(pwms, sizeof(pwms) / sizeof(pwms[0]));
  /* Infinite loop */
  for(;;)
  {
//	  passthroughMode(&rcChannels);
	  osDelay(10);

  }
  /* USER CODE END StartServoTask */
}

/* USER CODE BEGIN Header_StartStatusLedTask */
/**
* @brief Function implementing the StatusLedTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartStatusLedTask */
void StartStatusLedTask(void *argument)
{
  /* USER CODE BEGIN StartStatusLedTask */
  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
    osDelay(1000);
  }
  /* USER CODE END StartStatusLedTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

