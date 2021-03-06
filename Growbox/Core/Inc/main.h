/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f1xx_hal.h"

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
#define GreenLed_Pin GPIO_PIN_5
#define GreenLed_GPIO_Port GPIOA
#define FAN_OUT_Pin GPIO_PIN_10
#define FAN_OUT_GPIO_Port GPIOB
#define AM2301A_Pin GPIO_PIN_8
#define AM2301A_GPIO_Port GPIOA
#define FAN_MIX_Pin GPIO_PIN_9
#define FAN_MIX_GPIO_Port GPIOA
#define DHT11_Pin GPIO_PIN_10
#define DHT11_GPIO_Port GPIOA
#define PUMP_Pin GPIO_PIN_3
#define PUMP_GPIO_Port GPIOB
#define FAN_IN_Pin GPIO_PIN_4
#define FAN_IN_GPIO_Port GPIOB
#define LED_PANEL_Pin GPIO_PIN_5
#define LED_PANEL_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
