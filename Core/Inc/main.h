/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#define LED_SPARE_Pin GPIO_PIN_0
#define LED_SPARE_GPIO_Port GPIOC
#define LED_ERROR_Pin GPIO_PIN_1
#define LED_ERROR_GPIO_Port GPIOC
#define LED_BUSY_Pin GPIO_PIN_2
#define LED_BUSY_GPIO_Port GPIOC
#define LED_READY_Pin GPIO_PIN_3
#define LED_READY_GPIO_Port GPIOC
#define LEFT_Pin GPIO_PIN_1
#define LEFT_GPIO_Port GPIOA
#define RIGHT_Pin GPIO_PIN_2
#define RIGHT_GPIO_Port GPIOA
#define STCK_Pin GPIO_PIN_3
#define STCK_GPIO_Port GPIOA
#define CS_Pin GPIO_PIN_4
#define CS_GPIO_Port GPIOA
#define SW_Pin GPIO_PIN_4
#define SW_GPIO_Port GPIOC
#define STBY_RESET_Pin GPIO_PIN_5
#define STBY_RESET_GPIO_Port GPIOC
#define FLAG_Pin GPIO_PIN_10
#define FLAG_GPIO_Port GPIOB
#define BUSY_Pin GPIO_PIN_11
#define BUSY_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
