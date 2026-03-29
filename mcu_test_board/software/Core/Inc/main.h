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
#include "stm32g4xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ENC_A_Pin GPIO_PIN_0
#define ENC_A_GPIO_Port GPIOA
#define ENC_B_Pin GPIO_PIN_1
#define ENC_B_GPIO_Port GPIOA
#define ECN_Z_Pin GPIO_PIN_2
#define ECN_Z_GPIO_Port GPIOA
#define ECN_Z_EXTI_IRQn EXTI2_IRQn
#define LIMIT_SW1_Pin GPIO_PIN_3
#define LIMIT_SW1_GPIO_Port GPIOA
#define LIMIT_SW2_Pin GPIO_PIN_4
#define LIMIT_SW2_GPIO_Port GPIOA
#define ENABLE_Pin GPIO_PIN_6
#define ENABLE_GPIO_Port GPIOA
#define DIR_Pin GPIO_PIN_7
#define DIR_GPIO_Port GPIOA
#define PUL_Pin GPIO_PIN_0
#define PUL_GPIO_Port GPIOB
#define ADDR0_Pin GPIO_PIN_9
#define ADDR0_GPIO_Port GPIOA
#define ADDR1_Pin GPIO_PIN_10
#define ADDR1_GPIO_Port GPIOA
#define ADDR2_Pin GPIO_PIN_11
#define ADDR2_GPIO_Port GPIOA
#define STATUS_LED_Pin GPIO_PIN_12
#define STATUS_LED_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
