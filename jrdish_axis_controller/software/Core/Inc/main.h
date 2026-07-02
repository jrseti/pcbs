/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#define ENC2_A_Pin GPIO_PIN_0
#define ENC2_A_GPIO_Port GPIOC
#define ENC2_B_Pin GPIO_PIN_1
#define ENC2_B_GPIO_Port GPIOC
#define USER_SW_1_Pin GPIO_PIN_2
#define USER_SW_1_GPIO_Port GPIOC
#define ENC2_Z_Pin GPIO_PIN_3
#define ENC2_Z_GPIO_Port GPIOC
#define ENC2_Z_EXTI_IRQn EXTI3_IRQn
#define ENC1_A_Pin GPIO_PIN_0
#define ENC1_A_GPIO_Port GPIOA
#define ENC1_B_Pin GPIO_PIN_1
#define ENC1_B_GPIO_Port GPIOA
#define ENC1_Z_Pin GPIO_PIN_2
#define ENC1_Z_GPIO_Port GPIOA
#define ENC1_Z_EXTI_IRQn EXTI2_IRQn
#define LIMIT1_SW1_Pin GPIO_PIN_3
#define LIMIT1_SW1_GPIO_Port GPIOA
#define LIMIT1_SW2_Pin GPIO_PIN_4
#define LIMIT1_SW2_GPIO_Port GPIOA
#define LIMIT2_SW1_Pin GPIO_PIN_5
#define LIMIT2_SW1_GPIO_Port GPIOA
#define PUL1_Pin GPIO_PIN_6
#define PUL1_GPIO_Port GPIOA
#define LIMIT2_SW2_Pin GPIO_PIN_7
#define LIMIT2_SW2_GPIO_Port GPIOA
#define DIR1_Pin GPIO_PIN_4
#define DIR1_GPIO_Port GPIOC
#define ENABLE1_Pin GPIO_PIN_5
#define ENABLE1_GPIO_Port GPIOC
#define PUL2_Pin GPIO_PIN_0
#define PUL2_GPIO_Port GPIOB
#define DIR2_Pin GPIO_PIN_1
#define DIR2_GPIO_Port GPIOB
#define ENABLE2_Pin GPIO_PIN_2
#define ENABLE2_GPIO_Port GPIOB
#define RS232_RX_Pin GPIO_PIN_10
#define RS232_RX_GPIO_Port GPIOB
#define RS232_TX_Pin GPIO_PIN_11
#define RS232_TX_GPIO_Port GPIOB
#define FLASH_CS_Pin GPIO_PIN_12
#define FLASH_CS_GPIO_Port GPIOB
#define USER_SW_2_Pin GPIO_PIN_8
#define USER_SW_2_GPIO_Port GPIOC
#define USER_SW_3_Pin GPIO_PIN_9
#define USER_SW_3_GPIO_Port GPIOC
#define CONFIG_SW_Pin GPIO_PIN_8
#define CONFIG_SW_GPIO_Port GPIOA
#define ADDR2_Pin GPIO_PIN_9
#define ADDR2_GPIO_Port GPIOA
#define ADDR1_Pin GPIO_PIN_10
#define ADDR1_GPIO_Port GPIOA
#define ADDR0_Pin GPIO_PIN_11
#define ADDR0_GPIO_Port GPIOA
#define STATUS_LED_Pin GPIO_PIN_12
#define STATUS_LED_GPIO_Port GPIOA
#define USER_SW_4_Pin GPIO_PIN_15
#define USER_SW_4_GPIO_Port GPIOA
#define RS485_2_TX_Pin GPIO_PIN_10
#define RS485_2_TX_GPIO_Port GPIOC
#define RS485_2_RX_Pin GPIO_PIN_11
#define RS485_2_RX_GPIO_Port GPIOC
#define RS485_2_DE_Pin GPIO_PIN_2
#define RS485_2_DE_GPIO_Port GPIOD
#define FAN_Pin GPIO_PIN_4
#define FAN_GPIO_Port GPIOB
#define RS485_1_DE_Pin GPIO_PIN_5
#define RS485_1_DE_GPIO_Port GPIOB
#define RS485_1_TX_Pin GPIO_PIN_6
#define RS485_1_TX_GPIO_Port GPIOB
#define RS485_1_RX_Pin GPIO_PIN_7
#define RS485_1_RX_GPIO_Port GPIOB
#define GPS_1PPS_Pin GPIO_PIN_9
#define GPS_1PPS_GPIO_Port GPIOB
#define GPS_1PPS_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
