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
#define KEY2_Pin GPIO_PIN_13
#define KEY2_GPIO_Port GPIOC
#define AP_EN_Pin GPIO_PIN_8
#define AP_EN_GPIO_Port GPIOF
#define BEEP_Pin GPIO_PIN_0
#define BEEP_GPIO_Port GPIOC
#define RS485_EN_SEND_Pin GPIO_PIN_2
#define RS485_EN_SEND_GPIO_Port GPIOC
#define KEY1_Pin GPIO_PIN_0
#define KEY1_GPIO_Port GPIOA
#define SPI_FALSH_CS_Pin GPIO_PIN_4
#define SPI_FALSH_CS_GPIO_Port GPIOA
#define LED_GREEN_Pin GPIO_PIN_0
#define LED_GREEN_GPIO_Port GPIOB
#define LED_BLUE_Pin GPIO_PIN_1
#define LED_BLUE_GPIO_Port GPIOB
#define DETECT_SDIO_Pin GPIO_PIN_6
#define DETECT_SDIO_GPIO_Port GPIOD
#define W5500_CS_Pin GPIO_PIN_9
#define W5500_CS_GPIO_Port GPIOG
#define LED_RED_Pin GPIO_PIN_5
#define LED_RED_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define LED_RED_ON          HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET)
#define LED_RED_OFF         HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET)
#define LED_RED_TOGGLE      HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin)

#define LED_GREEN_ON        HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET)
#define LED_GREEN_OFF       HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET)
#define LED_GREEN_TOGGLE    HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin)

#define LED_BLUE_ON         HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET)
#define LED_BLUE_OFF        HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_SET)
#define LED_BLUE_TOGGLE     HAL_GPIO_TogglePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin)

#define BEEP_ON             HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET)
#define BEEP_OFF            HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET)

#define RS485_EN_SEND_ON HAL_GPIO_WritePin(RS485_EN_SEND_GPIO_Port, RS485_EN_SEND_Pin, GPIO_PIN_SET)
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
