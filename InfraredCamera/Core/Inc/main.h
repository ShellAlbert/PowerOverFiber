/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32l4xx_hal.h"

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
#define CAM1_PWDN_Pin GPIO_PIN_6
#define CAM1_PWDN_GPIO_Port GPIOF
#define LED1_Pin GPIO_PIN_8
#define LED1_GPIO_Port GPIOF
#define LED2_Pin GPIO_PIN_9
#define LED2_GPIO_Port GPIOF
#define CAM1_RST_Pin GPIO_PIN_1
#define CAM1_RST_GPIO_Port GPIOC
#define LD_PWR_EN_Pin GPIO_PIN_5
#define LD_PWR_EN_GPIO_Port GPIOA
#define SYNC_SWITCH_Pin GPIO_PIN_1
#define SYNC_SWITCH_GPIO_Port GPIOB
#define DAY_NIGHT_Pin GPIO_PIN_8
#define DAY_NIGHT_GPIO_Port GPIOA
#define VER1_Pin GPIO_PIN_15
#define VER1_GPIO_Port GPIOA
#define CAM2_PWR_EN_Pin GPIO_PIN_9
#define CAM2_PWR_EN_GPIO_Port GPIOG
#define CAM1_PWR_EN_Pin GPIO_PIN_10
#define CAM1_PWR_EN_GPIO_Port GPIOG
#define BUS_SWITCH_Pin GPIO_PIN_5
#define BUS_SWITCH_GPIO_Port GPIOB
#define VER2_Pin GPIO_PIN_8
#define VER2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
extern int gTimer1OverflowFlag;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
