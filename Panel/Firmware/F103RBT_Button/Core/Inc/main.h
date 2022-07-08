/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#define OUT_POWER_LED_Pin GPIO_PIN_13
#define OUT_POWER_LED_GPIO_Port GPIOC
#define OUT_TRIG_LED_Pin GPIO_PIN_14
#define OUT_TRIG_LED_GPIO_Port GPIOC
#define LED_BUILTIN_Pin GPIO_PIN_15
#define LED_BUILTIN_GPIO_Port GPIOC
#define IN_SPD0_Pin GPIO_PIN_0
#define IN_SPD0_GPIO_Port GPIOA
#define IN_SPD1_Pin GPIO_PIN_1
#define IN_SPD1_GPIO_Port GPIOA
#define IN_FIRING0_Pin GPIO_PIN_2
#define IN_FIRING0_GPIO_Port GPIOA
#define IN_FIRING1_Pin GPIO_PIN_3
#define IN_FIRING1_GPIO_Port GPIOA
#define IN_TARGET_PREV_Pin GPIO_PIN_4
#define IN_TARGET_PREV_GPIO_Port GPIOA
#define IN_TARGET_NEXT_Pin GPIO_PIN_5
#define IN_TARGET_NEXT_GPIO_Port GPIOA
#define IN_FOCUS_IN_Pin GPIO_PIN_6
#define IN_FOCUS_IN_GPIO_Port GPIOA
#define IN_FOCUS_OUT_Pin GPIO_PIN_7
#define IN_FOCUS_OUT_GPIO_Port GPIOA
#define IN_LRF_START_Pin GPIO_PIN_2
#define IN_LRF_START_GPIO_Port GPIOB
#define IN_LRF_UP_Pin GPIO_PIN_10
#define IN_LRF_UP_GPIO_Port GPIOB
#define IN_LRF_DOWN_Pin GPIO_PIN_11
#define IN_LRF_DOWN_GPIO_Port GPIOB
#define IN_CAM_SELECT_Pin GPIO_PIN_12
#define IN_CAM_SELECT_GPIO_Port GPIOB
#define IN_LRF_EN_Pin GPIO_PIN_13
#define IN_LRF_EN_GPIO_Port GPIOB
#define OUT_LRF_EN_Pin GPIO_PIN_9
#define OUT_LRF_EN_GPIO_Port GPIOC
#define OUT_CAM_SELECT_Pin GPIO_PIN_8
#define OUT_CAM_SELECT_GPIO_Port GPIOA
#define VCP_TX_Pin GPIO_PIN_9
#define VCP_TX_GPIO_Port GPIOA
#define VCP_RX_Pin GPIO_PIN_10
#define VCP_RX_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define IN_TRIG_Pin GPIO_PIN_15
#define IN_TRIG_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
