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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define JSR_TRIG_Pin GPIO_PIN_13
#define JSR_TRIG_GPIO_Port GPIOC
#define JSR_B_Pin GPIO_PIN_14
#define JSR_B_GPIO_Port GPIOC
#define JSR_A_Pin GPIO_PIN_15
#define JSR_A_GPIO_Port GPIOC
#define JSR_ELEVATION_Pin GPIO_PIN_0
#define JSR_ELEVATION_GPIO_Port GPIOA
#define JSR_AZIMUTH_Pin GPIO_PIN_1
#define JSR_AZIMUTH_GPIO_Port GPIOA
#define JSR_DEADMAN_Pin GPIO_PIN_2
#define JSR_DEADMAN_GPIO_Port GPIOA
#define JSL_ELEVATION_Pin GPIO_PIN_3
#define JSL_ELEVATION_GPIO_Port GPIOA
#define JSL_AZIMUTH_Pin GPIO_PIN_4
#define JSL_AZIMUTH_GPIO_Port GPIOA
#define VBAT_SCALED_Pin GPIO_PIN_5
#define VBAT_SCALED_GPIO_Port GPIOA
#define DATA_OUT_Pin GPIO_PIN_6
#define DATA_OUT_GPIO_Port GPIOA
#define DATA_IN_Pin GPIO_PIN_7
#define DATA_IN_GPIO_Port GPIOA
#define LATCH_Pin GPIO_PIN_0
#define LATCH_GPIO_Port GPIOB
#define CLK_Pin GPIO_PIN_1
#define CLK_GPIO_Port GPIOB
#define VCP_TXD_Pin GPIO_PIN_10
#define VCP_TXD_GPIO_Port GPIOB
#define VCP_RXD_Pin GPIO_PIN_11
#define VCP_RXD_GPIO_Port GPIOB
#define LED_BUILTIN_Pin GPIO_PIN_8
#define LED_BUILTIN_GPIO_Port GPIOA
#define JSL_TRIG_Pin GPIO_PIN_15
#define JSL_TRIG_GPIO_Port GPIOA
#define JSL_B_Pin GPIO_PIN_3
#define JSL_B_GPIO_Port GPIOB
#define JSL_A_Pin GPIO_PIN_4
#define JSL_A_GPIO_Port GPIOB
#define JSL_DEADMAN_Pin GPIO_PIN_5
#define JSL_DEADMAN_GPIO_Port GPIOB
#define BUTTON_TXD_Pin GPIO_PIN_6
#define BUTTON_TXD_GPIO_Port GPIOB
#define BUTTON_RXD_Pin GPIO_PIN_7
#define BUTTON_RXD_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
