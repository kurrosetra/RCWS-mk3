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
#define LED_BUILTIN_Pin GPIO_PIN_0
#define LED_BUILTIN_GPIO_Port GPIOC
#define XROM_NCS_Pin GPIO_PIN_2
#define XROM_NCS_GPIO_Port GPIOA
#define XROM_CLK_Pin GPIO_PIN_3
#define XROM_CLK_GPIO_Port GPIOA
#define XROM_IO3_Pin GPIO_PIN_6
#define XROM_IO3_GPIO_Port GPIOA
#define XROM_IO2_Pin GPIO_PIN_7
#define XROM_IO2_GPIO_Port GPIOA
#define XROM_IO1_Pin GPIO_PIN_0
#define XROM_IO1_GPIO_Port GPIOB
#define XROM_IO0_Pin GPIO_PIN_1
#define XROM_IO0_GPIO_Port GPIOB
#define PC_TXD_Pin GPIO_PIN_10
#define PC_TXD_GPIO_Port GPIOB
#define PC_RXD_Pin GPIO_PIN_11
#define PC_RXD_GPIO_Port GPIOB
#define VCP_TX_Pin GPIO_PIN_9
#define VCP_TX_GPIO_Port GPIOA
#define VCP_RX_Pin GPIO_PIN_10
#define VCP_RX_GPIO_Port GPIOA
#define BUS_CAN_RX_Pin GPIO_PIN_11
#define BUS_CAN_RX_GPIO_Port GPIOA
#define BUS_CAN_TX_Pin GPIO_PIN_12
#define BUS_CAN_TX_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define TRIGGER_PULSE_OUT_Pin GPIO_PIN_15
#define TRIGGER_PULSE_OUT_GPIO_Port GPIOA
#define BUTTON_RXD_Pin GPIO_PIN_10
#define BUTTON_RXD_GPIO_Port GPIOC
#define BUTTON_TXD_Pin GPIO_PIN_11
#define BUTTON_TXD_GPIO_Port GPIOC
#define JSR_TRIG_Pin GPIO_PIN_4
#define JSR_TRIG_GPIO_Port GPIOB
#define OTHER_CAN_RX_Pin GPIO_PIN_5
#define OTHER_CAN_RX_GPIO_Port GPIOB
#define OTHER_CAN_TX_Pin GPIO_PIN_6
#define OTHER_CAN_TX_GPIO_Port GPIOB
#define BOOT0_Pin GPIO_PIN_8
#define BOOT0_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
