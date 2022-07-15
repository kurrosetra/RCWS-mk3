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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RTOS_USE_STACK_HIGH_WATER 0
#define START_ARR_60ms 6000
#define START_ARR_100ms 10000
#define START_ARR_300ms 30000
#define NRST_Pin GPIO_PIN_10
#define NRST_GPIO_Port GPIOG
#define COCK_ENC_A_Pin GPIO_PIN_0
#define COCK_ENC_A_GPIO_Port GPIOA
#define IMU_DE_Pin GPIO_PIN_1
#define IMU_DE_GPIO_Port GPIOA
#define IMU_RX_Pin GPIO_PIN_2
#define IMU_RX_GPIO_Port GPIOA
#define IMU_TX_Pin GPIO_PIN_3
#define IMU_TX_GPIO_Port GPIOA
#define COCK_PWR_RETRACT_Pin GPIO_PIN_4
#define COCK_PWR_RETRACT_GPIO_Port GPIOA
#define COCK_PWR_EN_Pin GPIO_PIN_5
#define COCK_PWR_EN_GPIO_Port GPIOA
#define COCK_PWR_ERECT_Pin GPIO_PIN_6
#define COCK_PWR_ERECT_GPIO_Port GPIOA
#define LIM_AZ_ZERO_Pin GPIO_PIN_7
#define LIM_AZ_ZERO_GPIO_Port GPIOA
#define MUNC_A_Pin GPIO_PIN_11
#define MUNC_A_GPIO_Port GPIOB
#define MUNC_A_EXTI_IRQn EXTI15_10_IRQn
#define MUNC_B_Pin GPIO_PIN_12
#define MUNC_B_GPIO_Port GPIOB
#define MUNC_B_EXTI_IRQn EXTI15_10_IRQn
#define T_JS_PULSE_Pin GPIO_PIN_14
#define T_JS_PULSE_GPIO_Port GPIOB
#define T_START_Pin GPIO_PIN_15
#define T_START_GPIO_Port GPIOB
#define T_HOLD_Pin GPIO_PIN_6
#define T_HOLD_GPIO_Port GPIOC
#define VCP_TX_Pin GPIO_PIN_9
#define VCP_TX_GPIO_Port GPIOA
#define VCP_RX_Pin GPIO_PIN_10
#define VCP_RX_GPIO_Port GPIOA
#define BUS_RX_Pin GPIO_PIN_11
#define BUS_RX_GPIO_Port GPIOA
#define BUS_TX_Pin GPIO_PIN_12
#define BUS_TX_GPIO_Port GPIOA
#define TRIGGER_ENABLE_Pin GPIO_PIN_15
#define TRIGGER_ENABLE_GPIO_Port GPIOA
#define COCK_ENC_B_Pin GPIO_PIN_3
#define COCK_ENC_B_GPIO_Port GPIOB
#define MOTOR_RX_Pin GPIO_PIN_5
#define MOTOR_RX_GPIO_Port GPIOB
#define MOTOR_TX_Pin GPIO_PIN_6
#define MOTOR_TX_GPIO_Port GPIOB
#define BOOT0_Pin GPIO_PIN_8
#define BOOT0_GPIO_Port GPIOB
#define LED_BUILTIN_Pin GPIO_PIN_9
#define LED_BUILTIN_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
