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
#include "stm32f7xx_hal.h"

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
#define SYS_WKUP4_Pin GPIO_PIN_13
#define SYS_WKUP4_GPIO_Port GPIOC
#define JSR_DEADMAN_Pin GPIO_PIN_0
#define JSR_DEADMAN_GPIO_Port GPIOC
#define JSR_AZIMUTH_Pin GPIO_PIN_1
#define JSR_AZIMUTH_GPIO_Port GPIOC
#define JSR_ELEVATION_Pin GPIO_PIN_2
#define JSR_ELEVATION_GPIO_Port GPIOC
#define JSR_A_Pin GPIO_PIN_3
#define JSR_A_GPIO_Port GPIOC
#define JSR_B_Pin GPIO_PIN_0
#define JSR_B_GPIO_Port GPIOA
#define JSR_TRIG_Pin GPIO_PIN_1
#define JSR_TRIG_GPIO_Port GPIOA
#define PC_S_RXD_Pin GPIO_PIN_2
#define PC_S_RXD_GPIO_Port GPIOA
#define PC_S_TXD_Pin GPIO_PIN_3
#define PC_S_TXD_GPIO_Port GPIOA
#define JSL_DEADMAN_Pin GPIO_PIN_4
#define JSL_DEADMAN_GPIO_Port GPIOA
#define JSL_AZIMUTH_Pin GPIO_PIN_5
#define JSL_AZIMUTH_GPIO_Port GPIOA
#define JSL_ELEVATION_Pin GPIO_PIN_6
#define JSL_ELEVATION_GPIO_Port GPIOA
#define JSL_A_Pin GPIO_PIN_7
#define JSL_A_GPIO_Port GPIOA
#define JSL_B_Pin GPIO_PIN_4
#define JSL_B_GPIO_Port GPIOC
#define JSL_TRIG_Pin GPIO_PIN_5
#define JSL_TRIG_GPIO_Port GPIOC
#define PC_E_RXD_Pin GPIO_PIN_10
#define PC_E_RXD_GPIO_Port GPIOB
#define PC_E_TXD_Pin GPIO_PIN_11
#define PC_E_TXD_GPIO_Port GPIOB
#define OUT_RESV_Pin GPIO_PIN_8
#define OUT_RESV_GPIO_Port GPIOG
#define OUT_TRIGGER_Pin GPIO_PIN_6
#define OUT_TRIGGER_GPIO_Port GPIOC
#define OUT_POWER_Pin GPIO_PIN_7
#define OUT_POWER_GPIO_Port GPIOC
#define OUT_CAMERA_SELECT_Pin GPIO_PIN_8
#define OUT_CAMERA_SELECT_GPIO_Port GPIOC
#define OUT_LRF_ENABLE_Pin GPIO_PIN_9
#define OUT_LRF_ENABLE_GPIO_Port GPIOC
#define LED_BUILTIN_Pin GPIO_PIN_8
#define LED_BUILTIN_GPIO_Port GPIOA
#define VCP_TX_Pin GPIO_PIN_9
#define VCP_TX_GPIO_Port GPIOA
#define VCP_RX_Pin GPIO_PIN_10
#define VCP_RX_GPIO_Port GPIOA
#define V_BATT_FREQ_Pin GPIO_PIN_15
#define V_BATT_FREQ_GPIO_Port GPIOA
#define BMS_RXD_Pin GPIO_PIN_10
#define BMS_RXD_GPIO_Port GPIOC
#define BMS_TXD_Pin GPIO_PIN_11
#define BMS_TXD_GPIO_Port GPIOC
#define IN_LRF_START_Pin GPIO_PIN_0
#define IN_LRF_START_GPIO_Port GPIOD
#define IN_LRF_MAN_UP_Pin GPIO_PIN_1
#define IN_LRF_MAN_UP_GPIO_Port GPIOD
#define IN_TRIGGER_ENABLE_Pin GPIO_PIN_2
#define IN_TRIGGER_ENABLE_GPIO_Port GPIOD
#define IN_FOCUS_FAR_Pin GPIO_PIN_3
#define IN_FOCUS_FAR_GPIO_Port GPIOD
#define IN_TARGET_NEXT_Pin GPIO_PIN_4
#define IN_TARGET_NEXT_GPIO_Port GPIOD
#define IN_FIRE_MODE1_Pin GPIO_PIN_5
#define IN_FIRE_MODE1_GPIO_Port GPIOD
#define IN_SPD1_Pin GPIO_PIN_6
#define IN_SPD1_GPIO_Port GPIOD
#define IN_SPD0_Pin GPIO_PIN_7
#define IN_SPD0_GPIO_Port GPIOD
#define IN_FIRE_MODE0_Pin GPIO_PIN_9
#define IN_FIRE_MODE0_GPIO_Port GPIOG
#define IN_TARGET_PREV_Pin GPIO_PIN_10
#define IN_TARGET_PREV_GPIO_Port GPIOG
#define IN_FOCUS_NEAR_Pin GPIO_PIN_11
#define IN_FOCUS_NEAR_GPIO_Port GPIOG
#define IN_CAMERA_SELECT_Pin GPIO_PIN_12
#define IN_CAMERA_SELECT_GPIO_Port GPIOG
#define IN_LRF_MAN_DOWN_Pin GPIO_PIN_13
#define IN_LRF_MAN_DOWN_GPIO_Port GPIOG
#define IN_LRF_ENABLE_Pin GPIO_PIN_14
#define IN_LRF_ENABLE_GPIO_Port GPIOG
#define IN_RESV_Pin GPIO_PIN_15
#define IN_RESV_GPIO_Port GPIOG
#define SLA_POWER_ENABLE_Pin GPIO_PIN_3
#define SLA_POWER_ENABLE_GPIO_Port GPIOB
#define DUAL_TRIG_IN_Pin GPIO_PIN_4
#define DUAL_TRIG_IN_GPIO_Port GPIOB
#define TRIG_PULSE_OUT_Pin GPIO_PIN_5
#define TRIG_PULSE_OUT_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
