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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TIM4_ON 1000
#define LED_CH TIM_CHANNEL_1
#define TRIG_CH TIM_CHANNEL_2
#define TIM4_OFF 0
#define SLA_POWER_EN_Pin GPIO_PIN_14
#define SLA_POWER_EN_GPIO_Port GPIOC
#define JSL_AZIMUTH_Pin GPIO_PIN_0
#define JSL_AZIMUTH_GPIO_Port GPIOC
#define JSL_ELEVATION_Pin GPIO_PIN_1
#define JSL_ELEVATION_GPIO_Port GPIOC
#define V_BATT_SCALED_Pin GPIO_PIN_1
#define V_BATT_SCALED_GPIO_Port GPIOA
#define XROM_NCS_Pin GPIO_PIN_2
#define XROM_NCS_GPIO_Port GPIOA
#define XROM_CLK_Pin GPIO_PIN_3
#define XROM_CLK_GPIO_Port GPIOA
#define XROM_IO3_Pin GPIO_PIN_6
#define XROM_IO3_GPIO_Port GPIOA
#define XROM_IO2_Pin GPIO_PIN_7
#define XROM_IO2_GPIO_Port GPIOA
#define JSR_AZIMUTH_Pin GPIO_PIN_4
#define JSR_AZIMUTH_GPIO_Port GPIOC
#define JSR_ELEVATION_Pin GPIO_PIN_5
#define JSR_ELEVATION_GPIO_Port GPIOC
#define XROM_IO1_Pin GPIO_PIN_0
#define XROM_IO1_GPIO_Port GPIOB
#define XROM_IO0_Pin GPIO_PIN_1
#define XROM_IO0_GPIO_Port GPIOB
#define PC_TXD_Pin GPIO_PIN_10
#define PC_TXD_GPIO_Port GPIOB
#define PC_RXD_Pin GPIO_PIN_11
#define PC_RXD_GPIO_Port GPIOB
#define OTHER_CAN_RX_Pin GPIO_PIN_12
#define OTHER_CAN_RX_GPIO_Port GPIOB
#define OTHER_CAN_TX_Pin GPIO_PIN_13
#define OTHER_CAN_TX_GPIO_Port GPIOB
#define JSR_DEADMAN_Pin GPIO_PIN_14
#define JSR_DEADMAN_GPIO_Port GPIOB
#define JSR_A_Pin GPIO_PIN_15
#define JSR_A_GPIO_Port GPIOB
#define JSR_B_Pin GPIO_PIN_6
#define JSR_B_GPIO_Port GPIOC
#define JSR_TRIG_Pin GPIO_PIN_7
#define JSR_TRIG_GPIO_Port GPIOC
#define JSL_DEADMAN_Pin GPIO_PIN_8
#define JSL_DEADMAN_GPIO_Port GPIOC
#define JSL_A_Pin GPIO_PIN_9
#define JSL_A_GPIO_Port GPIOC
#define JSL_B_Pin GPIO_PIN_8
#define JSL_B_GPIO_Port GPIOA
#define VCP_TX_Pin GPIO_PIN_9
#define VCP_TX_GPIO_Port GPIOA
#define VCP_RX_Pin GPIO_PIN_10
#define VCP_RX_GPIO_Port GPIOA
#define BUS_FDCAN_RX_Pin GPIO_PIN_11
#define BUS_FDCAN_RX_GPIO_Port GPIOA
#define BUS_FDCAN_TX_Pin GPIO_PIN_12
#define BUS_FDCAN_TX_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define JSL_TRIG_Pin GPIO_PIN_15
#define JSL_TRIG_GPIO_Port GPIOA
#define BUTTON_RXD_Pin GPIO_PIN_3
#define BUTTON_RXD_GPIO_Port GPIOB
#define BUTTON_TXD_Pin GPIO_PIN_4
#define BUTTON_TXD_GPIO_Port GPIOB
#define LED_BUILTIN_Pin GPIO_PIN_6
#define LED_BUILTIN_GPIO_Port GPIOB
#define TRIG_PULSE_OUT_Pin GPIO_PIN_7
#define TRIG_PULSE_OUT_GPIO_Port GPIOB
#define BOOT0_Pin GPIO_PIN_8
#define BOOT0_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define BUTTON_DEBUG		0
#define BUS_MOTOR_DEBUG		0
#if BUS_MOTOR_DEBUG==0
#define BUS_DEBUG			1
#endif	//if BUS_MOTOR_DEBUG==0
#define PC_DEBUG			1
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
