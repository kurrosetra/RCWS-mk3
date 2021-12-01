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
typedef union
{
	float f;
	uint32_t u32;
	int32_t i32;
	uint8_t u8[4];
	int8_t i8[4];
	uint16_t u16[2];
	int16_t i16[2];
} Union_u;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define MTR_DEBUG_ENABLE			1
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BUS_NOMINAL_BAUDRATE_KHZ 500
#define BUS_DATA_BAUDRATE_KHZ 2000
#define MTR_FDCAN_RXD_Pin GPIO_PIN_12
#define MTR_FDCAN_RXD_GPIO_Port GPIOB
#define MTR_FDCAN_TXD_Pin GPIO_PIN_13
#define MTR_FDCAN_TXD_GPIO_Port GPIOB
#define COUNTER_PULSE_Pin GPIO_PIN_14
#define COUNTER_PULSE_GPIO_Port GPIOB
#define COCK_EN_Pin GPIO_PIN_15
#define COCK_EN_GPIO_Port GPIOB
#define COCT_ERECT_Pin GPIO_PIN_6
#define COCT_ERECT_GPIO_Port GPIOC
#define COCK_RETRACT_Pin GPIO_PIN_7
#define COCK_RETRACT_GPIO_Port GPIOC
#define LIM_EL_DOWN_Pin GPIO_PIN_8
#define LIM_EL_DOWN_GPIO_Port GPIOC
#define LIM_EL_UP_Pin GPIO_PIN_9
#define LIM_EL_UP_GPIO_Port GPIOC
#define TRIGGER_ENABLE_Pin GPIO_PIN_8
#define TRIGGER_ENABLE_GPIO_Port GPIOA
#define VCP_TX_Pin GPIO_PIN_9
#define VCP_TX_GPIO_Port GPIOA
#define VCP_TXA10_Pin GPIO_PIN_10
#define VCP_TXA10_GPIO_Port GPIOA
#define BUS_FDCAN_RXD_Pin GPIO_PIN_11
#define BUS_FDCAN_RXD_GPIO_Port GPIOA
#define BUS_FDCAN_RXDA12_Pin GPIO_PIN_12
#define BUS_FDCAN_RXDA12_GPIO_Port GPIOA
#define COCK_MIN_Pin GPIO_PIN_15
#define COCK_MIN_GPIO_Port GPIOA
#define COCK_MAX_Pin GPIO_PIN_10
#define COCK_MAX_GPIO_Port GPIOC
#define LED_BUILTIN_Pin GPIO_PIN_2
#define LED_BUILTIN_GPIO_Port GPIOD
#define LIM_AZ_ZERO_Pin GPIO_PIN_3
#define LIM_AZ_ZERO_GPIO_Port GPIOB
#define EL_ENC_B_Pin GPIO_PIN_4
#define EL_ENC_B_GPIO_Port GPIOB
#define EL_ENC_A_Pin GPIO_PIN_5
#define EL_ENC_A_GPIO_Port GPIOB
#define AZ_ENC_B_Pin GPIO_PIN_6
#define AZ_ENC_B_GPIO_Port GPIOB
#define AZ_ENC_A_Pin GPIO_PIN_7
#define AZ_ENC_A_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
