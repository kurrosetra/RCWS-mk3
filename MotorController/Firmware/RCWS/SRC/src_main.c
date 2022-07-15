/*
 * src_main.c
 *
 *  Created on: Jul 11, 2022
 *      Author: 62812
 */

#include "fdcan.h"

#include <string.h>

#include "driver/uartTerminal/uartTerminal.h"
#include "driver/bus_fdcan/bus_fdcan.h"
#include "driver/ingenia/ingenia.h"
#include "hal/motor/hal_motor.h"

#if DEBUG_ENABLE==1
#include <stdio.h>
/*** Internal Const Values, Macros ***/
#	define LOG(str, ...) printf("[%ld %s:%d] " str, HAL_GetTick(), __FILE_NAME__, __LINE__, ##__VA_ARGS__)
#	define LOG_E(str, ...) printf("[%s Err:%d] " str, __FILE_NAME__, __LINE__, ##__VA_ARGS__)
#else
#	define LOG(str, ...)
#	define LOG_E(str, ...)

#endif	//if DEBUG_ENABLE==1

/* MAIN variables */
static reset_cause_e g_reset_cause;

/* MAIN functions */
void reset_source_init()
{
	g_reset_cause = RESET_CAUSE_UNKNOWN;

	if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST))
		g_reset_cause = RESET_CAUSE_LOW_POWER_RESET;
	else if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST))
		g_reset_cause = RESET_CAUSE_WINDOW_WATCHDOG_RESET;
	else if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST))
		g_reset_cause = RESET_CAUSE_INDEPENDENT_WATCHDOG_RESET;
	else if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST))
		/* This reset is induced by calling the ARM CMSIS */
		/* `NVIC_SystemReset()` function! */
		g_reset_cause = RESET_CAUSE_SOFTWARE_RESET;
	else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST))
		g_reset_cause = RESET_CAUSE_EXTERNAL_RESET_PIN_RESET;
	else if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST))
		g_reset_cause = RESET_CAUSE_BROWNOUT_RESET;

	__HAL_RCC_CLEAR_RESET_FLAGS();
}

uint8_t get_reset_cause()
{
	return g_reset_cause;
}

void main_c_init()
{
	retarget_init();

	LOG("====\r\n");
	LOG("MotorController firmware!\r\n");
	LOG("====\r\n");

	if ((g_reset_cause == RESET_CAUSE_INDEPENDENT_WATCHDOG_RESET) || (g_reset_cause == RESET_CAUSE_SOFTWARE_RESET)) {
		LOG("system is reset from IWDG or s/w reset!\r\n");
	}

	if (g_reset_cause == RESET_CAUSE_EXTERNAL_RESET_PIN_RESET)
		LOG("do you reset this chip?\r\n");

}

/**
 * @brief  Rx FIFO 0 callback.
 * @param  hfdcan pointer to an FDCAN_HandleTypeDef structure that contains
 *         the configuration information for the specified FDCAN.
 * @param  RxFifo0ITs indicates which Rx FIFO 0 interrupts are signalled.
 *         This parameter can be any combination of @arg FDCAN_Rx_Fifo0_Interrupts.
 * @retval None
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	FDCAN_RxHeaderTypeDef RxHeader;
	uint8_t RxData[64];
	BUS_Rx_Buffer_t bus_buffer;
	CAN_Data_t ingenia_buffer;

	if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
		if (hfdcan->Instance == hfdcan2.Instance) {
//			Ingenia_IRQHandler(hfdcan);
			/* Retrieve Rx messages from RX FIFO0 */
			if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
				if (RxHeader.IdType == FDCAN_STANDARD_ID) {
					ingenia_buffer.id = RxHeader.Identifier;
					ingenia_buffer.len = FDCAN_Convert_Datalength(RxHeader.DataLength);
					memcpy(ingenia_buffer.rxData, RxData, ingenia_buffer.len);

					Ingenia_rx_callback(&ingenia_buffer);
				}
			}
		}
		else if (hfdcan->Instance == hfdcan1.Instance) {
			/* Retrieve Rx messages from RX FIFO0 */
			if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
				if (RxHeader.IdType == FDCAN_STANDARD_ID) {
					bus_buffer.lastTimestamp = HAL_GetTick();
					bus_buffer.id = RxHeader.Identifier;
					bus_buffer.len = FDCAN_Convert_Datalength(RxHeader.DataLength);
					memcpy(bus_buffer.data, RxData, bus_buffer.len);
					bus_buffer.counter++;

					bus_rx_callback(&bus_buffer);
				}
			}
		}
	}

}

/**
 * @brief  Rx FIFO 1 callback.
 * @param  hfdcan pointer to an FDCAN_HandleTypeDef structure that contains
 *         the configuration information for the specified FDCAN.
 * @param  RxFifo1ITs indicates which Rx FIFO 1 interrupts are signalled.
 *         This parameter can be any combination of @arg FDCAN_Rx_Fifo1_Interrupts.
 * @retval None
 */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(hfdcan);
	UNUSED(RxFifo1ITs);

	/* NOTE : This function Should not be modified, when the callback is needed,
	 the HAL_FDCAN_RxFifo1Callback could be implemented in the user file
	 */
}

/**
 * @brief  Error status callback.
 * @param  hfdcan pointer to an FDCAN_HandleTypeDef structure that contains
 *         the configuration information for the specified FDCAN.
 * @param  ErrorStatusITs indicates which Error Status interrupts are signaled.
 *         This parameter can be any combination of @arg FDCAN_Error_Status_Interrupts.
 * @retval None
 */
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
	/* Prevent unused argument(s) compilation warning */
	if (hfdcan->Instance == hfdcan1.Instance) {
		MX_FDCAN1_Init();
		bus_init();
		LOG("\r\n===BUS ERROR!===\r\n\r\n");
	}
	else if (hfdcan->Instance == hfdcan2.Instance)
		mtr_error_callback();

	/* NOTE : This function Should not be modified, when the callback is needed,
	 the HAL_FDCAN_ErrorStatusCallback could be implemented in the user file
	 */
}
