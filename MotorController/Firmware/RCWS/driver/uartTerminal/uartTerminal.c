/*
 * uartTerminal.c
 *
 *  Created on: Dec 29, 2021
 *      Author: miftakur
 */

#include "uartTerminal.h"

/*** Internal Const Values, Macros ***/
#if RETARGET_USE_RX_DMA==1
#	define BUFFER_SIZE 16
#	define bufferRxWp ( (BUFFER_SIZE - sp_huart->hdmarx->Instance->CNDTR) & (BUFFER_SIZE - 1) )
#endif	//if RETARGET_USE_RX_DMA==1

/*** Static Variables ***/
static UART_HandleTypeDef *sp_huart;
#if RETARGET_USE_RX_DMA==1
static volatile uint8_t s_bufferRx[BUFFER_SIZE];
static volatile uint8_t s_bufferRxRp = 0;
#endif	//if RETARGET_USE_RX_DMA==1

/*** Internal Function Declarations ***/

/*** External Function Defines ***/
HAL_StatusTypeDef uartTerminal_init(UART_HandleTypeDef *huart)
{
	sp_huart = huart;
#if RETARGET_USE_RX_DMA==1
	HAL_UART_Receive_DMA(sp_huart, (uint8_t*) s_bufferRx, BUFFER_SIZE);
	s_bufferRxRp = 0;
#endif	//if RETARGET_USE_RX_DMA==1

	return HAL_OK;
}

HAL_StatusTypeDef uartTerminal_send(uint8_t data)
{
	HAL_StatusTypeDef ret = HAL_ERROR;
	ret = HAL_UART_Transmit(sp_huart, &data, 1, 100);

	return ret;
}

uint8_t uartTerminal_recv()
{
	uint8_t data = 0;

#if RETARGET_USE_RX_DMA==1
	while (bufferRxWp == s_bufferRxRp)
		;
	data = s_bufferRx[s_bufferRxRp++];
	s_bufferRxRp &= (BUFFER_SIZE - 1);

#else
	if (HAL_UART_Receive(sp_huart, &data, 1, 100) != HAL_OK)
		data = 0;
#endif	//if RETARGET_USE_RX_DMA==1

	return data;
}

HAL_StatusTypeDef uartTerminal_recvTry(uint8_t *data)
{
#if RETARGET_USE_RX_DMA==1
	if (bufferRxWp == s_bufferRxRp)
		return HAL_TIMEOUT;
	*data = s_bufferRx[s_bufferRxRp++];
	s_bufferRxRp &= (BUFFER_SIZE - 1);
#else
	if (HAL_UART_Receive(sp_huart, data, 1, 100) != HAL_OK) {
		*data = 0;
		return HAL_ERROR;
	}
#endif	//if RETARGET_USE_RX_DMA==1

	return HAL_OK;
}
