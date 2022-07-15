/*
 * uartTerminal.h
 *
 *  Created on: Dec 29, 2021
 *      Author: miftakur
 */

#ifndef DRIVER_UARTTERMINAL_H_
#define DRIVER_UARTTERMINAL_H_

#include "main.h"
#include "usart.h"

#define RETARGET_USE_RX_DMA				0

void retarget_init();

HAL_StatusTypeDef  uartTerminal_init(UART_HandleTypeDef *huart);
HAL_StatusTypeDef uartTerminal_send(uint8_t data);
uint8_t uartTerminal_recv();
HAL_StatusTypeDef uartTerminal_recvTry(uint8_t *data);

#endif /* DRIVER_UARTTERMINAL_H_ */
