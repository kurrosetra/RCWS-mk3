/*
 * Button.c
 *
 *  Created on: Sep 22, 2021
 *      Author: miftakur
 */

#include "Button.h"

uint8_t rxBtnBuffer[RING_BUFFER_RX_SIZE];
uint8_t txBtnBuffer[RING_BUFFER_TX_SIZE];
TSerial button;

void button_init(TButton *btn, UART_HandleTypeDef *button_uart)
{
	btn->serial = &button;
	serial_init(btn->serial, (char*) &rxBtnBuffer, sizeof(rxBtnBuffer), (char*) &txBtnBuffer,
			sizeof(txBtnBuffer), button_uart);

}

void button_handler(TButton *btn)
{

}

void USART2_IRQHandler(void)
{
	USARTx_IRQHandler(&button);
}

