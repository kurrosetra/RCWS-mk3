/*
 * stm_hal_serial.h
 *
 *  Created on: Nov 24, 2018
 *      Author: miftakur
 *
 *  ChangeLog:
 *  20210331:
 *  	- to be more general to accommodate G4 series
 *  	- porting to new ring_buffer library
 *  	- add retarget_stdio library
 *  20190108:
 *  	- add return value from IRQ handler
 *  	- add init function
 */

#ifndef STM_HAL_SERIAL_H_
#define STM_HAL_SERIAL_H_

#include "ring_buffer.h"

typedef enum
{
	HAL_UART_RETURN_NONE,
	HAL_UART_RETURN_RX,
	HAL_UART_RETURN_RX_IDLE,
	HAL_UART_RETURN_TX_INGOING,
	HAL_UART_RETURN_TX_FULL,
	HAL_UART_RETURN_TX_DONE
} HAL_UART_ReturnTypeDef;

typedef struct
{
	Ring_Buffer_t TBufferRx;
	Ring_Buffer_t TBufferTx;
	UART_HandleTypeDef *huart;
} TSerial;

uint8_t USARTx_IRQHandler(TSerial *serial);
void retarget_init(TSerial *serial);

void serial_init(TSerial *serial, char *rxBuffer, const uint16_t rxBufsize, char *txBuffer, const uint16_t txBufsize,
		UART_HandleTypeDef *huart);
uint16_t serial_available(TSerial *serial);
uint16_t serial_read_free(TSerial *serial);
uint16_t serial_write_free(TSerial *serial);

char serial_peek(TSerial *serial);
char serial_peekn(TSerial *serial, const uint16_t pos);
char serial_read(TSerial *serial);
HAL_StatusTypeDef serial_read_str(TSerial *serial, char *str);
uint16_t serial_read_until(TSerial *serial, char *str, const char stopChar);
void serial_read_flush(TSerial *serial);

void serial_write(TSerial *serial, const char c);
void serial_write_str(TSerial *serial, const char *str, const uint16_t len);
void serial_write_flush(TSerial *serial);

#endif /* STM_HAL_SERIAL_H_ */
