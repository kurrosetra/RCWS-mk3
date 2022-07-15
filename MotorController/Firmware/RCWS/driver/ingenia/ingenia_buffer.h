/*
 * ingenia_buffer.h
 *
 *  Created on: Jan 5, 2022
 *      Author: miftakur
 */

#ifndef DRIVER_INGENIA_INGENIA_BUFFER_H_
#define DRIVER_INGENIA_INGENIA_BUFFER_H_

#include "main.h"
#include "ingenia_conf.h"

typedef struct
{
	uint32_t id;
	uint8_t len;
	uint8_t rxData[CAN_DATA_MAX];
} CAN_Data_t;

typedef struct
{
	CAN_Data_t buf[CAN_RX_BUFFER_SIZE];
	volatile uint8_t tail;
	volatile uint8_t head;
} CAN_Buffer_t;

uint8_t can_buffer_available(CAN_Buffer_t *buffer);
void can_buffer_write(CAN_Buffer_t *buffer, CAN_Data_t *data);
int8_t can_buffer_peek(CAN_Buffer_t *buffer, CAN_Data_t *data);
int8_t can_buffer_read(CAN_Buffer_t *buffer, CAN_Data_t *data);
void can_buffer_flush(CAN_Buffer_t *buffer);


#endif /* DRIVER_INGENIA_INGENIA_BUFFER_H_ */
