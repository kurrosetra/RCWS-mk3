/*
 * tbt.h
 *
 *  Created on: Dec 30, 2021
 *      Author: miftakur
 */

#ifndef DRIVER_TBT_TBT_H_
#define DRIVER_TBT_TBT_H_

#include "main.h"
#include "tbt_conf.h"
#include "tbt_def.h"

/* timeout in ms */
#define TBT_SERIAL_LONG_WAIT				5000
#define TBT_SERIAL_WAIT						100

/* size of the local packet buffer */
#define TBT_INPUT_BUFFER_SIZE				20
#define TBT_OUTPUT_BUFFER_SIZE				6

/* This is the interface for the STM platform.
 */
typedef struct
{
	/* uart */
	UART_HandleTypeDef *port_fd;

	/* input buffer */
	uint8_t ibuf[TBT_INPUT_BUFFER_SIZE];
	uint8_t bytes;

	/* output buffer */
	uint8_t oBuf[TBT_OUTPUT_BUFFER_SIZE];
} TBT_interface_t;

uint32_t TBT_set_zoom_optical_stop(TBT_interface_t *iface);
uint32_t TBT_set_zoom_optical_start(TBT_interface_t *iface, const uint8_t zoom_end,
		const uint8_t speed);
uint32_t TBT_set_zoom_optical_direct(TBT_interface_t *iface, const uint16_t zoom,
		const uint8_t speed);
uint32_t TBT_set_focus_stop(TBT_interface_t *iface);
uint32_t TBT_set_focus_start(TBT_interface_t *iface, const uint8_t focus_end, const uint8_t speed);
uint32_t TBT_set_focus_direct(TBT_interface_t *iface, const uint16_t focus, const uint8_t speed);
uint32_t TBT_set_focus_auto_one_push(TBT_interface_t *iface);
uint32_t TBT_set_image_stabilizer(TBT_interface_t *iface, const uint8_t power);

#endif /* DRIVER_TBT_TBT_H_ */
