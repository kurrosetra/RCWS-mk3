/*
 * Button.h
 *
 *  Created on: Sep 22, 2021
 *      Author: miftakur
 */

#ifndef BUTTON_H_
#define BUTTON_H_

#include "stm_hal_serial.h"

typedef struct
{
	float hor;
	float el;
	uint8_t deadman;
	uint8_t trig;
	uint8_t aButton;
	uint8_t bButton;
} TJoystick;

typedef struct
{

} TPanelButton;

typedef struct
{
	TSerial *serial;
	TJoystick JRight;
	TJoystick JLeft;
	TPanelButton panelButton;
} TButton;

void button_init(TButton *btn, UART_HandleTypeDef *button_uart);
void button_handler(TButton *btn);

#endif /* BUTTON_H_ */
