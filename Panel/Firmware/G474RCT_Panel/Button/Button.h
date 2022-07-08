/*
 * Button.h
 *
 *  Created on: Sep 22, 2021
 *      Author: miftakur
 */

#ifndef BUTTON_H_
#define BUTTON_H_

#include "usart.h"
#include "bus_button_config.h"

typedef struct
{
	uint8_t deadman :1; /* JSR or JSL to activate */
	uint8_t trigger :1; /* JSR for firing, JSL for balistic compensation */
	uint8_t aButton :1; /* JSL for track mode, JSR for travel mode */
	uint8_t bButton :1; /* JSL for memory mode, JSR for stabilize mode */
	uint8_t reservedBit :4;
} __attribute__ ((packed)) TJoystickButton;

typedef struct
{
	int32_t az; /* JSR for manual azimuth speed or azimuth correction speed, JSL for track size (right->bigger, left->smaller)  */
	uint32_t az_max; /* maximum value of az */
	int32_t az_tem;	/* used for smoothing */
	int32_t el; /* JSR for manual elevation speed or elevation correction speed, JSL for zoom action (up->tele, down->wide)  */
	uint32_t el_max; /* maximum value of el */
	int32_t el_tem;	/* used for smoothing */
	float alpha;
	float betha;
	TJoystickButton button; /* state of joystick's buttons */
} TJoystick;

typedef struct
{
	sModeGpioInput_t input; /* state of panel buttons */
	sModeGpioOutput_t output; /* state of panel LEDs */
} TPanelButton;

typedef struct
{
	uint32_t mode_1_arr;
	uint32_t mode_3_arr;
} TFireModeVal;

typedef struct
{
	TSerial *serial; /* serial port of panel button */
	TJoystick JRight; /* joystick's Right */
	TJoystick JLeft; /* joystick's Left */
	TPanelButton panelButton; /* panel's buttons & LEDs */
	uint32_t vBat; /* battery voltage */

	TFireModeVal fire_mode_arr;	/* firing time out */
} TButton;

extern TButton button;

void button_init();
void button_handler();

void tim4_out_on(const uint32_t ch);
void tim4_out_off(const uint32_t ch);
void tim4_out_toggle(const uint32_t ch);

#endif /* BUTTON_H_ */
