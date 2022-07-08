/*
 * pc.h
 *
 *  Created on: Dec 9, 2021
 *      Author: miftakur
 */

#ifndef PC_H_
#define PC_H_

#include "main.h"
#include "usart.h"

typedef struct
{
	uint8_t weaponType :1;
	uint8_t reservedBit :7;
} __attribute__ ((packed)) pc_state_recv_t;

typedef struct
{
	pc_state_recv_t state;
	uint16_t balistic_px_offset;
} PC_recv_t;

typedef enum
{
	PC_TRK_GATE_STOP = 0,
	PC_TRK_GATE_BIGGER = 1,
	PC_TRK_GATE_SMALLER = 2,
} PC_track_gate_resize_e;

typedef struct
{
	uint8_t camera :1; /* OPT camera state */
	uint8_t zoom :2; /* OPT zoom level */
	uint8_t trackerTargetSelect :2; /* tracker target selection */
	uint8_t trackerGateResize :2; /* tracker gate size command */
	uint8_t reservedBit :1;
} __attribute__ ((packed)) pc_state_command_t;

typedef struct
{
	pc_state_command_t state;
	uint16_t lrfRange; /* lrf range storage, in m */
	uint16_t angleAz; /* angle of rcws's azimuth relative to vehicle, in degrees */
	uint16_t angleEl; /* angle of rcws's elevation relative to vehicle, in 1/100 degrees */
	uint16_t munCounter;
} PC_send_t;

typedef struct
{
	TSerial *port; /* pc's serial port */

	PC_recv_t recv; /* storage from pc */
	PC_send_t send; /* storage to pc */
} PC_t;
extern PC_t comp;

void pc_init(void);
void pc_handler(void);

#endif /* PC_H_ */
