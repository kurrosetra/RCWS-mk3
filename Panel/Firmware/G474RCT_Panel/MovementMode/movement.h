/*
 * movement.h
 *
 *  Created on: Dec 9, 2021
 *      Author: miftakur
 */

#ifndef MOVEMENT_H_
#define MOVEMENT_H_

#include "main.h"

typedef enum
{
	MODE_MANUAL,
	MODE_TRAVEL,
	MODE_STABILIZE,
	MODE_TRACK,
	MODE_MEMORY
} sMovementMode_e;

typedef struct
{
	uint8_t travel :1;
	uint8_t stabilize :1;
	uint8_t track :1;
	uint8_t memory :1;
	uint8_t reservedBit :4;
} __attribute__ ((packed)) sMovementByte_t;

typedef enum
{
	MODE_STOP = 0,
	MODE_START
} mode_start_stop_e;

typedef struct
{
	sMovementByte_t mByte;
	uint8_t mode;

} Movement_t;
extern Movement_t move;

void move_init();
void move_handler();

void move_memory_mode(const uint8_t stopStart);
void move_track_mode(const uint8_t stopStart);
void move_stabilize_mode(const uint8_t stopStart);
void move_travel_mode(const uint8_t stopStart);
void move_manual_mode(const uint8_t stopStart);

#endif /* MOVEMENT_H_ */
