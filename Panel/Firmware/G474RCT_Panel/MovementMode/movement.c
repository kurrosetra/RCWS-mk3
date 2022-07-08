/*
 * movement.c
 *
 *  Created on: Dec 9, 2021
 *      Author: miftakur
 */

#include "movement.h"

#include "busPanel.h"
#include "Button.h"

Movement_t move;

void move_init()
{
	move.mode = MODE_MANUAL;
	*(uint8_t*) &move.mByte = 0;
}

void move_handler()
{
	/* TODO add movement algorithm here*/
	if (button.JRight.button.deadman == SET) {
		bus.panelButtonCommand.movementMode.panEnable = 1;
		bus.panelButtonCommand.panSpeedCommand = button.JRight.az;
		bus.panelButtonCommand.movementMode.tiltEnable = 1;
		bus.panelButtonCommand.tiltSpeedCommand = button.JRight.el;
	}
	else {
		bus.panelButtonCommand.movementMode.panEnable =
				bus.panelButtonCommand.movementMode.tiltEnable = 0;
		bus.panelButtonCommand.panSpeedCommand = bus.panelButtonCommand.tiltSpeedCommand = 0;
	}
}

void move_memory_mode(const uint8_t stopStart)
{
	/* stop memory mode */
	if (stopStart == MODE_STOP) {
		move.mByte.memory = RESET;
		if (move.mByte.track == SET)
			move_track_mode(MODE_START);
		else if (move.mByte.stabilize == SET)
			move_stabilize_mode(MODE_START);
		else if (move.mByte.travel == SET)
			move_travel_mode(MODE_START);
		else
			move_manual_mode(MODE_START);
	}
	/* TODO start memory mode*/
	else {
		move.mByte.memory = SET;
	}
}

void move_track_mode(const uint8_t stopStart)
{
	/* stop track mode */
	if (stopStart == MODE_STOP) {
		move.mByte.track = RESET;
		if (move.mByte.stabilize == SET)
			move_stabilize_mode(MODE_START);
		else if (move.mByte.travel == SET)
			move_travel_mode(MODE_START);
		else
			move_manual_mode(MODE_START);
	}
	else {
		/* TODO start track mode*/
		move.mByte.track = SET;
	}

}

void move_stabilize_mode(const uint8_t stopStart)
{
	/* stop stabilize mode */
	if (stopStart == MODE_STOP) {
		move.mByte.stabilize = RESET;
		if (move.mByte.track == SET)
			move_travel_mode(MODE_START);
		else
			move_manual_mode(MODE_START);
	}
	else {
		/* TODO start stabilize mode*/
		move.mByte.stabilize = SET;

	}
}

void move_travel_mode(const uint8_t stopStart)
{
	/* stop travel mode */
	if (stopStart == MODE_STOP) {
		move.mByte.travel = RESET;
		move_manual_mode(MODE_START);
	}
	else {
		/* TODO start travel mode*/
		move.mByte.travel = SET;
		move_manual_mode(MODE_STOP);
	}
}

void move_manual_mode(const uint8_t stopStart)
{
	if (stopStart == MODE_STOP) {

	}
	else {

	}
}

