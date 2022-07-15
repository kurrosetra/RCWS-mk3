/*
 * button.h
 *
 *  Created on: Jun 10, 2022
 *      Author: 62812
 */

#ifndef BUTTON_H_
#define BUTTON_H_

#include "main.h"
#include "rws_command_state.h"

typedef enum
{
	FIRING_MODE_CONT,
	FIRING_MODE_3,
	FIRING_MODE_1
} firing_mode_e;

typedef enum
{
	TARGET_NONE,
	TARGET_NEXT,
	TARGET_PREV
} target_selector_e;

typedef enum
{
	LRF_MAN_NONE,
	LRF_MAN_UP,
	LRF_MAN_DOWN
} lrf_man_mode_e;


#endif /* BUTTON_H_ */
