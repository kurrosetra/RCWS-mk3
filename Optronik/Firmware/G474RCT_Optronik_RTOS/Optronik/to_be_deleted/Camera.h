/*
 * Camera.h
 *
 *  Created on: Dec 27, 2021
 *      Author: miftakur
 */

#ifndef CAMERA_H_
#define CAMERA_H_

#include "bus_button_config.h"
#include "common.h"

typedef struct
{
	Panel_camera_command_t command;
	Optronik_camera_state_t state;
	uint8_t busy;
} Camera_t;

uint8_t cam_is_busy();
osStatus cam_read(Camera_t *cam, const uint32_t timeout);
osStatus cam_write(Camera_t *cam, const uint32_t timeout);

#endif /* CAMERA_H_ */
