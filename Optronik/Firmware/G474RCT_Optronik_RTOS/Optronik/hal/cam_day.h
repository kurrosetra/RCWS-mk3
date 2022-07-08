/*
 * sony.h
 *
 *  Created on: Dec 28, 2021
 *      Author: miftakur
 */

#ifndef CAMERA_SONY_H_
#define CAMERA_SONY_H_

#include "main.h"

void sony_init();
void sony_deInit();

void sony_zoom(const uint32_t zoom_value);
void sony_auto_focus_start();
void sony_man_focus_near_start();
void sony_man_focus_far_start();
void sony_man_focus_stop();
void sony_image_stabilizer(const uint8_t off_on);

#endif /* CAMERA_SONY_H_ */
