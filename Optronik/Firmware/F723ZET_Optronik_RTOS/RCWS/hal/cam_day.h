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
void sony_man_focus_near_speed_start(const uint8_t speed);
void sony_man_focus_near_start();
void sony_man_focus_far_speed_start(const uint8_t speed);
void sony_man_focus_far_start();
void sony_man_focus_stop();
void sony_image_stabilizer(const uint8_t off_on);
void sony_power(const uint8_t off_on);
void sony_set_video_mode(const uint8_t mode);
void sony_set_memory_1();

#endif /* CAMERA_SONY_H_ */
