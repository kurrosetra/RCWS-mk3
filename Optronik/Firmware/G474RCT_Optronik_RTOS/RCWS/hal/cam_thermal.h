/*
 * tbt.h
 *
 *  Created on: Dec 28, 2021
 *      Author: miftakur
 */

#ifndef CAMERA_TBT_H_
#define CAMERA_TBT_H_

#include "main.h"

void tbt_init();
void tbt_deInit();

void tbt_zoom(const uint32_t zoom_value);
void tbt_auto_focus_start();
void tbt_man_focus_near_speed_start(const uint8_t speed);
void tbt_man_focus_near_start();
void tbt_man_focus_far_speed_start(const uint8_t speed);
void tbt_man_focus_far_start();
void tbt_man_focus_stop();
void tbt_image_stabilizer(const uint8_t off_on);
void tbt_power(const uint8_t off_on);
void tbt_set_video_mode(const uint8_t mode);

#endif /* CAMERA_TBT_H_ */
