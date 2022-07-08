/*
 * tbt.c
 *
 *  Created on: Dec 28, 2021
 *      Author: miftakur
 */

#include "usart.h"
#include "cam_thermal.h"
#include "driver/tbt/tbt.h"

#define DEBUG_CAM_TBT		1
#if DEBUG_CAM_TBT==1
#include <stdio.h>

#	define LOG(str, ...) printf("[tbt:%d] " str,  __LINE__, ##__VA_ARGS__)
#	define LOG_E(str, ...) printf("[tbt_err:%d] " str, __LINE__, ##__VA_ARGS__)
#else
#	define LOG(str, ...)
#	define LOG_E(str, ...)
#endif	//if DEBUG_CAM_TBT==1

static TBT_interface_t tbt = { .port_fd = &huart5 };

void tbt_init()
{
	LOG("init\r\n");
}

void tbt_deInit()
{
}

void tbt_zoom(uint32_t zoom_value)
{
	if (TBT_set_zoom_optical_direct(&tbt, zoom_value,
	TBT_OPTICAL_ZOOM_DIRECT_SPEED_MAX) == TBT_SUCCESS)
		LOG("zoom_val=%ld\r\n", zoom_value);
	else
		LOG_E("failed\r\n");
}

void tbt_auto_focus_start()
{
	if (TBT_set_focus_auto_one_push(&tbt) == TBT_SUCCESS)
		LOG("auto focus start\r\n");
	else
		LOG_E("failed\r\n");
}

void tbt_man_focus_near_start()
{
	if (TBT_set_focus_start(&tbt, TBT_FOCUS_START_NEAR, TBT_FOCUS_START_SPEED_MAX) == TBT_SUCCESS)
		LOG("man focus near start\r\n");
	else
		LOG_E("failed\r\n");
}

void tbt_man_focus_far_start()
{
	if (TBT_set_focus_start(&tbt, TBT_FOCUS_START_FAR, TBT_FOCUS_START_SPEED_MAX) == TBT_SUCCESS)
		LOG("man focus far start\r\n");
	else
		LOG_E("failed\r\n");
}

void tbt_man_focus_stop()
{
	if (TBT_set_focus_stop(&tbt) == TBT_SUCCESS)
		LOG("man focus stop\r\n");
	else
		LOG_E("failed\r\n");
}

void tbt_image_stabilizer(const uint8_t off_on)
{
	uint8_t power = TBT_IMAGE_STABILIZER_OFF;
	if (off_on == 1)
		power = TBT_IMAGE_STABILIZER_ON;

	if (TBT_set_image_stabilizer(&tbt, power))
		LOG("image stabilizer=%d\r\n", off_on);
	else
		LOG_E("failed\r\n");
}
