/*
 * sony.c
 *
 *  Created on: Dec 28, 2021
 *      Author: miftakur
 */

#include "usart.h"
#include "cam_day.h"
#include "driver/libVisca_120/visca.h"

#define DEBUG_CAM_SONY		1
#if DEBUG_CAM_SONY==1
#include <stdio.h>

#	define LOG(str, ...) printf("[sony:%d] " str,  __LINE__, ##__VA_ARGS__)
#	define LOG_E(str, ...) printf("[sony_err:%d] " str, __LINE__, ##__VA_ARGS__)
#else
#	define LOG(str, ...)
#	define LOG_E(str, ...)
#endif	//if DEBUG_CAM_SONY==1

typedef struct
{
	VISCAInterface_t iface;
	VISCACamera_t camera;
} sony_block_t;
static sony_block_t sony;

/*
 * visca command:
 * 	power on:			8101040002FF
 * 	power off:			8101040003FF
 * 	auto focus:			8101043802FF
 * 	manual focus:		8101043803FF
 * 	one push:			8101041801FF	//auto focus once
 * 	man focus stop:		8101040800FF
 * 	man near focus: 	810104083[4]FF		speed=4
 * 	man far focus:		810104082[7]FF		speed=7
 *
 * 	optical zoom max=0x4000
 *
 *
 */

void sony_init()
{
#if CAMERA_SWAP==1
	sony.iface.port_fd = &huart2;
#else
	sony.iface.port_fd = &huart4;
#endif	//if CAMERA_SWAP==1

	sony.iface.broadcast = 0;
	sony.camera.address = 1;

	LOG("init\r\n");
}

void sony_deInit()
{

}

void sony_zoom(const uint32_t zoom_value)
{
	if (VISCA_set_zoom_value(&sony.iface, &sony.camera, zoom_value) == VISCA_SUCCESS)
		LOG("zoom_val=%ld\r\n", zoom_value);
	else
		LOG_E("failed\r\n");
}

void sony_auto_focus_start()
{
	if (VISCA_set_focus_auto(&sony.iface, &sony.camera, VISCA_FOCUS_AUTO_ON) == VISCA_SUCCESS)
		LOG("auto focus start\r\n");
	else
		LOG_E("failed\r\n");
}

void sony_man_focus_near_speed_start(const uint8_t speed)
{
	uint32_t a, b;
	/* start manual focus */
	a = VISCA_set_focus_auto(&sony.iface, &sony.camera, VISCA_FOCUS_AUTO_OFF);
	/* start focus near */
	b = VISCA_set_focus_near_speed(&sony.iface, &sony.camera, speed);

	if ((a == VISCA_SUCCESS) && (b == VISCA_SUCCESS))
		LOG("man focus near start\r\n");
	else
		LOG_E("failed\r\n");
}

void sony_man_focus_near_start()
{
	uint32_t a, b;
	/* start manual focus */
	a = VISCA_set_focus_auto(&sony.iface, &sony.camera, VISCA_FOCUS_AUTO_OFF);
	/* start focus near */
	b = VISCA_set_focus_near(&sony.iface, &sony.camera);

	if ((a == VISCA_SUCCESS) && (b == VISCA_SUCCESS))
		LOG("man focus near start\r\n");
	else
		LOG_E("failed\r\n");
}

void sony_man_focus_far_speed_start(const uint8_t speed)
{
	uint32_t a, b;
	/* start manual focus */
	a = VISCA_set_focus_auto(&sony.iface, &sony.camera, VISCA_FOCUS_AUTO_OFF);
	/* start focus far */
	b = VISCA_set_focus_far_speed(&sony.iface, &sony.camera, speed);

	if ((a == VISCA_SUCCESS) && (b == VISCA_SUCCESS))
		LOG("man focus far start\r\n");
	else
		LOG_E("failed\r\n");
}

void sony_man_focus_far_start()
{
	uint32_t a, b;
	/* start manual focus */
	a = VISCA_set_focus_auto(&sony.iface, &sony.camera, VISCA_FOCUS_AUTO_OFF);
	/* start focus far */
//	b = VISCA_set_focus_far(&sony.iface, &sony.camera);
	b = VISCA_set_focus_far_speed(&sony.iface, &sony.camera, 7);

	if ((a == VISCA_SUCCESS) && (b == VISCA_SUCCESS))
		LOG("man focus far start\r\n");
	else
		LOG_E("failed\r\n");
}

void sony_man_focus_stop()
{

	/* stop manual focus */
	if (VISCA_set_focus_stop(&sony.iface, &sony.camera) == VISCA_SUCCESS)
		LOG("man focus stop\r\n");
	else
		LOG_E("failed\r\n");
}

void sony_image_stabilizer(const uint8_t off_on)
{
	uint8_t power = VISCA_CAM_STABILIZER_OFF;
	if (off_on != 0)
		power = VISCA_CAM_STABILIZER_ON;

	VISCA_set_cam_stabilizer(&sony.iface, &sony.camera, power);
	LOG("image stabilizer=%d\r\n", off_on);
}

void sony_power(const uint8_t off_on)
{
//	const static uint8_t mem_pos = 1;

	uint8_t power = VISCA_POWER_OFF;
	if (off_on != 0)
		power = VISCA_POWER_ON;

	VISCA_set_power(&sony.iface, &sony.camera, power);
	LOG("power=%d\r\n", off_on);

	if (off_on != 0) {
		sony_zoom(0x2000);
//		/* recall setting */
//		VISCA_memory_recall(&sony.iface, &sony.camera, mem_pos);
//		LOG("recall memory %d\r\n", mem_pos);

//		VISCA_set_auto_exp_mode(&sony.iface, &sony.camera, VISCA_AUTO_EXP_FULL_AUTO);
//		LOG("AE exp full auto\r\n");
//		VISCA_set_auto_exp_mode(&sony.iface, &sony.camera, VISCA_AUTO_EXP_IRIS_PRIORITY);
//		LOG("AE exp iris priority\r\n");
	}
}

void sony_set_memory_1()
{
	VISCA_memory_set(&sony.iface, &sony.camera, 1);
	LOG("set memory 1\r\n");
}

void sony_set_video_mode(const uint8_t mode)
{
	LOG("video mode!\r\n");
}
