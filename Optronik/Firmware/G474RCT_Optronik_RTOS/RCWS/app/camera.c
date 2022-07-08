/*
 * camera.c
 *
 *  Created on: Jan 17, 2022
 *      Author: miftakur
 */

#include <stdio.h>

#include "common.h"
#include "hal/cam_day.h"
#include "hal/cam_thermal.h"

/*** Internal Const Values, Macros ***/
#if DEBUG_CAMERA==1
#	define LOG(str, ...) printf("[%ld TCam:%d] " str, osKernelSysTick(), __LINE__, ##__VA_ARGS__)
#	define LOG_E(str, ...) printf("[TCam_Err:%d] " str, __LINE__, ##__VA_ARGS__)
#else
#	define LOG(str, ...)
#	define LOG_E(str, ...)
#endif	//if DEBUG_CAMERA==1

uint8_t g_camera_ready = 0;
static const uint16_t managerNotif = T_Camera_id;

static Camera_t camera_info;

static void (*camera_init)();
static void (*camera_zoom_direct)(const uint32_t zoom_value);
static void (*camera_auto_focus_start)();
static void (*camera_man_focus_near_speed_start)(const uint8_t speed);
static void (*camera_man_focus_near_start)();
static void (*camera_man_focus_far_speed_start)(const uint8_t speed);
static void (*camera_man_focus_far_start)();
static void (*camera_man_focus_stop)();
static void (*camera_image_stabilizer)(const uint8_t off_on);
static void (*camera_power)(const uint8_t off_on);
static void (*camera_set_video_mode)(const uint8_t mode);
static void (*camera_set_memory_1)();

static void cam_select(const uint8_t cam_num);
static uint32_t cam_get_zoom_value(const uint8_t cam_num, const uint8_t zoom_level);

static uint32_t cam_thermal_start_timer = 0;

static void cam_timer_start()
{
	osTimerStart(opt_get_timer(Tim_Camera_Notif_id), 500);
	osTimerStart(opt_get_timer(Tim_Camera_Timeout_id), 10000);
	LOG("timer start\r\n");
}

static void cam_timer_stop()
{
	osTimerStop(opt_get_timer(Tim_Camera_Notif_id));
	osTimerStop(opt_get_timer(Tim_Camera_Timeout_id));
	LOG("timer stop\r\n");
}

void osTimerCameraNotifCallback(void const *argument)
{
	/* USER CODE BEGIN osTimerCallback */
	(void) argument;

	/* send notif to task manager that this thread is still running */
	osMessagePut(opt_get_queue(Q_MANAGER_NOTIF), managerNotif, 0);

	/* USER CODE END osTimerCallback */
}

void osTimerCameraTimeoutCallback(void const *argument)
{
	/* USER CODE BEGIN osTimerCallback */
	(void) argument;

	cam_timer_stop();
	LOG_E("timeout!\r\n");

	/* USER CODE END osTimerCallback */
}

void t_camera(void const *argument)
{
	/* USER CODE BEGIN t_camera */

	LOG("created!\r\n");

	/* TODO Built-In Test */
	cam_select(camera_info.state.cameraActive);

//	if ((g_reset_cause == RESET_CAUSE_INDEPENDENT_WATCHDOG_RESET)
//			|| (g_reset_cause == RESET_CAUSE_SOFTWARE_RESET)
//			|| (g_reset_cause == RESET_CAUSE_EXTERNAL_RESET_PIN_RESET)) {
////	if ((g_reset_cause == RESET_CAUSE_INDEPENDENT_WATCHDOG_RESET)
////			|| (g_reset_cause == RESET_CAUSE_SOFTWARE_RESET)){
//		/* set zoomLevel to OPT_ZOOM_LEVEL_1 */
//		camera_zoom_direct(cam_get_zoom_value(camera_info.state.cameraActive, OPT_ZOOM_LEVEL_1));
//		/* set to auto-focus */
//		camera_auto_focus_start();
//	}

//	g_camera_ready = 1;
	LOG("camera ready!\r\n");

	osDelay(T_Camera_id);

	/* Infinite loop */
	uint8_t _focus_speed = 7;
	for ( ;; ) {
		Panel_camera_command_t notif;

		osEvent event = osMessageGet(opt_get_queue(Q_CAMERA_NOTIF), 1000);
		if (event.status == osEventMessage) {
			*(uint8_t*) &notif = (uint8_t) event.value.v;
			if (*(uint8_t*) &camera_info.command != *(uint8_t*) &notif) {
				if (osMutexWait(opt_get_mutex(M_CAMERA_STATE), 100) == osOK) {
					camera_info.busy = 1;
					/* camera changed */
					if (camera_info.command.cameraActive != notif.cameraActive) {
						LOG("cmd to %d\r\n", notif.cameraActive);

						/* redirect notif to task manager by timer */
						cam_timer_start();

						/* set camera */
						cam_select(notif.cameraActive);
						camera_info.command.cameraActive = notif.cameraActive;
						if (notif.cameraActive == CAM_SELECT_DAY) {
							/* set zoomLevel to OPT_ZOOM_LEVEL_1 */
							camera_zoom_direct(
									cam_get_zoom_value(notif.cameraActive, OPT_ZOOM_LEVEL_1));

							/* set to auto-focus */
							camera_auto_focus_start();
							_focus_speed = 7;

							camera_info.state.zoomLevel = OPT_ZOOM_LEVEL_1;
						}

						camera_info.state.cameraActive = notif.cameraActive;
						/* stop timer */
						cam_timer_stop();
					}
					else {
						/* zoom changed */
						if (camera_info.command.zoom != notif.zoom) {
							if (notif.zoom == ZF_IN) {
								if (camera_info.state.zoomLevel < OPT_ZOOM_LEVEL_3) {
									camera_info.state.zoomLevel++;

									/* send notif to task manager by timer */
									cam_timer_start();

									camera_zoom_direct(
											cam_get_zoom_value(camera_info.state.cameraActive,
													camera_info.state.zoomLevel));
//									if (camera_info.state.cameraActive == CAM_SELECT_DAY)
									if (camera_info.state.cameraActive == CAM_SELECT_THERMAL)
										osDelay(3000);
									/* set to auto-focus */
									camera_auto_focus_start();
									if (camera_info.state.cameraActive == CAM_SELECT_THERMAL)
										osDelay(3000);

									cam_timer_stop();
								}
							}
							else if (notif.zoom == ZF_OUT) {
								if (camera_info.state.zoomLevel > OPT_ZOOM_LEVEL_1) {
									camera_info.state.zoomLevel--;

									/* send notif to task manager by timer */
									cam_timer_start();

									camera_zoom_direct(
											cam_get_zoom_value(camera_info.state.cameraActive,
													camera_info.state.zoomLevel));
//									if (camera_info.state.cameraActive == CAM_SELECT_DAY)
									if (camera_info.state.cameraActive == CAM_SELECT_THERMAL)
										osDelay(3000);
									/* set to auto-focus */
									camera_auto_focus_start();
									if (camera_info.state.cameraActive == CAM_SELECT_THERMAL)
										osDelay(3000);

									cam_timer_stop();
								}
//								else {
//									if (camera_info.state.cameraActive == CAM_SELECT_DAY)
//										camera_set_memory_1();
//								}
							}
							camera_info.command.zoom = notif.zoom;
							if (camera_info.command.zoom == OPT_ZOOM_LEVEL_1)
								_focus_speed = 4;
							else if (camera_info.command.zoom == OPT_ZOOM_LEVEL_2)
								_focus_speed = 3;
							else if (camera_info.command.zoom == OPT_ZOOM_LEVEL_3)
								_focus_speed = 2;
						}

						/* focus changed */
						if (camera_info.command.focus != notif.focus) {
							/* send notif to task manager by timer */
							cam_timer_start();

							if (notif.focus == ZF_IN) {
//								camera_man_focus_far_start();
								camera_man_focus_far_speed_start(_focus_speed);
							}
							else if (notif.focus == ZF_OUT) {
//								camera_man_focus_near_start();
								camera_man_focus_near_speed_start(_focus_speed);
							}
							else
								camera_man_focus_stop();

							cam_timer_stop();

							camera_info.command.focus = notif.focus;
						}
					}
					camera_info.busy = 0;
					osMutexRelease(opt_get_mutex(M_CAMERA_STATE));

				}	//if (osMutexWait(opt_get_mutex(M_CAMERA_STATE), 100) == osOK) {
				else
					LOG_E("mutex timeout!\r\n");
			}

		}

		if ((g_camera_ready == 0) && (osKernelSysTick() >= 7500)) {
			cam_select(camera_info.state.cameraActive);
			g_camera_ready = 1;
		}

//		if (cam_thermal_start_timer != 0 && osKernelSysTick() >= cam_thermal_start_timer) {
//			cam_thermal_start_timer = 0;
//
//			camera_set_video_mode(1);
//		}

		/* send notif to task manager that this thread is still running */
		osMessagePut(opt_get_queue(Q_MANAGER_NOTIF), managerNotif, 0);
	}
	/* USER CODE END t_camera */
}

uint8_t cam_is_busy()
{
	return camera_info.busy;
}

osStatus cam_read(Camera_t *cam, const uint32_t timeout)
{
	osStatus ret = osMutexWait(opt_get_mutex(M_CAMERA_STATE), timeout);
	if (ret == osOK) {
		*cam = camera_info;

		ret = osMutexRelease(opt_get_mutex(M_CAMERA_STATE));
	}

	return ret;
}

osStatus cam_write(Camera_t *cam, const uint32_t timeout)
{
	osStatus ret = osMutexWait(opt_get_mutex(M_CAMERA_STATE), timeout);
	if (ret == osOK) {
		camera_info = *cam;

		ret = osMutexRelease(opt_get_mutex(M_CAMERA_STATE));
	}

	return ret;
}

static void cam_sony_selected()
{
	camera_init = &sony_init;
	camera_zoom_direct = &sony_zoom;
	camera_auto_focus_start = &sony_auto_focus_start;
	camera_man_focus_near_speed_start = &sony_man_focus_near_speed_start;
	camera_man_focus_near_start = &sony_man_focus_near_start;
	camera_man_focus_far_speed_start = &sony_man_focus_far_speed_start;
	camera_man_focus_far_start = &sony_man_focus_far_start;
	camera_man_focus_stop = &sony_man_focus_stop;
	camera_image_stabilizer = &sony_image_stabilizer;
	camera_power = &sony_power;
	camera_set_video_mode = &sony_set_video_mode;
	camera_set_memory_1 = &sony_set_memory_1;
}

static void cam_tbt_selected()
{
	camera_init = &tbt_init;
	camera_zoom_direct = &tbt_zoom;
	camera_auto_focus_start = &tbt_auto_focus_start;
	camera_man_focus_near_speed_start = &tbt_man_focus_near_speed_start;
	camera_man_focus_near_start = &tbt_man_focus_near_start;
	camera_man_focus_far_speed_start = &tbt_man_focus_far_speed_start;
	camera_man_focus_far_start = &tbt_man_focus_far_start;
	camera_man_focus_stop = &tbt_man_focus_stop;
	camera_image_stabilizer = &tbt_image_stabilizer;
	camera_power = &tbt_power;
	camera_set_video_mode = &tbt_set_video_mode;
}

static void cam_select(const uint8_t cam_num)
{
	if (cam_num == CAM_SELECT_DAY) {
//		camera_set_video_mode(1);
		HAL_GPIO_WritePin(CAMERA_SELECT_GPIO_Port, CAMERA_SELECT_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(THERMAL_ENABLE_GPIO_Port, THERMAL_ENABLE_Pin, GPIO_PIN_RESET);
		cam_sony_selected();
		camera_init();
		/* sony cam power on */
		camera_power(1);
	}
	else if (cam_num == CAM_SELECT_THERMAL) {
		/* sony cam power off */
		camera_power(0);
		HAL_GPIO_WritePin(THERMAL_ENABLE_GPIO_Port, THERMAL_ENABLE_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(CAMERA_SELECT_GPIO_Port, CAMERA_SELECT_Pin, GPIO_PIN_SET);
		cam_tbt_selected();
		camera_init();
		/* todo wait comm to ready */
		HAL_Delay(5000);
//		camera_set_video_mode(1);
//		cam_thermal_start_timer = osKernelSysTick() + 60000;
	}
}

static uint32_t cam_get_zoom_value(const uint8_t cam_num, const uint8_t zoom_level)
{
	uint32_t ret = 0;

	if (cam_num == CAM_SELECT_THERMAL) {
		if (zoom_level == OPT_ZOOM_LEVEL_2)
			ret = 5780;
		else if (zoom_level == OPT_ZOOM_LEVEL_3)
			ret = 11999;
		else
			ret = 1;
	}
	else {
		if (zoom_level == OPT_ZOOM_LEVEL_2)
			ret = 0x4000;
		else if (zoom_level == OPT_ZOOM_LEVEL_3)
			ret = 0x6000;
		else
			ret = 0x2000;
	}

	return ret;
}
