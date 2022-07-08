/*
 * common.h
 *
 *  Created on: Jan 17, 2022
 *      Author: miftakur
 */

#ifndef APP_COMMON_H_
#define APP_COMMON_H_

//#include "FreeRTOS.h"
//#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "common_message.h"

#define DEBUG_ENABLE			1
#if DEBUG_ENABLE==1
#	define DEBUG_BUFSIZE		64

#	define DEBUG_MANAGER		0
#	define DEBUG_BUS			1
#	define DEBUG_CAMERA			1
#	define DEBUG_LRF			0
#endif	//if DEBUG_ENABLE==1

osMessageQId opt_get_queue(const QUEUE_ID qID);
osMutexId opt_get_mutex(const MUTEX_ID mID);
osMailQId opt_get_bus_mail();
osTimerId opt_get_timer(const Timer_ID tID);

uint8_t cam_is_busy();
osStatus cam_read(Camera_t *cam, const uint32_t timeout);
osStatus cam_write(Camera_t *cam, const uint32_t timeout);

uint8_t lrf_is_busy();
osStatus lrf_read(Lrf_t *lrf, const uint32_t timeout);
osStatus lrf_write(Lrf_t *lrf, const uint32_t timeout);

extern reset_cause_e g_reset_cause;
extern uint8_t g_camera_ready;
extern uint8_t g_lrf_ready;

#endif /* APP_COMMON_H_ */
