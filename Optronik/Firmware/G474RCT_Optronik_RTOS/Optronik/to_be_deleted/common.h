/*
 * common.h
 *
 *  Created on: Jan 1, 2022
 *      Author: miftakur
 */

#ifndef COMMON_H_
#define COMMON_H_

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#define DEBUG_ENABLE			1
#if DEBUG_ENABLE==1
#	define DEBUG_BUFSIZE		64

#	define DEBUG_MANAGER		0
#	define DEBUG_BUS			1
#	define DEBUG_CAMERA			1
#	define DEBUG_LRF			1
#endif	//if DEBUG_ENABLE==1

typedef enum
{
	Q_CAMERA_NOTIF,
	Q_LRF_NOTIF,
	Q_MANAGER_NOTIF,
} QUEUE_ID;

typedef enum
{
	M_CAMERA_STATE,
	M_LRF_STATE,
} MUTEX_ID;

typedef enum
{
	T_Manager_id,
	T_Bus_Recv_id = 1,
	T_Camera_id = 2,
	T_Lrf_id = 4,
	T_Bus_Send_id = 8,
} Task_ID;

typedef enum
{
	Tim_Camera_Notif_id,
	Tim_Camera_Timeout_id,
	Tim_Lrf_Notif_id,
	Tim_Lrf_Timeout_id,
} Timer_ID;

osMessageQId opt_get_queue(const QUEUE_ID qID);
osMutexId opt_get_mutex(const MUTEX_ID mID);
osMailQId opt_get_bus_mail();
osTimerId opt_get_timer(const Timer_ID tID);

#endif /* COMMON_H_ */
