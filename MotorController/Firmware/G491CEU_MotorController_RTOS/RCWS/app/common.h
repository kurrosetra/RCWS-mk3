/*
 * common.h
 *
 *  Created on: May 25, 2022
 *      Author: 62812
 */

#ifndef APP_COMMON_H_
#define APP_COMMON_H_

#include "main.h"
#include "cmsis_os.h"

#include "common_message.h"

#define DEBUG_ENABLE				1
#if DEBUG_ENABLE==1
#	define DEBUG_BUFSIZE			64

#	define DEBUG_MANAGER			0
#	define DEBUG_BUS				0
#	define DEBUG_MOTOR				1
#	define DEBUG_WEAPON				0
#	define DEBUG_IMU				0
#endif	//if DEBUG_ENABLE==1

osMessageQId opt_get_queue(const Queue_ID qID);
osMailQId mtr_get_mail(const Mail_ID id);
osTimerId mtr_get_timer(const Timer_ID tID);
osMutexId mtr_get_mutex(const Mutex_ID mId);

#endif /* APP_COMMON_H_ */
