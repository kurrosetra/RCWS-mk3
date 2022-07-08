/*
 * manager.c
 *
 *  Created on: Jan 17, 2022
 *      Author: miftakur
 */

#include <stdio.h>

#include "common.h"

#include "iwdg.h"

#if DEBUG_MANAGER==1
/*** Internal Const Values, Macros ***/
#	define LOG(str, ...) printf("[%ld TMan:%d] " str, osKernelSysTick(), __LINE__, ##__VA_ARGS__)
#	define LOG_E(str, ...) printf("[TMan_Err:%d] " str, __LINE__, ##__VA_ARGS__)
#else
#	define LOG(str, ...)
#	define LOG_E(str, ...)
#endif	//if DEBUG_MANAGER==1

void t_manager(void const *argument)
{
	/* USER CODE BEGIN t_manager */
	LOG("created!\r\n");

	osThreadSetPriority(osThreadGetId(), osPriorityIdle);

	osDelay(T_Manager_id);

	/* Infinite loop */
	uint16_t allNotif = 0;
	for ( ;; ) {
		osEvent event = osMessageGet(opt_get_queue(Q_MANAGER_NOTIF), 1000);
		if (event.status == osEventMessage) {
			uint16_t notif = event.value.v;
			allNotif |= notif;
			LOG("recv: 0x%X\r\n", notif);
//#if DEBUG_LRF==1
//			if(notif==T_Lrf_id)
//				printf("\t\tLRF notif recv\r\n");
//#endif	//if DEBUG_LRF==1

			if (allNotif >= 0b1111) {
				allNotif = 0;
				/* refresh wdt timer */
				HAL_IWDG_Refresh(&hiwdg);

				LOG("\t\tWDT kicked in!\r\n");
			}
		}
		else if (event.status == osEventTimeout) {
			LOG("timeout!\r\n");
		}
	}
	/* USER CODE END t_manager */
}

