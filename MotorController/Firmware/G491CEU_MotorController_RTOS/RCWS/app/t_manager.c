/*
 * manager.c
 *
 *  Created on: May 25, 2022
 *      Author: 62812
 */

#include <stdio.h>
#include "iwdg.h"

#include "common.h"

#if RTOS_USE_STACK_HIGH_WATER==1
#include "task.h"
#endif	//if RTOS_USE_STACK_HIGH_WATER==1


#if DEBUG_MANAGER==1
/*** Internal Const Values, Macros ***/
#	define LOG(str, ...) printf("[%ld TMan:%d] " str, (osKernelSysTick()%10000UL), __LINE__, ##__VA_ARGS__)
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
			LOG("recv: 0x%X -> 0x%X\r\n", notif, allNotif);

//#if DEBUG_BUS_RECV==1
//			if (notif == T_Bus_Recv_id)
//				printf("\t\tBus_Recv notif recv\r\n");
//#endif	//if DEBUG_BUS_RECV==1
//
//#if DEBUG_BUS_SEND==1
//			if (notif == T_Bus_Send_id)
//				printf("\t\tBus_Send notif recv\r\n");
//#endif	//if DEBUG_BUS_SEND==1
//
//#if DEBUG_MOTOR==1
//			if (notif == T_Motor_id)
//				printf("\t\tMotor notif recv\r\n");
//#endif	//if DEBUG_MOTOR==1
//
//#if DEBUG_WEAPON==1
//			if (notif == T_Weapon_id)
//				printf("\t\tWeapon notif recv\r\n");
//#endif	//if DEBUG_WEAPON==1

			if (allNotif == T_Manager_id) {
				allNotif = 0;
				/* refresh wdt timer */
				HAL_IWDG_Refresh(&hiwdg);

				LOG("\t\tWDT kicked in!\r\n");
//				HAL_GPIO_TogglePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin);
			}
		}
		else if (event.status == osEventTimeout) {
			LOG("timeout!\r\n");
		}
	}
	/* USER CODE END t_manager */
}
