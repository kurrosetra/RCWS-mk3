/*
 * bus.c
 *
 *  Created on: Jan 17, 2022
 *      Author: miftakur
 */

#include <stdio.h>
#include <string.h>

#include "common.h"
#include "hal/hal_bus.h"

/*** Internal Const Values, Macros ***/
#if DEBUG_BUS==1
#	define LOG(str, ...) printf("[%ld TBus:%d] " str, osKernelSysTick(), __LINE__, ##__VA_ARGS__)
#	define LOG_E(str, ...) printf("[TBus_Err:%d] " str, __LINE__, ##__VA_ARGS__)
#else
#	define LOG(str, ...)
#	define LOG_E(str, ...)
#endif	//if DEBUG_BUS==1

void t_bus_send(void const *argument)
{
	/* USER CODE BEGIN t_bus_send */
	Bus_Tx_Buffer_t bus_tx_camera = { .id = RWS_OPTRONIK_CAM_ID, .datalength = 1 };
	Bus_Tx_Buffer_t bus_tx_lrf = { .id = RWS_OPTRONIK_LRF_ID, .datalength = 8 };
	uint16_t managerNotif = T_Bus_Send_id;
	static Camera_t _cam = { .busy = 0 };
	static Lrf_t _lrf = { .busy = 0 };

	*(uint8_t*) &_cam.command = 0;
	*(uint8_t*) &_cam.state = 0;
	*(uint8_t*) &_lrf.command = 0;

	_lrf.state.lrf_enable = 0;
	_lrf.state.value.counter = _lrf.state.value.d[0] = _lrf.state.value.d[1] = _lrf.state.value.d[2] = 0;

	LOG("S: created!\r\n");

	while (g_camera_ready == 0 || g_lrf_ready == 0) {
		osDelay(100);
		/* send notif to task manager that this thread is still running */
		osMessagePut(opt_get_queue(Q_MANAGER_NOTIF), managerNotif, 0);
	}

	osDelay(T_Bus_Send_id);

	/* Infinite loop */
	uint32_t PreviousWakeTime;
	for ( ;; ) {
		osDelayUntil(&PreviousWakeTime, 100);

		/* get value from mutex */
		if (cam_read(&_cam, 10) == osOK)
			bus_tx_camera.data[0] = *(uint8_t*) &_cam.state;

		if (lrf_read(&_lrf, 10) == osOK) {
			bus_tx_lrf.data[0] = _lrf.state.value.counter;
			bus_tx_lrf.data[0] = 0;
			for ( int i = 0; i < 3; i++ ) {
				bus_tx_lrf.data[2 + (i * 2)] = _lrf.state.value.d[i] >> 8;
				bus_tx_lrf.data[3 + (i * 2)] = _lrf.state.value.d[i] & 0xFF;
			}
		}

		/* Start the Transmission process */
		hal_bus_send(&bus_tx_camera);
		hal_lrf_send(&bus_tx_lrf);
		/* send notif to task manager that this thread is still running */
		osMessagePut(opt_get_queue(Q_MANAGER_NOTIF), managerNotif, 0);
	}
	/* USER CODE END t_bus_send */
}

void t_bus_recv(void const *argument)
{
	/* USER CODE BEGIN t_bus_recv */
	static Camera_t _cam = { .busy = 0 };
	static Lrf_t _lrf = { .busy = 0 };
	uint16_t managerNotif = T_Bus_Recv_id;

	*(uint8_t*) &_cam.command = 0;
	*(uint8_t*) &_cam.state = 0;
	*(uint8_t*) &_lrf.command = 0;
	_lrf.state.value.counter = _lrf.state.value.d[0] = _lrf.state.value.d[1] = _lrf.state.value.d[2] = 0;

	LOG("R created!\r\n");

	hal_bus_init();
	osDelay(1);

	/* Infinite loop */
	for ( ;; ) {
		/* get message from the queue */
		osEvent event = osMailGet(opt_get_bus_mail(), BUS_MAX_TIMEOUT);
		if (event.status == osEventMail) {
			/* create buffer pointer to hold queue value */
			Bus_Rx_Buffer_t *pRMail;
			pRMail = event.value.p;
			/* TODO recv panel command*/
			/* send new command to camera */
			if ((*(uint8_t*) &_cam.command != pRMail->data[4]) && (cam_is_busy() == 0)) {
				LOG("new cam cmd: 0x%X x 0x%X\r\n", pRMail->data[4], *(uint8_t* ) &_cam.command);

				if (g_camera_ready == 1) {
					osMessagePut(opt_get_queue(Q_CAMERA_NOTIF), pRMail->data[4], 0);
					*(uint8_t*) &_cam.command = pRMail->data[4];
				}
			}

			if ((*(uint8_t*) &_lrf.command != pRMail->data[5]) && (lrf_is_busy() == 0)) {
				LOG("new lrf cmd: 0x%X x 0x%X\r\n", pRMail->data[5], *(uint8_t* ) &_lrf.command);

				if (g_lrf_ready == 1) {
					osMessagePut(opt_get_queue(Q_LRF_NOTIF), pRMail->data[5], 0);
					*(uint8_t*) &_lrf.command = pRMail->data[5];
				}
			}

			/* free memory allocated for mail */
			osMailFree(opt_get_bus_mail(), pRMail);
			/* toggle led indicator */
			HAL_GPIO_TogglePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin);
		}
		else if (event.status == osEventTimeout)
			HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin, GPIO_PIN_SET);

		/* send notif to task manager that this thread is still running */
		osMessagePut(opt_get_queue(Q_MANAGER_NOTIF), managerNotif, 0);
	}
	/* USER CODE END t_bus_recv */
}

