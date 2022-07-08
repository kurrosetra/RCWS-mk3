/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app/common.h"
#include "driver/bus_can/bus_can.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osMailQId mail_bus_recv;

/* USER CODE END Variables */
osThreadId TManagerHandle;
osThreadId TBusSendHandle;
osThreadId TBusRecvHandle;
osThreadId TCameraHandle;
osThreadId TLrfHandle;
osMessageQId q_cameraHandle;
osMessageQId q_lrfHandle;
osMessageQId q_managerHandle;
osTimerId cameraNotifTimerHandle;
osTimerId cameraTimeoutTimerHandle;
osTimerId lrfNotifTimerHandle;
osTimerId lrfTimeoutTimerHandle;
osMutexId m_camera_stateHandle;
osMutexId m_lrf_stateHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void mail_set_bus_recv();
/* USER CODE END FunctionPrototypes */

void t_manager(void const * argument);
extern void t_bus_send(void const * argument);
extern void t_bus_recv(void const * argument);
extern void t_camera(void const * argument);
extern void t_lrf(void const * argument);
extern void osTimerCameraNotifCallback(void const * argument);
extern void osTimerCameraTimeoutCallback(void const * argument);
extern void osTimerLrfNotifCallback(void const * argument);
extern void osTimerLrfTimeoutCallback(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* definition and creation of m_camera_state */
  osMutexDef(m_camera_state);
  m_camera_stateHandle = osMutexCreate(osMutex(m_camera_state));

  /* definition and creation of m_lrf_state */
  osMutexDef(m_lrf_state);
  m_lrf_stateHandle = osMutexCreate(osMutex(m_lrf_state));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of cameraNotifTimer */
  osTimerDef(cameraNotifTimer, osTimerCameraNotifCallback);
  cameraNotifTimerHandle = osTimerCreate(osTimer(cameraNotifTimer), osTimerPeriodic, NULL);

  /* definition and creation of cameraTimeoutTimer */
  osTimerDef(cameraTimeoutTimer, osTimerCameraTimeoutCallback);
  cameraTimeoutTimerHandle = osTimerCreate(osTimer(cameraTimeoutTimer), osTimerOnce, NULL);

  /* definition and creation of lrfNotifTimer */
  osTimerDef(lrfNotifTimer, osTimerLrfNotifCallback);
  lrfNotifTimerHandle = osTimerCreate(osTimer(lrfNotifTimer), osTimerPeriodic, NULL);

  /* definition and creation of lrfTimeoutTimer */
  osTimerDef(lrfTimeoutTimer, osTimerLrfTimeoutCallback);
  lrfTimeoutTimerHandle = osTimerCreate(osTimer(lrfTimeoutTimer), osTimerOnce, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of q_camera */
  osMessageQDef(q_camera, 6, uint8_t);
  q_cameraHandle = osMessageCreate(osMessageQ(q_camera), NULL);

  /* definition and creation of q_lrf */
  osMessageQDef(q_lrf, 6, uint8_t);
  q_lrfHandle = osMessageCreate(osMessageQ(q_lrf), NULL);

  /* definition and creation of q_manager */
  osMessageQDef(q_manager, 6, uint16_t);
  q_managerHandle = osMessageCreate(osMessageQ(q_manager), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  mail_set_bus_recv();

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of TManager */
  osThreadDef(TManager, t_manager, osPriorityHigh, 0, 256);
  TManagerHandle = osThreadCreate(osThread(TManager), NULL);

  /* definition and creation of TBusSend */
  osThreadDef(TBusSend, t_bus_send, osPriorityLow, 0, 256);
  TBusSendHandle = osThreadCreate(osThread(TBusSend), NULL);

  /* definition and creation of TBusRecv */
  osThreadDef(TBusRecv, t_bus_recv, osPriorityAboveNormal, 0, 512);
  TBusRecvHandle = osThreadCreate(osThread(TBusRecv), NULL);

  /* definition and creation of TCamera */
  osThreadDef(TCamera, t_camera, osPriorityBelowNormal, 0, 768);
  TCameraHandle = osThreadCreate(osThread(TCamera), NULL);

  /* definition and creation of TLrf */
  osThreadDef(TLrf, t_lrf, osPriorityNormal, 0, 512);
  TLrfHandle = osThreadCreate(osThread(TLrf), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_t_manager */
/**
  * @brief  Function implementing the TManager thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_t_manager */
__weak void t_manager(void const * argument)
{
  /* USER CODE BEGIN t_manager */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END t_manager */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
osTimerId opt_get_timer(const Timer_ID tID)
{
	switch (tID)
	{
	case Tim_Camera_Notif_id:
		return cameraNotifTimerHandle;
	case Tim_Camera_Timeout_id:
		return cameraTimeoutTimerHandle;
	case Tim_Lrf_Notif_id:
		return lrfNotifTimerHandle;
	case Tim_Lrf_Timeout_id:
		return lrfTimeoutTimerHandle;
	default:
		return NULL;
	}
}

osMessageQId opt_get_queue(const QUEUE_ID qID)
{
	switch (qID)
	{
	case Q_CAMERA_NOTIF:
		return q_cameraHandle;
	case Q_LRF_NOTIF:
		return q_lrfHandle;
	case Q_MANAGER_NOTIF:
		return q_managerHandle;
	default:
		return 0;
	}
}

osMutexId opt_get_mutex(const MUTEX_ID mID)
{
	switch (mID)
	{
	case M_CAMERA_STATE:
		return m_camera_stateHandle;
	case M_LRF_STATE:
		return m_lrf_stateHandle;
	default:
		return 0;
	}
}

osMailQId opt_get_bus_mail()
{
	return mail_bus_recv;
}

static void mail_set_bus_recv()
{
	/* Create the mail queue used by the bus_recv tasks to pass the struct Bus_Rx_Buffer_t */
	osMailQDef(mail, 5, Bus_Rx_Buffer_t); /* Define mail queue */
	mail_bus_recv = osMailCreate(osMailQ(mail), NULL); /* create mail queue */
}

void HAL_Delay(uint32_t Delay)
{
	osDelay(Delay);
}
/* USER CODE END Application */

