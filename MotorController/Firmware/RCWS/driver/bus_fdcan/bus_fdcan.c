/*
 * bus.c
 *
 *  Created on: Jun 9, 2022
 *      Author: 62812
 */

#include "fdcan.h"

#include "app/common.h"
#include "bus_fdcan.h"

#if DEBUG_BUS==1
#include <stdio.h>

/*** Internal Const Values, Macros ***/
#	define LOG(str, ...) printf("[%ld %s:%d] " str, (osKernelSysTick()%10000UL),__FILE_NAME__, __LINE__, ##__VA_ARGS__)
#	define LOG_E(str, ...) printf("[%s_Err:%d] " str, __FILE_NAME__,__LINE__, ##__VA_ARGS__)
#else
#	define LOG(str, ...)
#	define LOG_E(str, ...)
#endif	//if DEBUG_BUS==1

const uint8_t dlc2len[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64 };

typedef struct
{
	FDCAN_HandleTypeDef *hfdcan;
	uint32_t index;
	uint32_t low_id;
	uint32_t high_id;
	uint32_t type;
} FDCAN_Rx_Filter_Params_t;

static void FDCAN_Config(FDCAN_HandleTypeDef *hfdcan);
static void __FDCAN_RX_Filter_Range(FDCAN_Rx_Filter_Params_t *params);
static void FDCAN_RX_Filter0_Range(FDCAN_HandleTypeDef *hfdcan, const uint32_t filterIndex, const uint32_t filterId1,
		const uint32_t filterId2);
void FDCAN_RX_Filter1_Range(FDCAN_HandleTypeDef *hfdcan, const uint32_t filterIndex, const uint32_t filterId1,
		const uint32_t filterId2);

static void FDCAN_TX_Config(FDCAN_TxHeaderTypeDef *txHeader, const uint32_t txID, const uint32_t dataLength);

FDCAN_TxHeaderTypeDef bus_tx_motor_status;
FDCAN_TxHeaderTypeDef bus_tx_weapon_status;
FDCAN_TxHeaderTypeDef bus_tx_position;
FDCAN_TxHeaderTypeDef bus_tx_imu;

FDCAN_HandleTypeDef *hfdcan = &hfdcan1;

void bus_init()
{
	FDCAN_RX_Filter0_Range(hfdcan, 0, RWS_PANEL_ID, (RWS_PANEL_ID + 0xF));
	FDCAN_Config(hfdcan);
	FDCAN_TX_Config(&bus_tx_motor_status, RWS_MOTOR_STATUS_ID, FDCAN_DLC_BYTES_5);
	FDCAN_TX_Config(&bus_tx_position, RWS_MOTOR_POS_ID, FDCAN_DLC_BYTES_8);
	FDCAN_TX_Config(&bus_tx_imu, RWS_MOTOR_IMU_ID, FDCAN_DLC_BYTES_8);
	FDCAN_TX_Config(&bus_tx_weapon_status, RWS_WEAPON_STATUS_ID, FDCAN_DLC_BYTES_3);
}

HAL_StatusTypeDef bus_send_motor_status(uint8_t *data)
{
	return HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &bus_tx_motor_status, data);
}

HAL_StatusTypeDef bus_send_weapon_status(uint8_t *data)
{
	return HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &bus_tx_weapon_status, data);
}

HAL_StatusTypeDef bus_send_position(const int32_t pan, const int32_t tilt)
{
	Rws_Union_u p, t;
	uint8_t data[8];

	p.i32 = pan;
	t.i32 = tilt;
	for ( int i = 0; i < 4; i++ ) {
		data[i] = p.u8[i];
		data[i + 4] = t.u8[i];
	}

	return HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &bus_tx_position, data);
}

HAL_StatusTypeDef bus_send_imu(const int16_t yaw, const int32_t pitch, const int32_t roll)
{

	return HAL_OK;
}

uint8_t FDCAN_Convert_Datalength(const uint32_t datalength)
{
	return dlc2len[datalength >> 16];
}

void bus_rx_callback(BUS_Rx_Buffer_t *buffer)
{
	if (buffer->id == RWS_PANEL_CMD_ID) {
		MAIL_Weapon_t *wpnMail;
		// allocate memory; receiver must be free it
		wpnMail = osMailAlloc(mtr_get_mail(Mail_Weapon_id), 0);
		wpnMail->sender_id = Weapon_Sender_Bus_id;
		*(uint8_t*) &wpnMail->param.command.mode = buffer->data[1];
		wpnMail->param.command.shoot_limit = ((uint16_t) buffer->data[2] << 8) | buffer->data[3];

		/* send mail queue*/
		osMailPut(mtr_get_mail(Mail_Weapon_id), wpnMail);

		/* if t_motor is ready */
		if (osMutexWait(mtr_get_mutex(Mutex_Motor_id), 0) == osOK) {
			LOG("send mtr_st\r\n");
			MAIL_Motor_t *mtrMail;
			/* allocate memory; receiver must be free it */
			mtrMail = osMailAlloc(mtr_get_mail(Mail_Motor_id), 0);
			mtrMail->sender_id = Motor_Sender_Bus_Mode_id;

			*(uint8_t*) &mtrMail->param.mode = buffer->data[0];

			/* send mail queue*/
			osMailPut(mtr_get_mail(Mail_Motor_id), mtrMail);

			osMutexRelease(mtr_get_mutex(Mutex_Motor_id));
		}
	}
	else if (buffer->id == RWS_PANEL_TRK_ID) {
		uint8_t trk_state;
		int16_t trk_x, trk_y;

		trk_state = buffer->data[0];
		trk_x = (int16_t) buffer->data[1] << 8 | buffer->data[2];
		trk_y = (int16_t) buffer->data[3] << 8 | buffer->data[4];

		/* if t_motor is ready */
		if (osMutexWait(mtr_get_mutex(Mutex_Motor_id), 0) == osOK) {
			osMutexRelease(mtr_get_mutex(Mutex_Motor_id));

			if (mtr_get_movement_mode() == MOVE_MODE_TRACK) {
				LOG("send trk_cmd\r\n");
				MAIL_Motor_Ext_t *mtrExtMail;
				/* allocate memory; receiver must be free it */
				mtrExtMail = osMailAlloc(mtr_get_mail(Mail_Motor_Ext_id), 0);

				mtrExtMail->sender_id = Motor_Ext_Sender_Value_id;
				*(uint8_t*) &mtrExtMail->param.track.cam_state = trk_state;
				mtrExtMail->param.track.trk_x = trk_x;
				mtrExtMail->param.track.trk_y = trk_y;

				/* send mail queue*/
				osMailPut(mtr_get_mail(Mail_Motor_Ext_id), mtrExtMail);
			}
		}
	}
	else if ((buffer->id == RWS_PANEL_MAN_ID) || (buffer->id == RWS_PANEL_BAL_ID) || (buffer->id == RWS_PANEL_HOM_ID)) {
		Rws_Union_u pan, tilt;

		for ( int i = 0; i < 4; i++ ) {
			pan.u8[i] = buffer->data[i];
			tilt.u8[i] = buffer->data[4 + i];
		}

		/* if t_motor is ready */
		if (osMutexWait(mtr_get_mutex(Mutex_Motor_id), 0) == osOK) {
			osMutexRelease(mtr_get_mutex(Mutex_Motor_id));
			LOG("send mtr_cmd\r\n");

			uint8_t motor_mode = mtr_get_movement_mode();
			if ((motor_mode == MOVE_MODE_MAN) || (motor_mode == MOVE_MODE_TRAVEL) || (motor_mode == MOVE_MODE_STAB)
					|| (motor_mode == MOVE_MODE_HOMING)) {
				MAIL_Motor_Ext_t *mtrExtMail;
				/* allocate memory; receiver must be free it */
				mtrExtMail = osMailAlloc(mtr_get_mail(Mail_Motor_Ext_id), 0);

				if ((motor_mode == MOVE_MODE_MAN) || (motor_mode == MOVE_MODE_HOMING))
					mtrExtMail->sender_id = Motor_Ext_Sender_Value_id;
				else if ((motor_mode == MOVE_MODE_TRAVEL) || (motor_mode == MOVE_MODE_STAB))
					mtrExtMail->sender_id = Motor_Ext_Sender_Offset_id;

				mtrExtMail->param.value.pan = pan.i32;
				mtrExtMail->param.value.tilt = tilt.i32;

				/* send mail queue*/
				osMailPut(mtr_get_mail(Mail_Motor_Ext_id), mtrExtMail);
			}
		}
	}
}

/**
 * @brief  Configures the FDCAN.
 * @param  None
 * @retval None
 */
static void FDCAN_Config(FDCAN_HandleTypeDef *hfdcan)
{
	/* Configure and enable Tx Delay Compensation, required for BRS mode.
	 TdcOffset default recommended value: DataTimeSeg1 * DataPrescaler
	 TdcFilter default recommended value: 0 */
	if (hfdcan->Init.FrameFormat == FDCAN_FRAME_FD_BRS) {
		/* Configure and enable Tx Delay Compensation, required for BRS mode.
		 TdcOffset default recommended value: DataTimeSeg1 * DataPrescaler
		 TdcFilter default recommended value: 0 */
		if (HAL_FDCAN_ConfigTxDelayCompensation(hfdcan, (hfdcan->Init.DataTimeSeg1 * hfdcan->Init.DataPrescaler), 0)
				!= HAL_OK) {
			Error_Handler();
		}
		if (HAL_FDCAN_EnableTxDelayCompensation(hfdcan) != HAL_OK) {
			Error_Handler();
		}
	}

	/* Activate Rx FIFO 0 new message notification */
	if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
		Error_Handler();
	}

//	/* Activate bus off notification */
//	if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_BUS_OFF, 0) != HAL_OK) {
//		Error_Handler();
//	}

	/* Start the FDCAN module */
	if (HAL_FDCAN_Start(hfdcan) != HAL_OK) {
		Error_Handler();
	}

}

static void __FDCAN_RX_Filter_Range(FDCAN_Rx_Filter_Params_t *params)
{
	FDCAN_FilterTypeDef sFilterConfig;

	/* Configure Rx filter */
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = params->index;
	sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
	sFilterConfig.FilterConfig = params->type;
	sFilterConfig.FilterID1 = params->low_id;
	sFilterConfig.FilterID2 = params->high_id;
	if (HAL_FDCAN_ConfigFilter(params->hfdcan, &sFilterConfig) != HAL_OK) {
		Error_Handler();
	}

	/* Configure global filter:
	 Filter all remote frames with STD and EXT ID
	 Reject non matching frames with STD ID and EXT ID */
	if (HAL_FDCAN_ConfigGlobalFilter(params->hfdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE,
	FDCAN_FILTER_REMOTE) != HAL_OK) {
		Error_Handler();
	}
}

static void FDCAN_RX_Filter0_Range(FDCAN_HandleTypeDef *hfdcan, const uint32_t filterIndex, const uint32_t filterId1,
		const uint32_t filterId2)
{
	FDCAN_Rx_Filter_Params_t params = { .hfdcan = hfdcan, .index = filterIndex, .type = FDCAN_FILTER_TO_RXFIFO0 };
	if (filterId1 > filterId2) {
		params.low_id = filterId2;
		params.high_id = filterId1;
	}
	else {
		params.low_id = filterId1;
		params.high_id = filterId2;
	}
	__FDCAN_RX_Filter_Range(&params);
}

void FDCAN_RX_Filter1_Range(FDCAN_HandleTypeDef *hfdcan, const uint32_t filterIndex, const uint32_t filterId1,
		const uint32_t filterId2)
{
	FDCAN_Rx_Filter_Params_t params = { .hfdcan = hfdcan, .index = filterIndex, .type = FDCAN_FILTER_TO_RXFIFO1 };
	if (filterId1 > filterId2) {
		params.low_id = filterId2;
		params.high_id = filterId1;
	}
	else {
		params.low_id = filterId1;
		params.high_id = filterId2;
	}
	__FDCAN_RX_Filter_Range(&params);
}

/**
 * @brief  Formatting Tx Header.
 * @param  hfdcan pointer to an FDCAN_HandleTypeDef structure that contains
 *         the configuration information for the specified FDCAN.
 * @param  txID indicates Tx Header ID.
 * @param  dataLength FDCAN Tx Data Length, default FDCAN_DLC_BYTES_8 8 Bytes to be send.
 *         This parameter can be any combination of @arg FDCAN_data_length_code.
 * @retval None
 */
static void FDCAN_TX_Config(FDCAN_TxHeaderTypeDef *txHeader, const uint32_t txID, const uint32_t dataLength)
{
	/* Prepare Tx Header */
	txHeader->Identifier = txID;
	txHeader->IdType = FDCAN_STANDARD_ID;
	txHeader->TxFrameType = FDCAN_DATA_FRAME;
	txHeader->DataLength = dataLength;
	txHeader->ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	txHeader->BitRateSwitch = FDCAN_BRS_OFF;
	txHeader->FDFormat = FDCAN_CLASSIC_CAN;
	txHeader->TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	txHeader->MessageMarker = 0;
}
