/*
 * ingenia_conf.h
 *
 *  Created on: Jan 5, 2022
 *      Author: miftakur
 */

#ifndef DRIVER_INGENIA_INGENIA_CONF_H_
#define DRIVER_INGENIA_INGENIA_CONF_H_

#define DEBUG_INGENIA_ENABLE			1

#define INGENIA_USE_RTOS				0
#define INGENIA_USE_WDT					0

#define CAN_RX_BUFFER_SIZE				10
#define CAN_DATA_MAX					8
#define SERVO_NODE_SIZE_MAX				4
#define SERVO_NODE_ID_MIN				1
#define SERVO_NODE_ID_MAX				127

#define INGENIA_USE_FDCAN				1
#define INGENIA_USE_CAN					2

#define INGENIA_BUS_USED				INGENIA_USE_FDCAN

#if INGENIA_BUS_USED==INGENIA_USE_FDCAN
#	define CAN_INTERFACE_HANDLETYPE			FDCAN_HandleTypeDef
#	define CAN_IF_TxHeaderTypeDef			FDCAN_TxHeaderTypeDef
#	define CAN_IF_RX_FIFO0					FDCAN_RX_FIFO0
#	define CAN_IF_RX_FIFO1					FDCAN_RX_FIFO1
#endif	//if INGENIA_BUS_USED==INGENIA_USE_FDCAN

#if INGENIA_BUS_USED==INGENIA_USE_CAN
#	define CAN_INTERFACE_HANDLETYPE			CAN_HandleTypeDef
#	define CAN_IF_TxHeaderTypeDef			CAN_TxHeaderTypeDef
#	define CAN_IF_RxHeaderTypeDef			CAN_RxHeaderTypeDef
#	define CAN_IF_RX_FIFO0					CAN_RX_FIFO0
#	define CAN_IF_RX_FIFO1					CAN_RX_FIFO1
#endif	//if INGENIA_BUS_USED==INGENIA_USE_CAN

#endif /* DRIVER_INGENIA_INGENIA_CONF_H_ */
