/*
 * ingenia.h
 *
 *  Created on: Jan 5, 2022
 *      Author: miftakur
 */

#ifndef DRIVER_INGENIA_INGENIA_H_
#define DRIVER_INGENIA_INGENIA_H_

#include "main.h"
#include "ingenia_conf.h"
#include "ingenia_def.h"
#include "ingenia_buffer.h"

typedef struct
{
	CAN_INTERFACE_HANDLETYPE *hcan;
	CAN_IF_TxHeaderTypeDef txHeader;
	struct
	{
		CAN_Buffer_t bufTSDO;
		CAN_Buffer_t bufNMT;
		CAN_Buffer_t bufTPDO;
		CAN_Buffer_t bufEMER;
	} buffer;
	uint8_t _isRegistered;
	uint8_t _u8Node;
	uint8_t _isInitialAngleDeterminationProcessFinished;
	uint8_t _isBusy;

	/* TPDO4 */
	int32_t posActual;
	int32_t veloActual;
	uint32_t tpdo4_counter;

	/* TPDO3 */
	int32_t voltage;
	int16_t current;
	uint16_t statusword;
	uint32_t tpdo3_counter;
} Servo_t;

//typedef struct
//{
//	uint8_t _id;
//	Servo_t *servo;
//} SlaveNodeId_t;

typedef struct
{
//	SlaveNodeId_t _node[SERVO_NODE_SIZE_MAX];
	struct
	{
		uint8_t id;
		Servo_t *servo;
	} node[SERVO_NODE_SIZE_MAX];
	uint8_t size;
} SlaveNode_t;

void Ingenia_node_init();
HAL_StatusTypeDef Ingenia_begin(CAN_INTERFACE_HANDLETYPE *hcan);
void Ingenia_prepare_tx_header(Servo_t *servo);
HAL_StatusTypeDef _INGENIA_write_data(Servo_t *servo, const uint32_t id, const uint8_t len, const uint8_t *buf);
HAL_StatusTypeDef _INGENIA_get_data(CAN_INTERFACE_HANDLETYPE *hcan, const uint32_t rxLocation, uint32_t *_id_recv,
		uint8_t *buf);
void Ingenia_rx_callback(CAN_Data_t *data);

HAL_StatusTypeDef Ingenia_init(Servo_t *servo, CAN_INTERFACE_HANDLETYPE *hcan, const uint8_t node);
HAL_StatusTypeDef Ingenia_node_remove(Servo_t *servo);
HAL_StatusTypeDef Ingenia_buffer_available(CAN_Buffer_t *buf, CAN_Data_t *data);
void Ingenia_emer_callback(CAN_Buffer_t *buffer);
void Ingenia_nmt_callback(CAN_Buffer_t *buffer);
void Ingenia_tpdo_callback(CAN_Buffer_t *buffer);
void Ingenia_tsdo_callback(CAN_Buffer_t *buffer);

uint32_t Ingenia_read_sdo(Servo_t *servo, uint16_t u16Index, uint8_t u8SubIndex, uint8_t *bIsValid);
uint32_t Ingenia_read_reg_sdo(Servo_t *servo, uint16_t u16Index, uint8_t u8SubIndex);

HAL_StatusTypeDef Ingenia_write_nmt(Servo_t *servo, NmtModes_e mode);
void Ingenia_write_rpdo(Servo_t *servo, uint32_t cob_rpdo, uint8_t *data, uint8_t len);

HAL_StatusTypeDef Ingenia_write_sdo(Servo_t *servo, uint16_t u16Index, uint8_t u8SubIndex, uint8_t objSize, uint32_t value);
HAL_StatusTypeDef Ingenia_write_sdo_u32(Servo_t *servo, uint16_t u16Index, uint8_t u8SubIndex, uint32_t value);
HAL_StatusTypeDef Ingenia_write_sdo_i32(Servo_t *servo, uint16_t u16Index, uint8_t u8SubIndex, int32_t value);
HAL_StatusTypeDef Ingenia_write_sdo_u16(Servo_t *servo, uint16_t u16Index, uint8_t u8SubIndex, uint16_t value);
HAL_StatusTypeDef Ingenia_write_sdo_i16(Servo_t *servo, uint16_t u16Index, uint8_t u8SubIndex, int16_t value);
HAL_StatusTypeDef Ingenia_write_sdo_u8(Servo_t *servo, uint16_t u16Index, uint8_t u8SubIndex, uint8_t value);
HAL_StatusTypeDef Ingenia_write_sdo_i8(Servo_t *servo, uint16_t u16Index, uint8_t u8SubIndex, int8_t value);

StateMachineStatus_e Ingenia_decodeStatusWord(Servo_t *servo, uint16_t u16StatusWord);
void Ingenia_gotoStatus(Servo_t *servo, const StateMachineStatus_e eDestinationStateMachineStatus);

// Motor enable/disable functions
void Ingenia_enableMotor(Servo_t *servo);
void Ingenia_disableMotor(Servo_t *servo);

// Homing
void Ingenia_doHoming(Servo_t *servo, int8_t i8HomingMethod);

// Motion funcions
void Ingenia_setModeOfOperation(Servo_t *servo, DriveModes_e driverMode);

void Ingenia_setTargetVelocity(Servo_t *servo, int32_t value);
void Ingenia_setTargetTorque(Servo_t *servo, int16_t value);
void Ingenia_setTargetPosition(Servo_t *servo, int32_t value);
void Ingenia_setTargetPositionAdv(Servo_t *servo, int32_t value, uint8_t isImmediate, uint8_t isRelative,
		uint8_t isHaltEnabled);
void Ingenia_setTargetPositionVelocity(Servo_t *servo, int32_t pos, uint32_t velo, uint8_t isImmediate, uint8_t isRelative,
		uint8_t isHaltEnabled);

int32_t Ingenia_getActualPosition(Servo_t *servo);
int32_t Ingenia_getActualVelocity(Servo_t *servo);
int16_t Ingenia_getActualTorque(Servo_t *servo);

// Statusword functions
uint16_t Ingenia_getStatusword(Servo_t *servo);
uint8_t Ingenia_statuswordIsReadyToSwitchOn(Servo_t *servo);
uint8_t Ingenia_statuswordIsSwitchedOn(Servo_t *servo);
uint8_t Ingenia_statuswordIsOperationEnabled(Servo_t *servo);
uint8_t Ingenia_statuswordIsFault(Servo_t *servo);
uint8_t Ingenia_statuswordIsVoltageEnabled(Servo_t *servo);
uint8_t Ingenia_statuswordIsQuickStop(Servo_t *servo);
uint8_t Ingenia_statuswordIsSwitchOnDisabled(Servo_t *servo);
uint8_t Ingenia_statuswordIsWarning(Servo_t *servo);
uint8_t Ingenia_statuswordIsRemote(Servo_t *servo);
uint8_t Ingenia_statuswordIsTargetReached(Servo_t *servo);
uint8_t Ingenia_statuswordIsInternalLimitActive(Servo_t *servo);
uint8_t Ingenia_statuswordIsInitialAngleDeterminationProcessFinished(Servo_t *servo);

// Homing status functions
uint8_t Ingenia_homingStatusIsInProgress(Servo_t *servo);
uint8_t Ingenia_homingStatusIsError(Servo_t *servo);
uint8_t Ingenia_homingStatusIsSuccess(Servo_t *servo);
uint8_t Ingenia_homingStatusIsAttained(Servo_t *servo);
uint8_t Ingenia_homingStatusIsInterrupted(Servo_t *servo);
uint8_t Ingenia_homingStatusNotStarted(Servo_t *servo);

#endif /* DRIVER_INGENIA_INGENIA_H_ */
