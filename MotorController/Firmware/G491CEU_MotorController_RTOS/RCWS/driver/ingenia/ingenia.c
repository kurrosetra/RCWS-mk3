/*
 * ingenia.c
 *
 *  Created on: Jan 5, 2022
 *      Author: miftakur
 */
#include <string.h>
#include "ingenia.h"

#if INGENIA_USE_RTOS==1
#include "cmsis_os.h"
#endif	//if INGENIA_USE_RTOS==1

#if DEBUG_INGENIA_ENABLE==1
#include <stdio.h>

/*** Internal Const Values, Macros ***/
#	define LOG(str, ...) printf("[%ld %s:%d] " str, (HAL_GetTick()%10000UL), __FILE_NAME__, __LINE__, ##__VA_ARGS__)
#	define LOG_E(str, ...) printf("[%s Err:%d] " str, __FILE_NAME__, __LINE__, ##__VA_ARGS__)
#else
#	define LOG(str, ...)
#	define LOG_E(str, ...)

#endif	//if DEBUG_INGENIA_ENABLE==1

static SlaveNode_t slaveNodeBank;

/* init node slave buffer */
void Ingenia_node_init()
{
	slaveNodeBank.size = 0;
	for ( int i = 0; i < SERVO_NODE_SIZE_MAX; i++ ) {
		slaveNodeBank.node[i].id = 0;
		slaveNodeBank.node[i].servo = NULL;
	}
}

static HAL_StatusTypeDef Ingenia_node_add(Servo_t *servo)
{
	uint8_t currentBankSize = slaveNodeBank.size;

	if ((servo->_u8Node < SERVO_NODE_ID_MIN) || (servo->_u8Node > SERVO_NODE_ID_MAX))
		return HAL_ERROR;

	/* check if the node already exist in the banks */
	if (currentBankSize > 0) {
		for ( int i = 0; i < currentBankSize; i++ ) {
			if (slaveNodeBank.node[i].id == servo->_u8Node)
				return HAL_ERROR;
		}
	}

	/* check if the node buffer still has enough space */
	if (currentBankSize < SERVO_NODE_SIZE_MAX - 1) {
		slaveNodeBank.node[currentBankSize].id = servo->_u8Node;
		slaveNodeBank.node[currentBankSize].servo = servo;
		slaveNodeBank.node[currentBankSize].servo->_isRegistered = 1;
		slaveNodeBank.size++;
	}
	else
		return HAL_ERROR;

	return HAL_OK;
}

HAL_StatusTypeDef Ingenia_node_remove(Servo_t *servo)
{

	/* check if there is no data in buffer */
	if (slaveNodeBank.size == 0)
		return HAL_ERROR;

	/* check if the node already exist in the banks */
	for ( int i = 0; i < slaveNodeBank.size; i++ ) {
		if (slaveNodeBank.node[i].id == servo->_u8Node) {
			slaveNodeBank.node[i].id = 0;
			slaveNodeBank.node[i].servo = NULL;

			/* left shift node buffer */
			if (i < slaveNodeBank.size - 1) {
				for ( int j = i; j < slaveNodeBank.size - 1; j++ ) {
					slaveNodeBank.node[j].id = slaveNodeBank.node[j + 1].id;
					slaveNodeBank.node[j].servo = slaveNodeBank.node[j + 1].servo;
				}
			}
			/* delete node @ end buffer */
			slaveNodeBank.node[slaveNodeBank.size - 1].id = 0;
			slaveNodeBank.node[slaveNodeBank.size - 1].servo = NULL;
			slaveNodeBank.size--;

			return HAL_OK;
		}
	}

	return HAL_ERROR;
}

__weak HAL_StatusTypeDef Ingenia_begin(CAN_INTERFACE_HANDLETYPE *hcan)
{
	UNUSED(hcan);

	return HAL_ERROR;
}

__weak void Ingenia_prepare_tx_header(Servo_t *servo)
{
	UNUSED(servo);
}

__weak HAL_StatusTypeDef _INGENIA_write_data(Servo_t *servo, const uint32_t id, const uint8_t len, const uint8_t *buf)
{
	return HAL_ERROR;
}

__weak HAL_StatusTypeDef _INGENIA_get_data(CAN_INTERFACE_HANDLETYPE *hcan, const uint32_t rxLocation, uint32_t *_id_recv,
		uint8_t *buf)
{

	return HAL_ERROR;
}

void Ingenia_rx_callback(CAN_Data_t *data)
{
	uint32_t id;
	uint32_t idType;

	id = data->id % 0x80;
	idType = data->id - id;
//	LOG("%03lX\r\n", data->id);

	/* check if the node already exist in the banks */
	for ( int i = 0; i < slaveNodeBank.size; i++ ) {
		if (slaveNodeBank.node[i].id == id) {
			if (idType == COB_EMERGENCY) {
				can_buffer_write(&slaveNodeBank.node[i].servo->buffer.bufEMER, data);
				Ingenia_emer_callback(&slaveNodeBank.node[i].servo->buffer.bufEMER);
			}
			else if (idType == COB_NMT_CTRL) {
				can_buffer_write(&slaveNodeBank.node[i].servo->buffer.bufNMT, data);
				Ingenia_nmt_callback(&slaveNodeBank.node[i].servo->buffer.bufNMT);
			}
			else if (idType == COB_TSDO) {
				can_buffer_write(&slaveNodeBank.node[i].servo->buffer.bufTSDO, data);
				Ingenia_tsdo_callback(&slaveNodeBank.node[i].servo->buffer.bufTSDO);
			}
			else if ((idType == COB_TPDO1) || (idType == COB_TPDO2) || (idType == COB_TPDO3) || (idType == COB_TPDO4)) {
				can_buffer_write(&slaveNodeBank.node[i].servo->buffer.bufTPDO, data);
				Ingenia_tpdo_callback(&slaveNodeBank.node[i].servo->buffer.bufTPDO);
			}
		}
	}
}

HAL_StatusTypeDef Ingenia_init(Servo_t *servo, CAN_INTERFACE_HANDLETYPE *hcan, const uint8_t node)
{
	can_buffer_flush(&servo->buffer.bufEMER);
	can_buffer_flush(&servo->buffer.bufNMT);
	can_buffer_flush(&servo->buffer.bufTPDO);
	can_buffer_flush(&servo->buffer.bufTSDO);
	servo->hcan = hcan;
	servo->_u8Node = node;
	servo->_isRegistered = 0;
	servo->_isInitialAngleDeterminationProcessFinished = 0;
	servo->posActual = 0;
	servo->veloActual = 0;

	Ingenia_prepare_tx_header(servo);

#if INGENIA_USE_RTOS==1
	HAL_Delay(10);
	Ingenia_write_nmt(servo, NMT_RESET_COMM);
	HAL_Delay(500);
	Ingenia_write_nmt(servo, NMT_START_REMOTE_NODE);
	HAL_Delay(10);
#endif	//if INGENIA_USE_RTOS==1

	return Ingenia_node_add(servo);
}

HAL_StatusTypeDef Ingenia_buffer_available(CAN_Buffer_t *buf, CAN_Data_t *data)
{
	if (can_buffer_available(buf)) {
		can_buffer_read(buf, data);
		return HAL_OK;
	}
	return HAL_ERROR;
}

__weak void Ingenia_emer_callback(CAN_Buffer_t *buffer)
{
	UNUSED(buffer);
}

__weak void Ingenia_nmt_callback(CAN_Buffer_t *buffer)
{
	UNUSED(buffer);
}

__weak void Ingenia_tpdo_callback(CAN_Buffer_t *buffer)
{
	UNUSED(buffer);
}

__weak void Ingenia_tsdo_callback(CAN_Buffer_t *buffer)
{
	UNUSED(buffer);
}

uint32_t Ingenia_read_sdo(Servo_t *servo, uint16_t u16Index, uint8_t u8SubIndex, uint8_t *bIsValid)
{
	/* read register using SDO */
	uint8_t canTxBuffer[8];
	uint8_t isValid = 0;
	uint32_t millis = 0;
	uint32_t elapseTime;
	uint32_t sendTimer = 0;
	CAN_Data_t cData;
	uint16_t u16IndexFeed;
	uint8_t u8SubIndexFeed;
	uint32_t u32Result = 0;
	uint32_t _id_target = COB_RSDO + servo->_u8Node;
	uint8_t _len = 4;

	/* flush current buffer */
	can_buffer_flush(&servo->buffer.bufTSDO);
	memset(canTxBuffer, 0, 8);

	canTxBuffer[0] = 0x40;  //css[5..7]=2(upload), bit[0..4]=x
	canTxBuffer[1] = u16Index & 0xFF;
	canTxBuffer[2] = u16Index >> 8 & 0xFF;
	canTxBuffer[3] = u8SubIndex;

	elapseTime = HAL_GetTick() + 1000;
	while (!isValid) {
		millis = HAL_GetTick();
		/* check if timeout exceeded */
		if (millis >= elapseTime)
			break;

		if (millis >= sendTimer) {
			/* Start the Transmission process */
			_INGENIA_write_data(servo, _id_target, _len, canTxBuffer);
			/* retry send command */
			sendTimer = millis + 200;
		}

		if (can_buffer_available(&servo->buffer.bufTSDO) > 0) {
			if (can_buffer_read(&servo->buffer.bufTSDO, &cData) > 0) {
				/* check id sender (must be from CBO_TSDO & match index & sub-index */
				u16IndexFeed = (uint16_t) cData.rxData[2] << 8 | cData.rxData[1];
				u8SubIndexFeed = cData.rxData[3];

				if ((u16IndexFeed == u16Index) && (u8SubIndexFeed == u8SubIndex)) {
					u32Result = (uint32_t) cData.rxData[7] << 24 | (uint32_t) cData.rxData[6] << 16
							| (uint32_t) cData.rxData[5] << 8 | (uint32_t) cData.rxData[4];

					isValid = 1;
				}
			}
		}
#if INGENIA_USE_RTOS==1
		else
			osDelay(1);
#endif	//if INGENIA_USE_RTOS==1

	}

	*bIsValid = isValid;

	return u32Result;
}

uint32_t Ingenia_read_reg_sdo(Servo_t *servo, uint16_t u16Index, uint8_t u8SubIndex)
{

	uint32_t u32Result = 0;
	uint8_t isValid = 0;

	while (!isValid) {
		u32Result = Ingenia_read_sdo(servo, u16Index, u8SubIndex, &isValid);
	}

	return u32Result;
}

HAL_StatusTypeDef Ingenia_write_nmt(Servo_t *servo, NmtModes_e mode)
{
	uint8_t canTxBuffer[CAN_DATA_MAX];

	memset(canTxBuffer, 0, CAN_DATA_MAX);
	canTxBuffer[0] = mode;
	canTxBuffer[1] = servo->_u8Node;

	/* Start the Transmission process */
	return _INGENIA_write_data(servo, COB_NMT_SERVICE, 2, canTxBuffer);
}

void Ingenia_write_rpdo(Servo_t *servo, uint32_t cob_rpdo, uint8_t *data, uint8_t len)
{
	uint8_t canTxBuffer[8];
	uint32_t id_target = cob_rpdo | servo->_u8Node;

	memcpy(canTxBuffer, data, len);

	/* Start the Transmission process */
	uint32_t timeout = HAL_GetTick() + 1000;
	while (_INGENIA_write_data(servo, id_target, len, canTxBuffer) != HAL_OK) {
		if (HAL_GetTick() > timeout)
			break;
		HAL_Delay(1);
	}
}

HAL_StatusTypeDef Ingenia_write_sdo(Servo_t *servo, uint16_t u16Index, uint8_t u8SubIndex, uint8_t objSize, uint32_t value)
{
	uint8_t canTxBuffer[8];
	uint8_t sdoDataSize = 0;
	uint32_t elapseTime;
	uint8_t isValid = 0;
	CAN_Data_t dummy;
	uint32_t _id_target = COB_RSDO + servo->_u8Node;
	uint8_t _len = 8;

	memset(canTxBuffer, 0, CAN_DATA_MAX);
	switch (objSize)
	{
	case 1:
		sdoDataSize = SDO_DATA_SIZE_1;
		canTxBuffer[4] = value;
		break;
	case 2:
		sdoDataSize = SDO_DATA_SIZE_2;
		canTxBuffer[4] = value & 0xFF;
		canTxBuffer[5] = value >> 8 & 0xFF;
		break;
	case 4:
		sdoDataSize = SDO_DATA_SIZE_4;
		canTxBuffer[4] = value & 0xFF;
		canTxBuffer[5] = value >> 8 & 0xFF;
		canTxBuffer[6] = value >> 16 & 0xFF;
		canTxBuffer[7] = value >> 24 & 0xFF;
		break;
	}

	canTxBuffer[0] = SDO_DOWNLOAD_REQUEST_BITS | SDO_E_TRANSFER_BIT | SDO_S_SIZE_INDICATOR_BIT | sdoDataSize;

	canTxBuffer[1] = u16Index & 0xFF;
	canTxBuffer[2] = (u16Index >> 8) & 0xFF;
	canTxBuffer[3] = u8SubIndex;

	can_buffer_flush(&servo->buffer.bufTSDO);
	/* Start the Transmission process */
	if (_INGENIA_write_data(servo, _id_target, _len, canTxBuffer) == HAL_OK) {
		/* ignore reply */
		elapseTime = HAL_GetTick() + 1000;
		isValid = 0;
		while (!isValid) {
			if (HAL_GetTick() >= elapseTime)
				break;
			if (can_buffer_available(&servo->buffer.bufTSDO)) {
				can_buffer_read(&servo->buffer.bufTSDO, &dummy);
				if (((dummy.rxData[0] & SDO_DOWNLOAD_RESPONSE_BITS) == SDO_DOWNLOAD_RESPONSE_BITS)
						&& (dummy.rxData[1] == (u16Index & 0xFF)) && (dummy.rxData[2] == ((u16Index >> 8) & 0xFF))
						&& (dummy.rxData[3] == u8SubIndex)) {

					isValid = 1;
					break;
				}
			}
#if INGENIA_USE_RTOS==1
			osDelay(10);
//			osThreadYield();
#endif	//if INGENIA_USE_RTOS==1

		}
		if (isValid)
			return HAL_OK;
	}

	return HAL_ERROR;
}

HAL_StatusTypeDef Ingenia_write_sdo_u32(Servo_t *servo, uint16_t u16Index, uint8_t u8SubIndex, uint32_t value)
{
	return Ingenia_write_sdo(servo, u16Index, u8SubIndex, 4, value);
}

HAL_StatusTypeDef Ingenia_write_sdo_i32(Servo_t *servo, uint16_t u16Index, uint8_t u8SubIndex, int32_t value)
{
	return Ingenia_write_sdo(servo, u16Index, u8SubIndex, 4, value);
}

HAL_StatusTypeDef Ingenia_write_sdo_u16(Servo_t *servo, uint16_t u16Index, uint8_t u8SubIndex, uint16_t value)
{
	return Ingenia_write_sdo(servo, u16Index, u8SubIndex, 2, value);
}

HAL_StatusTypeDef Ingenia_write_sdo_i16(Servo_t *servo, uint16_t u16Index, uint8_t u8SubIndex, int16_t value)
{
	return Ingenia_write_sdo(servo, u16Index, u8SubIndex, 2, value);
}

HAL_StatusTypeDef Ingenia_write_sdo_u8(Servo_t *servo, uint16_t u16Index, uint8_t u8SubIndex, uint8_t value)
{
	return Ingenia_write_sdo(servo, u16Index, u8SubIndex, 1, value);
}

HAL_StatusTypeDef Ingenia_write_sdo_i8(Servo_t *servo, uint16_t u16Index, uint8_t u8SubIndex, int8_t value)
{
	return Ingenia_write_sdo(servo, u16Index, u8SubIndex, 1, value);
}

StateMachineStatus_e Ingenia_getDecodedStatusWord(Servo_t *servo)
{
	uint16_t u16StatusWord = servo->statusword;
	uint16_t u16LSBMask;
	StateMachineStatus_e eStateMachineStatus = STATUS_UNKNOWN;

	u16LSBMask = STATUS_WORD_REGISTER_BITS_READY_TO_SWITCH_ON | STATUS_WORD_REGISTER_BITS_SWITCHED_ON |
	STATUS_WORD_REGISTER_BITS_OPERATION_ENABLED | STATUS_WORD_REGISTER_BITS_FAULT;

// ----------------------------------------------   //
//      Value (binary)  |           Status          //
// ----------------------------------------------   //
// xxxx xxxx x0xx 0000  | Not ready to switch on    //
// xxxx xxxx x1xx 0000  | Switched on disabled      //
// xxxx xxxx x01x 0001  | Ready to switch on        //
// xxxx xxxx x01x 0011  | Switched on               //
// xxxx xxxx x01x 0111  | Operation enabled         //
// xxxx xxxx x00x 0111  | Quick stop active         //
// xxxx xxxx x0xx 1111  | Fault reaction active     //
// xxxx xxxx x0xx 1000  | Fault                     //
// ----------------------------------------------   //
//

	servo->_isInitialAngleDeterminationProcessFinished = ((u16StatusWord & STATUS_WORD_REGISTER_BITS_ANGLE_PROC_FINISHED)
			== STATUS_WORD_REGISTER_BITS_ANGLE_PROC_FINISHED);

	switch ((u16StatusWord & u16LSBMask))
	{
	case 0x00:
		if ((u16StatusWord & STATUS_WORD_REGISTER_BITS_SWITCH_ON_DISABLED) == 0x00) {
			eStateMachineStatus = STATUS_NOT_READY_TO_SWITCH_ON;
		}
		else {
			eStateMachineStatus = STATUS_SWITCH_ON_DISABLED;
		}
		break;

	case 0x01:
		if ((u16StatusWord & (STATUS_WORD_REGISTER_BITS_QUICK_STOP | STATUS_WORD_REGISTER_BITS_SWITCH_ON_DISABLED))
				== 0x20) {
			eStateMachineStatus = STATUS_READY_TO_SWITCH_ON;
		}
		break;

	case 0x03:
		if ((u16StatusWord & (STATUS_WORD_REGISTER_BITS_QUICK_STOP | STATUS_WORD_REGISTER_BITS_SWITCH_ON_DISABLED))
				== 0x20) {
			eStateMachineStatus = STATUS_SWITCH_ON;
		}
		break;

	case 0x07:
		if ((u16StatusWord & (STATUS_WORD_REGISTER_BITS_QUICK_STOP | STATUS_WORD_REGISTER_BITS_SWITCH_ON_DISABLED))
				== 0x20) {
			eStateMachineStatus = STATUS_OPERATION_ENABLED;
		}
		else if ((u16StatusWord & (STATUS_WORD_REGISTER_BITS_QUICK_STOP | STATUS_WORD_REGISTER_BITS_SWITCH_ON_DISABLED))
				== 0x00) {
			eStateMachineStatus = STATUS_QUICK_STOP_ACTIVE;
		}
		break;

	case 0x0F:
		if ((u16StatusWord & STATUS_WORD_REGISTER_BITS_SWITCH_ON_DISABLED) == 0x00) {
			eStateMachineStatus = STATUS_FAULT_REACTION_ACTIVE;
		}
		break;

	case 0x08:
		if ((u16StatusWord & STATUS_WORD_REGISTER_BITS_SWITCH_ON_DISABLED) == 0x00) {
			eStateMachineStatus = STATUS_FAULT;
		}
		break;

	default:
		eStateMachineStatus = STATUS_UNKNOWN;
		break;
	}

	return eStateMachineStatus;
}

static StateMachineStatus_e Ingenia_decodeStatusWord(Servo_t *servo, uint16_t u16StatusWord)
{
	uint16_t u16LSBMask;
	StateMachineStatus_e eStateMachineStatus = STATUS_UNKNOWN;

	u16LSBMask = STATUS_WORD_REGISTER_BITS_READY_TO_SWITCH_ON | STATUS_WORD_REGISTER_BITS_SWITCHED_ON |
	STATUS_WORD_REGISTER_BITS_OPERATION_ENABLED | STATUS_WORD_REGISTER_BITS_FAULT;

// ----------------------------------------------   //
//      Value (binary)  |           Status          //
// ----------------------------------------------   //
// xxxx xxxx x0xx 0000  | Not ready to switch on    //
// xxxx xxxx x1xx 0000  | Switched on disabled      //
// xxxx xxxx x01x 0001  | Ready to switch on        //
// xxxx xxxx x01x 0011  | Switched on               //
// xxxx xxxx x01x 0111  | Operation enabled         //
// xxxx xxxx x00x 0111  | Quick stop active         //
// xxxx xxxx x0xx 1111  | Fault reaction active     //
// xxxx xxxx x0xx 1000  | Fault                     //
// ----------------------------------------------   //
//

	servo->_isInitialAngleDeterminationProcessFinished = ((u16StatusWord & STATUS_WORD_REGISTER_BITS_ANGLE_PROC_FINISHED)
			== STATUS_WORD_REGISTER_BITS_ANGLE_PROC_FINISHED);

	switch ((u16StatusWord & u16LSBMask))
	{
	case 0x00:
		if ((u16StatusWord & STATUS_WORD_REGISTER_BITS_SWITCH_ON_DISABLED) == 0x00) {
			eStateMachineStatus = STATUS_NOT_READY_TO_SWITCH_ON;
		}
		else {
			eStateMachineStatus = STATUS_SWITCH_ON_DISABLED;
		}
		break;

	case 0x01:
		if ((u16StatusWord & (STATUS_WORD_REGISTER_BITS_QUICK_STOP | STATUS_WORD_REGISTER_BITS_SWITCH_ON_DISABLED))
				== 0x20) {
			eStateMachineStatus = STATUS_READY_TO_SWITCH_ON;
		}
		break;

	case 0x03:
		if ((u16StatusWord & (STATUS_WORD_REGISTER_BITS_QUICK_STOP | STATUS_WORD_REGISTER_BITS_SWITCH_ON_DISABLED))
				== 0x20) {
			eStateMachineStatus = STATUS_SWITCH_ON;
		}
		break;

	case 0x07:
		if ((u16StatusWord & (STATUS_WORD_REGISTER_BITS_QUICK_STOP | STATUS_WORD_REGISTER_BITS_SWITCH_ON_DISABLED))
				== 0x20) {
			eStateMachineStatus = STATUS_OPERATION_ENABLED;
		}
		else if ((u16StatusWord & (STATUS_WORD_REGISTER_BITS_QUICK_STOP | STATUS_WORD_REGISTER_BITS_SWITCH_ON_DISABLED))
				== 0x00) {
			eStateMachineStatus = STATUS_QUICK_STOP_ACTIVE;
		}
		break;

	case 0x0F:
		if ((u16StatusWord & STATUS_WORD_REGISTER_BITS_SWITCH_ON_DISABLED) == 0x00) {
			eStateMachineStatus = STATUS_FAULT_REACTION_ACTIVE;
		}
		break;

	case 0x08:
		if ((u16StatusWord & STATUS_WORD_REGISTER_BITS_SWITCH_ON_DISABLED) == 0x00) {
			eStateMachineStatus = STATUS_FAULT;
		}
		break;

	default:
		eStateMachineStatus = STATUS_UNKNOWN;
		break;
	}

	return eStateMachineStatus;
}

static void Ingenia_getStateMachineStatus(Servo_t *servo, StateMachineStatus_e *peStateMachineStatus)
{
	uint16_t u16StatusWord = Ingenia_read_reg_sdo(servo, OBJECT_STATUS_WORD);

	*peStateMachineStatus = Ingenia_decodeStatusWord(servo, u16StatusWord);
}

static void Ingenia_sendStateMachineCommand(Servo_t *servo, const StateMachineCommand_e eStateMachineCommand)
{
	uint8_t isValidCommand = 1;
	uint8_t needsTransition = 0;
	uint16_t u16TransitionAuxValue = 0;
	uint16_t u16ActualControlWord = 0;

	u16ActualControlWord = Ingenia_read_reg_sdo(servo, OBJECT_CONTROL_WORD);
	switch (eStateMachineCommand)
	{
	case COMMAND_SHUTDOWN:
		u16ActualControlWord &= ~(CONTROL_WORD_REGISTER_BITS_SWITCH_ON | CONTROL_WORD_REGISTER_BITS_FAULT_RESET);
		u16ActualControlWord |= (CONTROL_WORD_REGISTER_BITS_ENABLE_VOLTAGE | CONTROL_WORD_REGISTER_BITS_QUICK_STOP);
		break;

	case COMMAND_SWITCH_ON:
		u16ActualControlWord &= ~(CONTROL_WORD_REGISTER_BITS_ENABLE_OPERATION | CONTROL_WORD_REGISTER_BITS_FAULT_RESET);
		u16ActualControlWord |= (CONTROL_WORD_REGISTER_BITS_SWITCH_ON | CONTROL_WORD_REGISTER_BITS_ENABLE_VOLTAGE
				| CONTROL_WORD_REGISTER_BITS_QUICK_STOP);
		break;

	case COMMAND_SWITCH_ON_AND_ENABLE_OPERATION:
		u16ActualControlWord &= ~(CONTROL_WORD_REGISTER_BITS_FAULT_RESET);
		u16ActualControlWord |= (CONTROL_WORD_REGISTER_BITS_SWITCH_ON | CONTROL_WORD_REGISTER_BITS_ENABLE_VOLTAGE
				| CONTROL_WORD_REGISTER_BITS_QUICK_STOP | CONTROL_WORD_REGISTER_BITS_ENABLE_OPERATION);
		break;

	case COMMAND_DISABLE_VOLTAGE:
		u16ActualControlWord &= ~(CONTROL_WORD_REGISTER_BITS_ENABLE_VOLTAGE | CONTROL_WORD_REGISTER_BITS_FAULT_RESET);
		break;

	case COMMAND_QUICK_STOP:
		u16ActualControlWord &= ~(CONTROL_WORD_REGISTER_BITS_QUICK_STOP | CONTROL_WORD_REGISTER_BITS_FAULT_RESET);
		u16ActualControlWord |= CONTROL_WORD_REGISTER_BITS_ENABLE_VOLTAGE;
		break;

	case COMMAND_DISABLE_OPERATION:
		u16ActualControlWord &= ~(CONTROL_WORD_REGISTER_BITS_ENABLE_OPERATION | CONTROL_WORD_REGISTER_BITS_FAULT_RESET);
		u16ActualControlWord |= (CONTROL_WORD_REGISTER_BITS_SWITCH_ON | CONTROL_WORD_REGISTER_BITS_ENABLE_VOLTAGE
				| CONTROL_WORD_REGISTER_BITS_QUICK_STOP);
		break;

	case COMMAND_ENABLE_OPERATION:
		u16ActualControlWord &= ~(CONTROL_WORD_REGISTER_BITS_FAULT_RESET);
		u16ActualControlWord |= (CONTROL_WORD_REGISTER_BITS_SWITCH_ON | CONTROL_WORD_REGISTER_BITS_ENABLE_VOLTAGE
				| CONTROL_WORD_REGISTER_BITS_QUICK_STOP | CONTROL_WORD_REGISTER_BITS_ENABLE_OPERATION);
		break;

	case COMMAND_FAULT_RESET:
		if (u16ActualControlWord & CONTROL_WORD_REGISTER_BITS_FAULT_RESET) {
			u16ActualControlWord &= ~CONTROL_WORD_REGISTER_BITS_FAULT_RESET;
			needsTransition = 1;
			u16TransitionAuxValue = CONTROL_WORD_REGISTER_BITS_FAULT_RESET;
		}
		else {
			u16ActualControlWord |= CONTROL_WORD_REGISTER_BITS_FAULT_RESET;
		}
		break;

	default:
		isValidCommand = 0;
		break;
	}

	if (isValidCommand == 1) {
		Ingenia_write_sdo_u16(servo, OBJECT_CONTROL_WORD, u16ActualControlWord);
		if (needsTransition == 1) {
			u16ActualControlWord |= u16TransitionAuxValue;
			Ingenia_write_sdo_u16(servo, OBJECT_CONTROL_WORD, u16ActualControlWord);
		}
	}
}

static uint8_t Ingenia_verifyStatusIsReached(Servo_t *servo, const StateMachineStatus_e eNextStateMachineStatus)
{
	StateMachineStatus_e eActualStateMachineStatus;
	Ingenia_getStateMachineStatus(servo, &eActualStateMachineStatus);

	uint32_t started_waiting_at = HAL_GetTick();
	uint32_t millis = 0;
	while (eNextStateMachineStatus != eActualStateMachineStatus) {
		millis = HAL_GetTick();
		// Check if timeout excedeed
		if (millis - started_waiting_at > 1000)
			return 0;
		Ingenia_getStateMachineStatus(servo, &eActualStateMachineStatus);
	}
	return 1;

}

void Ingenia_gotoStatus(Servo_t *servo, const StateMachineStatus_e eDestinationStateMachineStatus)
{
	StateMachineStatus_e tCurrentStateMachineStatus = STATUS_NOT_READY_TO_SWITCH_ON;
	Ingenia_getStateMachineStatus(servo, &tCurrentStateMachineStatus);

	while (tCurrentStateMachineStatus != eDestinationStateMachineStatus) {
		StateMachineStatus_e nextState = eDestinationStateMachineStatus;
		switch (tCurrentStateMachineStatus)
		{
		case STATUS_SWITCH_ON_DISABLED:
			if ((eDestinationStateMachineStatus == STATUS_READY_TO_SWITCH_ON)
					|| (eDestinationStateMachineStatus == STATUS_SWITCH_ON)
					|| (eDestinationStateMachineStatus == STATUS_OPERATION_ENABLED)
					|| (eDestinationStateMachineStatus == STATUS_QUICK_STOP_ACTIVE)) {
				Ingenia_sendStateMachineCommand(servo, COMMAND_SHUTDOWN);
				nextState = STATUS_READY_TO_SWITCH_ON;
			}
			else {
				return;
			}
			break;
		case STATUS_READY_TO_SWITCH_ON:
			if (eDestinationStateMachineStatus == STATUS_SWITCH_ON_DISABLED) {
				Ingenia_sendStateMachineCommand(servo, COMMAND_DISABLE_VOLTAGE);
				nextState = STATUS_SWITCH_ON_DISABLED;
			}
			else if (eDestinationStateMachineStatus == STATUS_SWITCH_ON) {
				Ingenia_sendStateMachineCommand(servo, COMMAND_SWITCH_ON);
				nextState = STATUS_SWITCH_ON;
			}
			else if ((eDestinationStateMachineStatus == STATUS_OPERATION_ENABLED)
					|| (eDestinationStateMachineStatus == STATUS_QUICK_STOP_ACTIVE)) {
				Ingenia_sendStateMachineCommand(servo, COMMAND_SWITCH_ON_AND_ENABLE_OPERATION);
				nextState = STATUS_OPERATION_ENABLED;
			}
			else {
				return;
			}
			break;
		case STATUS_SWITCH_ON:
			if (eDestinationStateMachineStatus == STATUS_SWITCH_ON_DISABLED) {
				Ingenia_sendStateMachineCommand(servo, COMMAND_DISABLE_VOLTAGE);
				nextState = STATUS_SWITCH_ON_DISABLED;
			}
			else if (eDestinationStateMachineStatus == STATUS_READY_TO_SWITCH_ON) {
				Ingenia_sendStateMachineCommand(servo, COMMAND_SHUTDOWN);
				nextState = STATUS_READY_TO_SWITCH_ON;
			}
			else if ((eDestinationStateMachineStatus == STATUS_OPERATION_ENABLED)
					|| (eDestinationStateMachineStatus == STATUS_QUICK_STOP_ACTIVE)) {
				Ingenia_sendStateMachineCommand(servo, COMMAND_ENABLE_OPERATION);
				nextState = STATUS_OPERATION_ENABLED;
			}
			else {
				return;
			}
			break;
		case STATUS_OPERATION_ENABLED:
			if (eDestinationStateMachineStatus == STATUS_SWITCH_ON_DISABLED) {
				Ingenia_sendStateMachineCommand(servo, COMMAND_DISABLE_VOLTAGE);
				nextState = STATUS_SWITCH_ON_DISABLED;
			}
			else if (eDestinationStateMachineStatus == STATUS_READY_TO_SWITCH_ON) {
				Ingenia_sendStateMachineCommand(servo, COMMAND_SHUTDOWN);
				nextState = STATUS_READY_TO_SWITCH_ON;
			}
			else if (eDestinationStateMachineStatus == STATUS_SWITCH_ON) {
				Ingenia_sendStateMachineCommand(servo, COMMAND_DISABLE_OPERATION);
				nextState = STATUS_SWITCH_ON;
			}
			else if (eDestinationStateMachineStatus == STATUS_QUICK_STOP_ACTIVE) {
				Ingenia_sendStateMachineCommand(servo, COMMAND_QUICK_STOP);
				nextState = STATUS_QUICK_STOP_ACTIVE;
			}
			else {
				return;
			}
			break;
		case STATUS_QUICK_STOP_ACTIVE:
			if ((eDestinationStateMachineStatus == STATUS_SWITCH_ON_DISABLED)
					|| (eDestinationStateMachineStatus == STATUS_READY_TO_SWITCH_ON)
					|| (eDestinationStateMachineStatus == STATUS_SWITCH_ON)
					|| (eDestinationStateMachineStatus == STATUS_OPERATION_ENABLED)) {
				Ingenia_sendStateMachineCommand(servo, COMMAND_DISABLE_VOLTAGE);
				nextState = STATUS_SWITCH_ON_DISABLED;
			}
			else {
				return;
			}
			break;
		case STATUS_FAULT:
			if ((eDestinationStateMachineStatus == STATUS_SWITCH_ON_DISABLED)
					|| (eDestinationStateMachineStatus == STATUS_READY_TO_SWITCH_ON)
					|| (eDestinationStateMachineStatus == STATUS_SWITCH_ON)
					|| (eDestinationStateMachineStatus == STATUS_OPERATION_ENABLED)
					|| (eDestinationStateMachineStatus == STATUS_QUICK_STOP_ACTIVE)) {
				Ingenia_sendStateMachineCommand(servo, COMMAND_FAULT_RESET);
				nextState = STATUS_SWITCH_ON_DISABLED;
			}
			else {
				return;
			}
			break;
		default:
		{
			return;
		}
			break;
		}

		Ingenia_verifyStatusIsReached(servo, nextState);
		tCurrentStateMachineStatus = nextState;
	}
}

/* Motor enable/disable functions */
void Ingenia_enableMotor(Servo_t *servo)
{
	LOG("servo node=0x%02X\r\n", servo->_u8Node);
	Ingenia_gotoStatus(servo, STATUS_OPERATION_ENABLED);
}

void Ingenia_disableMotor(Servo_t *servo)
{
	StateMachineStatus_e currentStateMachineStatus;
	Ingenia_getStateMachineStatus(servo, &currentStateMachineStatus);

	if (currentStateMachineStatus == STATUS_OPERATION_ENABLED) {
		Ingenia_gotoStatus(servo, STATUS_SWITCH_ON);
	}

}

// Homing
void Ingenia_doHoming(Servo_t *servo, int8_t i8HomingMethod)
{
	uint16_t controlWord = 0;

	Ingenia_setModeOfOperation(servo, DRIVE_MODE_HOMING);
	Ingenia_write_sdo_i8(servo, 0x6098, 0, i8HomingMethod);

	Ingenia_enableMotor(servo);

	controlWord = Ingenia_read_reg_sdo(servo, OBJECT_CONTROL_WORD);
	controlWord &= ~0x0010;
	Ingenia_write_sdo_u16(servo, OBJECT_CONTROL_WORD, controlWord);
	controlWord = Ingenia_read_reg_sdo(servo, OBJECT_CONTROL_WORD);
	controlWord |= 0x0010;
	Ingenia_write_sdo_u16(servo, OBJECT_CONTROL_WORD, controlWord);
}

#define MODES_OF_OPERATION_INDEX				0x6060
#define MODES_OF_OPERATION_SUBINDEX				0x00
#define MODE_SPECIFIC_BITS_NEW_SETPOINT   		0x0010
#define MODE_SPECIFIC_BITS_CHANGE_SET			0x0020
#define MODE_SPECIFIC_BITS_ABS_REL				0x0040

// Motion funcions
void Ingenia_setModeOfOperation(Servo_t *servo, DriveModes_e driverMode)
{
	Ingenia_write_sdo_u8(servo, MODES_OF_OPERATION_INDEX, MODES_OF_OPERATION_SUBINDEX, (uint8_t) driverMode);
}

#define TARGET_POSITION_INDEX       0x607A
#define TARGET_POSITION_SUBINDEX    0x0
#define VELOCITY_INDEX              0x60FF
#define VELOCITY_SUBINDEX           0x0
#define TORQUE_INDEX                0x6071
#define TORQUE_SUBINDEX             0x0

void Ingenia_setTargetVelocity(Servo_t *servo, int32_t value)
{
	Ingenia_write_sdo_i32(servo, VELOCITY_INDEX, VELOCITY_SUBINDEX, value);
}

void Ingenia_setTargetTorque(Servo_t *servo, int16_t value)
{
	Ingenia_write_sdo_i16(servo, TORQUE_INDEX, TORQUE_SUBINDEX, value);
}

void Ingenia_setTargetPosition(Servo_t *servo, int32_t value)
{
	Ingenia_setTargetPositionAdv(servo, value, 1, 0, 0);
}

void Ingenia_setTargetPositionAdv(Servo_t *servo, int32_t value, uint8_t isImmediate, uint8_t isRelative,
		uint8_t isHaltEnabled)
{
	uint16_t u16ActualControlWord = 0;

	Ingenia_write_sdo_i32(servo, TARGET_POSITION_INDEX, TARGET_POSITION_SUBINDEX, value);

	u16ActualControlWord = Ingenia_read_reg_sdo(servo, OBJECT_CONTROL_WORD);

	/* Low new setpoint flag if needed */
	if ((u16ActualControlWord & MODE_SPECIFIC_BITS_NEW_SETPOINT) > 0) {
		u16ActualControlWord &= ~(MODE_SPECIFIC_BITS_NEW_SETPOINT);
		Ingenia_write_sdo_u16(servo, OBJECT_CONTROL_WORD, u16ActualControlWord);
	}

	/* Raise New Setpoint Bit */
	u16ActualControlWord |= 1 << 4;

	u16ActualControlWord &= ~(MODE_SPECIFIC_BITS_CHANGE_SET);
	u16ActualControlWord |= (uint16_t) isImmediate << 5;

	u16ActualControlWord &= ~(MODE_SPECIFIC_BITS_ABS_REL);
	u16ActualControlWord |= (uint16_t) isRelative << 6;

	u16ActualControlWord &= ~(CONTROL_WORD_REGISTER_BITS_HALT);
	u16ActualControlWord |= (uint16_t) isHaltEnabled << 8;

	Ingenia_write_sdo_u16(servo, OBJECT_CONTROL_WORD, u16ActualControlWord);

	/* Low New Setpoint */
	u16ActualControlWord &= ~(MODE_SPECIFIC_BITS_NEW_SETPOINT);
	Ingenia_write_sdo_u16(servo, OBJECT_CONTROL_WORD, u16ActualControlWord);
}

typedef union
{
	float f;
	uint32_t u32;
	int32_t i32;
	uint8_t b[4];
} Ingenia_Union_u;

void Ingenia_setTargetPositionVelocity(Servo_t *servo, int32_t pos, uint32_t velo, uint8_t isImmediate, uint8_t isRelative,
		uint8_t isHaltEnabled)
{
	uint16_t u16ActualControlWord = 0xF;
	uint8_t rpdo_value[8];
	Ingenia_Union_u p, v;

	/* set pos & velo */
	memset(rpdo_value, 0, 8);
	v.u32 = velo;
	p.i32 = pos;
	for ( int i = 0; i < 4; i++ ) {
		rpdo_value[i] = v.b[i];
		rpdo_value[i + 4] = p.b[i];
	}
	Ingenia_write_rpdo(servo, COB_RPDO2, rpdo_value, 8);

	/* set low setpoint */
	rpdo_value[0] = u16ActualControlWord;
	rpdo_value[1] = 0;
	Ingenia_write_rpdo(servo, COB_RPDO1, rpdo_value, 2);

	/* Raise New Setpoint Bit */
	u16ActualControlWord |= 1 << 4;

	u16ActualControlWord &= ~(MODE_SPECIFIC_BITS_CHANGE_SET);
	u16ActualControlWord |= (uint16_t) isImmediate << 5;

	u16ActualControlWord &= ~(MODE_SPECIFIC_BITS_ABS_REL);
	u16ActualControlWord |= (uint16_t) isRelative << 6;

	u16ActualControlWord &= ~(CONTROL_WORD_REGISTER_BITS_HALT);
	u16ActualControlWord |= (uint16_t) isHaltEnabled << 8;

	rpdo_value[0] = u16ActualControlWord & 0xFF;
	rpdo_value[1] = (u16ActualControlWord >> 8) & 0xFF;

	Ingenia_write_rpdo(servo, COB_RPDO1, rpdo_value, 2);

	/* Low New Setpoint */
	u16ActualControlWord &= ~(MODE_SPECIFIC_BITS_NEW_SETPOINT);
	rpdo_value[0] = u16ActualControlWord & 0xFF;
	rpdo_value[1] = (u16ActualControlWord >> 8) & 0xFF;
	Ingenia_write_rpdo(servo, COB_RPDO1, rpdo_value, 2);
}

int32_t Ingenia_getActualPosition(Servo_t *servo)
{
	return Ingenia_read_reg_sdo(servo, POSITION_ACTUAL_INDEX, POSITION_ACTUAL_SUBINDEX);
}

int32_t Ingenia_getActualVelocity(Servo_t *servo)
{
	return Ingenia_read_reg_sdo(servo, VELOCITY_ACTUAL_INDEX, VELOCITY_ACTUAL_SUBINDEX);
}
int16_t Ingenia_getActualTorque(Servo_t *servo)
{
	return Ingenia_read_reg_sdo(servo, TORQUE_ACTUAL_INDEX, TORQUE_ACTUAL_SUBINDEX);
}

// Statusword functions
uint16_t Ingenia_getStatusword(Servo_t *servo)
{
	return Ingenia_read_reg_sdo(servo, OBJECT_STATUS_WORD);
}
uint8_t Ingenia_statuswordIsReadyToSwitchOn(Servo_t *servo)
{
	return (Ingenia_getStatusword(servo) & STATUS_WORD_REGISTER_BITS_READY_TO_SWITCH_ON) > 0;
}
uint8_t Ingenia_statuswordIsSwitchedOn(Servo_t *servo)
{
	return (Ingenia_getStatusword(servo) & STATUS_WORD_REGISTER_BITS_SWITCHED_ON) > 0;
}
uint8_t Ingenia_statuswordIsOperationEnabled(Servo_t *servo)
{
	return (Ingenia_getStatusword(servo) & STATUS_WORD_REGISTER_BITS_OPERATION_ENABLED) > 0;
}
uint8_t Ingenia_statuswordIsFault(Servo_t *servo)
{
	return (Ingenia_getStatusword(servo) & STATUS_WORD_REGISTER_BITS_FAULT) > 0;
}
uint8_t Ingenia_statuswordIsVoltageEnabled(Servo_t *servo)
{
	return (Ingenia_getStatusword(servo) & STATUS_WORD_REGISTER_BITS_VOLTAGE_ENABLED) > 0;
}
uint8_t Ingenia_statuswordIsQuickStop(Servo_t *servo)
{
	return (Ingenia_getStatusword(servo) & STATUS_WORD_REGISTER_BITS_QUICK_STOP) > 0;
}
uint8_t Ingenia_statuswordIsSwitchOnDisabled(Servo_t *servo)
{
	return (Ingenia_getStatusword(servo) & STATUS_WORD_REGISTER_BITS_SWITCH_ON_DISABLED) > 0;
}
uint8_t Ingenia_statuswordIsWarning(Servo_t *servo)
{
	return (Ingenia_getStatusword(servo) & STATUS_WORD_REGISTER_BITS_WARNING) > 0;
}
uint8_t Ingenia_statuswordIsRemote(Servo_t *servo)
{
	return (Ingenia_getStatusword(servo) & STATUS_WORD_REGISTER_BITS_REMOTE) > 0;
}
uint8_t Ingenia_statuswordIsTargetReached(Servo_t *servo)
{
	return (Ingenia_getStatusword(servo) & STATUS_WORD_REGISTER_BITS_TARGET_REACHED) > 0;
}
uint8_t Ingenia_statuswordIsInternalLimitActive(Servo_t *servo)
{
	return (Ingenia_getStatusword(servo) & STATUS_WORD_REGISTER_BITS_INTERNAL_LIMIT_ACTIVE) > 0;
}
uint8_t Ingenia_statuswordIsInitialAngleDeterminationProcessFinished(Servo_t *servo)
{
	return (Ingenia_getStatusword(servo) & STATUS_WORD_REGISTER_BITS_ANGLE_PROC_FINISHED) > 0;
}

// Homing status functions
uint8_t Ingenia_homingStatusIsInProgress(Servo_t *servo)
{
	uint16_t _statusWord = Ingenia_getStatusword(servo);

	return ((_statusWord & STATUS_WORD_REGISTER_BITS_TARGET_REACHED) == 0)
			&& ((_statusWord & STATUS_WORD_REGISTER_BITS_OPMODE_SPECIFIC_1) == 0)
			&& ((_statusWord & STATUS_WORD_REGISTER_BITS_OPMODE_SPECIFIC_2) == 0);
}
uint8_t Ingenia_homingStatusIsError(Servo_t *servo)
{
	return (Ingenia_getStatusword(servo) & STATUS_WORD_REGISTER_BITS_OPMODE_SPECIFIC_2) > 0;
}
uint8_t Ingenia_homingStatusIsSuccess(Servo_t *servo)
{
	uint16_t _statusWord = Ingenia_getStatusword(servo);

	return ((_statusWord & STATUS_WORD_REGISTER_BITS_TARGET_REACHED) > 0)
			&& ((_statusWord & STATUS_WORD_REGISTER_BITS_OPMODE_SPECIFIC_1) > 0)
			&& ((_statusWord & STATUS_WORD_REGISTER_BITS_OPMODE_SPECIFIC_2) == 0);
}
uint8_t Ingenia_homingStatusIsAttained(Servo_t *servo)
{
	uint16_t _statusWord = Ingenia_getStatusword(servo);

	return ((_statusWord & STATUS_WORD_REGISTER_BITS_TARGET_REACHED) == 0)
			&& ((_statusWord & STATUS_WORD_REGISTER_BITS_OPMODE_SPECIFIC_1) > 0)
			&& ((_statusWord & STATUS_WORD_REGISTER_BITS_OPMODE_SPECIFIC_2) == 0);
}
uint8_t Ingenia_homingStatusIsInterrupted(Servo_t *servo)
{
	uint16_t _statusWord = Ingenia_getStatusword(servo);

	return ((_statusWord & STATUS_WORD_REGISTER_BITS_TARGET_REACHED) > 0)
			&& ((_statusWord & STATUS_WORD_REGISTER_BITS_OPMODE_SPECIFIC_1) == 0)
			&& ((_statusWord & STATUS_WORD_REGISTER_BITS_OPMODE_SPECIFIC_2) == 0);
}
uint8_t Ingenia_homingStatusNotStarted(Servo_t *servo)
{
	return Ingenia_homingStatusIsInterrupted(servo);
}
