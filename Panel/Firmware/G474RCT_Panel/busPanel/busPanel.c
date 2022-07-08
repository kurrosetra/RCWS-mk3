/*
 * busPanel.c
 *
 *  Created on: Dec 13, 2021
 *      Author: miftakur
 */

#include <string.h>

#include "fdcan.h"
#include "busPanel.h"
#include "Button.h"
#include "pc.h"

#if BUS_DEBUG==1 || BUS_MOTOR_DEBUG==1
#include <stdio.h>

#include "usart.h"
#endif	//if BUS_DEBUG==1

static volatile uint8_t g_fdcan_bus_busOff_error = 0;
Bus_t bus;
static FDCAN_TxHeaderTypeDef txHeader;

void bus_init(FDCAN_HandleTypeDef *hfdcan)
{
	bus.hfdcan = hfdcan;

#if BUS_DEBUG==1
	char buf[RING_BUFFER_TX_SIZE];
	uint16_t bufLen;
	bufLen = sprintf(buf, "fdcan bus init ...\r\n");
	serial_write_str(&debug, buf, bufLen);
#endif	//if BUS_DEBUG==1

	FDCAN_RX_Filter_Motor(bus.hfdcan, 0);
	bus.recvMotor.id = RWS_MOTOR_ID;
	FDCAN_RX_Filter_Optronik(bus.hfdcan, 1);
	bus.recvOptronik.id = RWS_OPTRONIK_ID;
	FDCAN_RX_Filter_Imu(bus.hfdcan, 2);
	bus.recvImu.id = RWS_IMU_ID;
	FDCAN_TX_Config(&txHeader, RWS_PANEL_ID, RWS_PANEL_DATA_LENGTH);
	FDCAN_Config(bus.hfdcan);

#if BUS_DEBUG==1
	bufLen = sprintf(buf, "done!\r\n");
	serial_write_str(&debug, buf, bufLen);
#endif	//if BUS_DEBUG==1

	Panel_button_command_t *_pbc = &bus.panelButtonCommand;
	*(uint8_t*) &_pbc->weaponCommand = 0;
	*(uint8_t*) &_pbc->movementMode = 0;
	*(uint8_t*) &_pbc->cameraCommand = 0;
	*(uint8_t*) &_pbc->lrfImuCommand = 0;
	_pbc->panSpeedCommand = _pbc->panSpeedCorrection = 0;
	_pbc->tiltSpeedCommand = _pbc->tiltSpeedCorrection = 0;
}

static void bus_motor_handler()
{
	static uint8_t motor_counter = 0;
	static uint8_t opt_counter = 0;
	Rws_Union_u mtr[4];

#if BUS_MOTOR_DEBUG==1
	char buf[RING_BUFFER_TX_SIZE];
	uint16_t bufLen;
#endif	//if BUS_MOTOR_DEBUG==1

#if BUS_DEBUG==1
	char buf[RING_BUFFER_TX_SIZE];
	uint16_t bufLen;
	uint8_t motor_debug_on = 0;
	static uint32_t motor_debug_timer = 0;
	uint8_t opt_debug_on = 0;
	static uint32_t opt_debug_timer = 0;
#endif	//if BUS_DEBUG==1

	/* TODO timeout occurred*/
#if BUS_DEBUG==1
	if (HAL_GetTick() - bus.recvMotor.lastTimestamp > BUS_MAX_TIMEOUT)
		motor_debug_on = 0;
	if (HAL_GetTick() - bus.recvOptronik.lastTimestamp > BUS_MAX_TIMEOUT)
		opt_debug_on = 0;
#endif	//if BUS_DEBUG==1

	if (motor_counter != bus.recvMotor.counter) {
		motor_counter = bus.recvMotor.counter;

		*(uint8_t*) &bus.motorWeaponStatus.motorStatus = bus.recvMotor.data[0];
		*(uint8_t*) &bus.motorWeaponStatus.weaponStatus = bus.recvMotor.data[1];

		bus.motorWeaponStatus.munitionCounter = ((uint16_t) bus.recvMotor.data[2] << 8)
				| bus.recvMotor.data[3];
		comp.send.munCounter = bus.motorWeaponStatus.munitionCounter;

		for ( int _i = 0; _i < 4; _i++ ) {
			mtr[0].u8[_i] = bus.recvMotor.data[4 + _i];
			mtr[1].u8[_i] = bus.recvMotor.data[8 + _i];
			mtr[2].u8[_i] = bus.recvMotor.data[12 + _i];
			mtr[3].u8[_i] = bus.recvMotor.data[16 + _i];
		}
		bus.motorWeaponStatus.panPosition = mtr[0].i32;
		bus.motorWeaponStatus.tiltPosition = mtr[1].i32;
		bus.motorWeaponStatus.panVelocity = mtr[2].i32;
		bus.motorWeaponStatus.tiltVelocity = mtr[3].i32;

		bus.motorWeaponStatus.panVoltage = (uint16_t) bus.recvMotor.data[20] << 8
				| bus.recvMotor.data[21];
		bus.motorWeaponStatus.panCurrent = (int16_t) bus.recvMotor.data[22] << 8
				| bus.recvMotor.data[23];
		bus.motorWeaponStatus.tiltVoltage = (uint16_t) bus.recvMotor.data[24] << 8
				| bus.recvMotor.data[25];
		bus.motorWeaponStatus.tiltCurrent = (int16_t) bus.recvMotor.data[26] << 8
				| bus.recvMotor.data[27];

		tim4_out_toggle(LED_CH);
#if BUS_MOTOR_DEBUG==1
		bufLen = sprintf(buf, "%ld,%X,%ld,%ld,%ld,%d,%d,%ld,%ld,%ld,%d,%d\r\n", HAL_GetTick(),
				*(uint8_t*) &bus.motorWeaponStatus.motorStatus,
				bus.panelButtonCommand.panSpeedCommand, bus.motorWeaponStatus.panPosition,
				bus.motorWeaponStatus.panVelocity, bus.motorWeaponStatus.panVoltage,
				bus.motorWeaponStatus.panCurrent, bus.panelButtonCommand.tiltSpeedCommand,
				bus.motorWeaponStatus.tiltPosition, bus.motorWeaponStatus.tiltVelocity,
				bus.motorWeaponStatus.tiltVoltage, bus.motorWeaponStatus.tiltCurrent);

		serial_write_str(&debug, buf, bufLen);
#else
		motor_debug_on = 1;
#endif	//if BUS_MOTOR_DEBUG==1
	}

	if (opt_counter != bus.recvOptronik.counter) {
		opt_counter = bus.recvOptronik.counter;

		*(uint8_t*) &bus.optronikStatus.camera = bus.recvOptronik.data[0];
		bus.optronikStatus.lrf.counter = bus.recvOptronik.data[1];
		bus.optronikStatus.lrf.d[0] = (uint16_t) bus.recvOptronik.data[2] << 8
				| bus.recvOptronik.data[3];
		bus.optronikStatus.lrf.d[1] = (uint16_t) bus.recvOptronik.data[4] << 8
				| bus.recvOptronik.data[5];
		bus.optronikStatus.lrf.d[2] = (uint16_t) bus.recvOptronik.data[6] << 8
				| bus.recvOptronik.data[7];

		comp.send.lrfRange = bus.optronikStatus.lrf.d[0] / 2;
#if BUS_DEBUG==1
		opt_debug_on = 1;
#endif	//if BUS_DEBUG==1
	}

#if BUS_DEBUG==1
	if (motor_debug_on != 0) {
		if (HAL_GetTick() >= motor_debug_timer) {
			motor_debug_timer = HAL_GetTick() + 500;

#if BUS_DEBUG==1
			bufLen = sprintf(buf, "(JS)%d:(%ld=%ld):(%ld=%ld)\r\n", bus.sendPanel.data[0],
					button.JRight.az, bus.panelButtonCommand.panSpeedCommand, button.JRight.el,
					bus.panelButtonCommand.tiltSpeedCommand);
//			bufLen = sprintf(buf, "(PAN)P:V= %ld %ld\r\n", bus.motorWeaponStatus.panPosition,
//					bus.motorWeaponStatus.panVelocity);
//			bufLen = sprintf(buf, "(TILT)P:V= %ld %ld\r\n", bus.motorWeaponStatus.tiltPosition,
//					bus.motorWeaponStatus.tiltVelocity);
//			bufLen = sprintf(buf, "%X,%ld,%ld,%d,%d,%ld,%ld,%d,%d\r\n",
//					*(uint8_t*) &bus.motorWeaponStatus.motorStatus,
//					bus.motorWeaponStatus.panPosition, bus.motorWeaponStatus.panVelocity,
//					bus.motorWeaponStatus.panVoltage, bus.motorWeaponStatus.panCurrent,
//					bus.motorWeaponStatus.tiltPosition, bus.motorWeaponStatus.tiltVelocity,
//					bus.motorWeaponStatus.tiltVoltage, bus.motorWeaponStatus.tiltCurrent);

			serial_write_str(&debug, buf, bufLen);
#endif	//if BUS_DEBUG==1

		}
	}

	if (opt_debug_on != 0) {
		if (HAL_GetTick() >= opt_debug_timer) {
			opt_debug_timer = HAL_GetTick() + 500;

#if BUS_DEBUG==1
			bufLen = sprintf(buf, "[OPT]C:L= %X:%X-%d,%d,%d\r\n",
					*(uint8_t*) &bus.optronikStatus.camera, bus.optronikStatus.lrf.counter,
					bus.optronikStatus.lrf.d[0], bus.optronikStatus.lrf.d[1],
					bus.optronikStatus.lrf.d[2]);
			serial_write_str(&debug, buf, bufLen);
#endif	//if BUS_DEBUG==1

		}
	}
#endif	//if BUS_DEBUG==1
}

static void bus_optronik_handler()
{
	static uint8_t optronik_counter = 0;
	Optronik_Lrf_t *lrf = &bus.optronikStatus.lrf;

	/* TODO timeout occurred*/

	if (optronik_counter != bus.recvOptronik.counter) {
		optronik_counter = bus.recvOptronik.counter;

		*(uint8_t*) &bus.optronikStatus.camera = bus.recvOptronik.data[0];
		/* new lrf's value available */
		if (bus.recvOptronik.data[1] != lrf->counter) {
			for ( int i = 0; i < 3; i++ )
				lrf->d[i] = (uint16_t) bus.recvOptronik.data[2 + i] << 8
						| bus.recvOptronik.data[2 + i + 1];

			lrf->counter = bus.recvOptronik.data[1];
			/* TODO verify lrf result*/
			comp.send.lrfRange = lrf->d[0];
		}
	}
}

void bus_handler()
{
	static uint32_t _panelSendTimer = 0;
	static uint32_t _ledPowerTimer = 0;
	static uint8_t _ledPowerCounter = 0;
	uint32_t _mtrTimer = 0, _optTimer = 0;
	uint8_t _ledPowerState = 1;

	Rws_Union_u mtr[4];

	if (g_fdcan_bus_busOff_error != 0) {
		MX_FDCAN1_Init();
		bus_init(bus.hfdcan);

		g_fdcan_bus_busOff_error = 0;
	}

	bus_motor_handler();
	bus_optronik_handler();

	/* PANEL led power handler */
	if (HAL_GetTick() >= _ledPowerTimer) {
		_ledPowerTimer = HAL_GetTick() + 250;

		_mtrTimer = HAL_GetTick() - bus.recvMotor.lastTimestamp;
		_optTimer = HAL_GetTick() - bus.recvOptronik.lastTimestamp;

		if (_mtrTimer < BUS_MAX_TIMEOUT)
			_ledPowerState += 2;
		if (_optTimer < BUS_MAX_TIMEOUT)
			_ledPowerState += 1;

		if (_ledPowerCounter <= _ledPowerState)
			button.panelButton.output.power_state = 1;
		else
			button.panelButton.output.power_state = 0;

		if (++_ledPowerCounter > 4)
			_ledPowerCounter = 0;
	}

	if (HAL_GetTick() >= _panelSendTimer) {
		_panelSendTimer = HAL_GetTick() + 50;

		bus.sendPanel.data[0] = *(uint8_t*) &bus.panelButtonCommand.movementMode;
		bus.sendPanel.data[1] = *(uint8_t*) &bus.panelButtonCommand.cameraCommand;
		bus.sendPanel.data[2] = *(uint8_t*) &bus.panelButtonCommand.weaponCommand;
		bus.sendPanel.data[3] = *(uint8_t*) &bus.panelButtonCommand.lrfImuCommand;
		mtr[0].i32 = bus.panelButtonCommand.panSpeedCommand;
		mtr[1].i32 = bus.panelButtonCommand.tiltSpeedCommand;
		mtr[2].i32 = bus.panelButtonCommand.panSpeedCorrection;
		mtr[3].i32 = bus.panelButtonCommand.tiltSpeedCorrection;

		uint16_t index = 4;
		for ( int i = 0; i < 4; i++ ) {
			for ( int j = 0; j < 4; j++ )
				bus.sendPanel.data[index++] = mtr[i].u8[j];
		}
		/* Start the Transmission process */
		HAL_FDCAN_AddMessageToTxFifoQ(bus.hfdcan, &txHeader, bus.sendPanel.data);
	}

}

void Bus_Notification_Callback(FDCAN_RxHeaderTypeDef *rxHeader, uint8_t *data)
{
	uint8_t _len;
	if (rxHeader->IdType == FDCAN_STANDARD_ID) {
		if (rxHeader->Identifier == RWS_MOTOR_ID) {
			bus.recvMotor.lastTimestamp = HAL_GetTick();
			bus.recvMotor.id = rxHeader->Identifier;
			bus.recvMotor.idType = CAN_BUS_STANDARD;
			_len = bus.recvMotor.len = FDCAN_Convert_Datalength(rxHeader->DataLength);
//			bus.recvMotor.rxHeader = *rxHeader;
			memcpy(bus.recvMotor.data, data, _len);
			bus.recvMotor.counter++;
		}
		else if (rxHeader->Identifier == RWS_OPTRONIK_ID) {
			bus.recvOptronik.lastTimestamp = HAL_GetTick();
			bus.recvOptronik.id = rxHeader->Identifier;
			bus.recvOptronik.idType = CAN_BUS_STANDARD;
			_len = bus.recvOptronik.len = FDCAN_Convert_Datalength(rxHeader->DataLength);
//			bus.recvOptronik.rxHeader = *rxHeader;
			memcpy(bus.recvOptronik.data, data, _len);
			bus.recvOptronik.counter++;
		}
		else if (rxHeader->Identifier == RWS_IMU_ID) {
			bus.recvImu.lastTimestamp = HAL_GetTick();
			bus.recvImu.id = rxHeader->Identifier;
			bus.recvImu.idType = CAN_BUS_STANDARD;
			_len = bus.recvImu.len = FDCAN_Convert_Datalength(rxHeader->DataLength);
//			bus.recvImu.rxHeader = *rxHeader;
			memcpy(bus.recvImu.data, data, _len);
			bus.recvImu.counter++;
		}
	}
//	if (rxHeader->IdType == FDCAN_STANDARD_ID) {
//		if (rxHeader->Identifier == RWS_MOTOR_ID) {
//			bus.recvMotor.lastTimestamp = HAL_GetTick();
//			bus.recvMotor.rxHeader = *rxHeader;
//			memcpy(bus.recvMotor.data, data, FDCAN_Convert_Datalength(rxHeader->DataLength));
//			bus.recvMotor.counter++;
//		}
//		else if (rxHeader->Identifier == RWS_OPTRONIK_ID) {
//			bus.recvOptronik.lastTimestamp = HAL_GetTick();
//			bus.recvOptronik.rxHeader = *rxHeader;
//			memcpy(bus.recvOptronik.data, data, FDCAN_Convert_Datalength(rxHeader->DataLength));
//			bus.recvOptronik.counter++;
//		}
//		else if (rxHeader->Identifier == RWS_IMU_ID) {
//			bus.recvImu.lastTimestamp = HAL_GetTick();
//			bus.recvImu.rxHeader = *rxHeader;
//			memcpy(bus.recvImu.data, data, FDCAN_Convert_Datalength(rxHeader->DataLength));
//			bus.recvImu.counter++;
//		}
//	}
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
	g_fdcan_bus_busOff_error = 1;
}

