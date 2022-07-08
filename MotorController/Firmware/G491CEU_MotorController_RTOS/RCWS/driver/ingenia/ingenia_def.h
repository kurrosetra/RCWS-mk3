/*
 * ingenia_def.h
 *
 *  Created on: Jan 5, 2022
 *      Author: miftakur
 */

#ifndef DRIVER_INGENIA_INGENIA_DEF_H_
#define DRIVER_INGENIA_INGENIA_DEF_H_

#define STATUS_WORD_REGISTER_BITS_READY_TO_SWITCH_ON     0x0001
#define STATUS_WORD_REGISTER_BITS_SWITCHED_ON            0x0002
#define STATUS_WORD_REGISTER_BITS_OPERATION_ENABLED      0x0004
#define STATUS_WORD_REGISTER_BITS_FAULT                  0x0008
#define STATUS_WORD_REGISTER_BITS_VOLTAGE_ENABLED        0x0010
#define STATUS_WORD_REGISTER_BITS_QUICK_STOP             0x0020
#define STATUS_WORD_REGISTER_BITS_SWITCH_ON_DISABLED     0x0040
#define STATUS_WORD_REGISTER_BITS_WARNING                0x0080
#define STATUS_WORD_REGISTER_BITS_RESERVED               0x0100
#define STATUS_WORD_REGISTER_BITS_REMOTE                 0x0200
#define STATUS_WORD_REGISTER_BITS_TARGET_REACHED         0x0400
#define STATUS_WORD_REGISTER_BITS_INTERNAL_LIMIT_ACTIVE  0x0800
#define STATUS_WORD_REGISTER_BITS_OPMODE_SPECIFIC_1      0x1000
#define STATUS_WORD_REGISTER_BITS_OPMODE_SPECIFIC_2      0x2000
#define STATUS_WORD_REGISTER_BITS_ANGLE_PROC_FINISHED    0x4000

//
// Control Word Register (CWR) bits meaning
//
//  --------------------------------------------------------------------------------
// | 15     9 |   8  |   7    | 6      4 |    3      |   2     |   1       |    0   |
// | -------------------------------------------------------------------------------
// | Reserved | Halt | Fault  | Mode     | Enable    | Quick   | Enable    | Switch |
// |          |      | reset  | specific | operation | stop    | voltage   | on     |
//  --------------------------------------------------------------------------------
//
#define CONTROL_WORD_REGISTER_BITS_SWITCH_ON             0x0001
#define CONTROL_WORD_REGISTER_BITS_ENABLE_VOLTAGE        0x0002
#define CONTROL_WORD_REGISTER_BITS_QUICK_STOP            0x0004
#define CONTROL_WORD_REGISTER_BITS_ENABLE_OPERATION      0x0008
#define CONTROL_WORD_REGISTER_BITS_MODE_SPECIFIC         0x0070
#define CONTROL_WORD_REGISTER_BITS_FAULT_RESET           0x0080
#define CONTROL_WORD_REGISTER_BITS_HALT                  0x0100
#define CONTROL_WORD_REGISTER_BITS_14_RESERVED           0x4000

// Useful object definitions (index, subindex)
#define OBJECT_CONTROL_WORD            0x6040,0x00
#define OBJECT_STATUS_WORD             0x6041,0x00

#define SDO_DOWNLOAD_REQUEST_BITS	0x20
#define SDO_DOWNLOAD_RESPONSE_BITS	0x60
#define SDO_UPLOAD_REQUEST_BITS		0x40
#define SDO_UPLOAD_RESPONSE_BITS	0x40

#define SDO_DATA_SIZE_MASK			0x0C
#define SDO_DATA_SIZE_1				0x0C
#define SDO_DATA_SIZE_2				0x08
#define SDO_DATA_SIZE_3				0x04
#define SDO_DATA_SIZE_4				0x00

#define SDO_E_TRANSFER_BIT			0x02
#define SDO_S_SIZE_INDICATOR_BIT	0x01

typedef enum
{
	COB_NMT_SERVICE = 0,
	COB_EMERGENCY = 0x80,
	COB_TPDO1 = 0x180,
	COB_TPDO2 = 0x280,
	COB_TPDO3 = 0x380,
	COB_TPDO4 = 0x480,
	COB_RPDO1 = 0x200,
	COB_RPDO2 = 0x300,
	COB_RPDO3 = 0x400,
	COB_RPDO4 = 0x500,
	COB_TSDO = 0x580,
	COB_RSDO = 0x600,
	COB_NMT_CTRL = 0x700
} COB_ID_e;

//! State Machine Commands
typedef enum
{
	COMMAND_SHUTDOWN,						//!< Shutdown command
	COMMAND_SWITCH_ON,						//!< Switch on command
	COMMAND_SWITCH_ON_AND_ENABLE_OPERATION,  //!< Switch on + Enable operation command
	COMMAND_DISABLE_VOLTAGE,				//!< Disable voltage command
	COMMAND_QUICK_STOP,						//!< Quick stop command
	COMMAND_DISABLE_OPERATION,				//!< Disable operation command
	COMMAND_ENABLE_OPERATION,				//!< Enable operation command
	COMMAND_FAULT_RESET						//!< Fault reset command
} StateMachineCommand_e;

//! The enumeration of State Machine Status
typedef enum
{
	STATUS_NOT_READY_TO_SWITCH_ON,	//!< Not ready to switch ON
	STATUS_SWITCH_ON_DISABLED,		//!< Switch ON disabled
	STATUS_READY_TO_SWITCH_ON,		//!< Ready to switch ON
	STATUS_SWITCH_ON,				//!< Switch ON status
	STATUS_OPERATION_ENABLED,		//!< Operation enabled
	STATUS_QUICK_STOP_ACTIVE,		//!< Quick stop active
	STATUS_FAULT_REACTION_ACTIVE,	//!< Fault reaction active
	STATUS_FAULT,					//!< Fault status
	STATUS_UNKNOWN					//!< Unknown status
} StateMachineStatus_e;

//! The enumeration of Drive Modes
typedef enum
{
	DRIVE_MODE_PROFILE_POSITION = 1,            //!< Profile position mode
	DRIVE_MODE_PROFILE_VELOCITY = 3,            //!< Profile velocity mode
	DRIVE_MODE_PROFILE_TORQUE = 4,              //!< Profile torque mode
	DRIVE_MODE_VELOCITY = 2,                    //!< Velocity mode
	DRIVE_MODE_HOMING = 6,                      //!< Homing mode
	DRIVE_MODE_INTERPOLATED_POSITION = 7,       //!< Interpolated position mode
	DRIVE_MODE_CYCLIC_SYNC_POSITION = 8,        //!< Cyclic sync position mode
	DRIVE_MODE_CYCLIC_SYNC_VELOCITY = 9,        //!< Cyclic sync velocity mode
	DRIVE_MODE_CYCLIC_SYNC_TORQUE = 10,         //!< Cyclic sync torque mode
	DRIVE_MODE_OPEN_LOOP_SCALAR = -1,           //!< Open loop scalar mode
	DRIVE_MODE_OPEN_LOOP_VECTOR = -2            //!< Open loop vector mode
} DriveModes_e;

//! The enumeration of NMT Modes
typedef enum
{
	NMT_START_REMOTE_NODE = 1,
	NMT_STOP_REMOTE_NODE = 2,
	NMT_ENTER_PRE_OPS = 0x80,
	NMT_RESET_NODE = 0x81,
	NMT_RESET_COMM = 0x82
} NmtModes_e;


/* TODO add register name here */
#define POSITION_ACTUAL_INDEX         0x6064
#define POSITION_ACTUAL_SUBINDEX      0x0
#define TORQUE_ACTUAL_INDEX           0x6077
#define TORQUE_ACTUAL_SUBINDEX        0x0
#define VELOCITY_ACTUAL_INDEX         0x606C
#define VELOCITY_ACTUAL_SUBINDEX      0x0

#endif /* DRIVER_INGENIA_INGENIA_DEF_H_ */
