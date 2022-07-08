/*
 * noptel_def.h
 *
 *  Created on: Jan 1, 2022
 *      Author: miftakur
 */

#ifndef DRIVER_NOPTEL_NOPTEL_DEF_H_
#define DRIVER_NOPTEL_NOPTEL_DEF_H_

#define NOPTEL_POWER_ON_STRING					"LRF2XX-M4       1.4.24"

#define NOPTEL_ACK_SYNC_HEADER_BYTE				0x59
#define NOPTEL_ACK_BYTE							0x3C

#define NOPTEL_SET_MEASURE_DISTANCE				0xC3
#define NOPTEL_SET_MEASURE_DISTANCE_STATUS		0x12
#define NOPTEL_SET_POINTER_MODE					0xC5
#define   NOPTEL_SET_POINTER_MODE_OFF			  0
#define   NOPTEL_SET_POINTER_MODE_ON			  3
#define NOPTEL_SET_MIN_RANGE					0x31
#define NOPTEL_SET_MAX_RANGE					0x32
#define NOPTEL_SET_IBIT_TEST					0x11
#define NOPTEL_SET_RESET_COMM_ERROR_COUNTER		0xCB

#define NOPTEL_INQ_LAST_DISTANCE_RESULT			0xC4
#define NOPTEL_INQ_STATUS_BYTE					0xC7
#define NOPTEL_INQ_RANGE_WINDOW					0x30
#define NOPTEL_INQ_IDENTIFICATION_FRAME			0xC0
#define NOPTEL_INQ_INFORMATION_FRAME			0xC2


#define NOPTEL_STATUS_B3_LPW_bit				0	/* Laser Power On (laser driving voltage on) */
#define NOPTEL_STATUS_B3_LA_bit					1	/* Laser Active (if continuous pulsing mode set) */
#define NOPTEL_STATUS_B3_MAD_bit				2	/* Transmitter sending pulses without request */
#define NOPTEL_STATUS_B3_NR_bit					3	/* Not Ready to perform (too high request) */
#define NOPTEL_STATUS_B3_ERR_bit				4	/* Error reported by LRF */
#define NOPTEL_STATUS_B3_NT_bit					5	/* No Target */
#define NOPTEL_STATUS_B3_MT_bit					6	/* Multiple target detected */



#endif /* DRIVER_NOPTEL_NOPTEL_DEF_H_ */
