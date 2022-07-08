/*
 * tbt_def.h
 *
 *  Created on: Dec 30, 2021
 *      Author: miftakur
 */

#ifndef DRIVER_TBT_TBT_DEF_H_
#define DRIVER_TBT_TBT_DEF_H_

/**********************/
/* Message formatting */
/**********************/
#define TBT_HEADER								0xA0
#define TBT_TERMINATOR							0xAF

/***************/
/* ERROR CODES */
/***************/

/* these two are defined by me, not by the specs. */
#define TBT_SUCCESS                    0x00
#define TBT_FAILURE                    0xFF

/* specs errors: */
#define TBT_ERROR_BUFFER_FULL			0x01
#define TBT_ERROR_SYSTEM_BUSY			0x02
#define TBT_ERROR_INVALID_PARAMETER		0x03
#define TBT_ERROR_UNKNOWN_PACKET		0x04
#define TBT_ERROR_NOT_AVAILABLE			0x05

/* Commands/inquiries codes */
#define TBT_INITIALIZE							0x01
#define TBT_REBOOTING							0x02
#define TBT_SAVE_SETTING						0x03
#define TBT_DIGITAL_ZOOM_START					0x11
#define   TBT_DIGITAL_ZOOM_START_TELE			  0x00
#define   TBT_DIGITAL_ZOOM_START_WIDE			  0x01
#define TBT_DIGITAL_ZOOM_STOP					0x10
#define TBT_DIGITAL_ZOOM_DIRECT					0x12
#define   TBT_DIGITAL_ZOOM_DIRECT_MIN			0x10	/* x1.0 */
#define   TBT_DIGITAL_ZOOM_DIRECT_MAX			0x1E	/* x4 */
#define TBT_OPTICAL_ZOOM_START					0x13
#define   TBT_OPTICAL_ZOOM_START_TELE			  0x00
#define   TBT_OPTICAL_ZOOM_START_WIDE			  0x01
#define   TBT_OPTICAL_ZOOM_START_SPEED_MIN		  0x01
#define   TBT_OPTICAL_ZOOM_START_SPEED_MAX		  0x07
#define TBT_OPTICAL_ZOOM_STOP					0x14
#define TBT_OPTICAL_ZOOM_DIRECT					0x15
#define   TBT_OPTICAL_ZOOM_DIRECT_POS_MIN		  1
#define   TBT_OPTICAL_ZOOM_DIRECT_POS_MAX		  11999
#define   TBT_OPTICAL_ZOOM_DIRECT_SPEED_MIN		  1
#define   TBT_OPTICAL_ZOOM_DIRECT_SPEED_MAX		  7
#define TBT_FOCUS_START							0x16
#define   TBT_FOCUS_START_NEAR					  0x00
#define   TBT_FOCUS_START_FAR					  0x01
#define   TBT_FOCUS_START_SPEED_MIN				  0x01
#define   TBT_FOCUS_START_SPEED_MAX				  0x07
#define TBT_FOCUS_STOP							0x17
#define TBT_FOCUS_DIRECT						0x18
#define   TBT_FOCUS_DIRECT_POS_MIN				  1
#define   TBT_FOCUS_DIRECT_POS_MAX				  13999
#define   TBT_FOCUS_DIRECT_SPEED_MIN			  0x01
#define   TBT_FOCUS_DIRECT_SPEED_MAX			  0x07
#define TBT_AUTO_FOCUS_START					0x19

#define TBT_COLOR_MODE							0x27
#define   TBT_COLOR_MODE_GRAY					  0x00
#define   TBT_COLOR_MODE_RAINBOW				  0x01
#define   TBT_COLOR_MODE_IRON					  0x02
#define   TBT_COLOR_MODE_GLOWBOW				  0x03
#define   TBT_COLOR_MODE_2COLOR					  0x04
#define TBT_COLOR_INVERSE						0x47
#define   TBT_COLOR_INVERSE_WHITE_HOT			  0x00
#define   TBT_COLOR_INVERSE_BLACK_HOT			  0x01

#define TBT_MIRROR_MODE							0x44
#define   TBT_MIRROR_MODE_OFF					  0x00
#define   TBT_MIRROR_MODE_ON					  0x01
#define TBT_FLIP_MODE							0x45
#define   TBT_FLIP_MODE_OFF					  	  0x00
#define   TBT_FLIP_MODE_ON						  0x01

#define TBT_IMAGE_STABILIZER					0x50
#define   TBT_IMAGE_STABILIZER_OFF				  0x00
#define   TBT_IMAGE_STABILIZER_ON				  0x01

#define TBT_VIDEO_MODE							0x79
#define 	TBT_VIDEO_MODE_NTSC					  0x00
#define 	TBT_VIDEO_MODE_PAL					  0x01

#endif /* DRIVER_TBT_TBT_DEF_H_ */
