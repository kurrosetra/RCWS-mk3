/*
 * VL6180X.h
 *
 *  Created on: Dec 1, 2021
 *      Author: miftakur
 */

#ifndef VL6180X_H_
#define VL6180X_H_

#include <stdbool.h>
#include "main.h"

#define VL6180X_DEBUG_ENABLE						1

/*I2C ADDRESS*/
#define VL6180X_I2C_ADDRESS							0x52
#define VL6180X_I2C_TIMEOUT							100

/*---------------------Register Address-------------------*/
//Device model identification number：0XB4
#define VL6180X_IDENTIFICATION_MODEL_ID               0x000

#define VL6180X_SYSTEM_MODE_GPIO0                     0X010
#define VL6180X_SYSTEM_MODE_GPIO1                     0X011
/*
 * SYSTEM__MODE_GPIO1
 * -------------------------------------------------------------------------------------------------
 * |    b7    |    b6    |       b5        |    b4    |    b3    |    b2    |    b1     |    b0    |
 * -------------------------------------------------------------------------------------------------
 * |      reversed       |     polarity    |                   select                   | reversed |
 * -------------------------------------------------------------------------------------------------
 *
 *
 */
typedef struct
{
	uint8_t reversed :1; /* Reserved. Write 0.*/
	uint8_t select :4; /* Functional configuration options [bit:4-1].
	 0000: OFF (Hi-Z)
	 1000: GPIO Interrupt output
	 */
	uint8_t polarity :1; /* Signal Polarity Selection.[bit:5]
	 0: Active-low
	 1: Active-high
	 */
	uint8_t reservedBit6_7 :2; /* Reserved. */
} __attribute__ ((packed)) sModeGpio1Reg_t;

#define VL6180X_SYSTEM_INTERRUPT_CONFIG_GPIO          0x014
/*
 * SYSTEM__INTERRUPT_CONFIG_GPIO
 * ---------------------------------------------------------------------------------------------
 * |    b7    |    b6    |       b5    |    b4    |    b3    |    b2    |    b1     |    b0    |
 * ---------------------------------------------------------------------------------------------
 * |      reversed       |             als_int_mode          |         range_int_mode          |
 * ---------------------------------------------------------------------------------------------
 */
typedef struct
{
	uint8_t rangeIntMode :3; /*  Interrupt mode source for ALS readings[bit:2-0]:
	 0: Disabled
	 1: Level Low (value < thresh_low)
	 2: Level High (value > thresh_high)
	 3: Out Of Window (value < thresh_low OR value > thresh_high)
	 4: New sample ready
	 */
	uint8_t alsIntMode :3; /*  Interrupt mode source for Range readings[bit:5-3]:
	 0: Disabled
	 1: Level Low (value < thresh_low)
	 2: Level High (value > thresh_high)
	 3: Out Of Window (value < thresh_low OR value > thresh_high)
	 4: New sample ready
	 */
	uint8_t reservedBit6_7 :2; /* Reserved. */
} __attribute__ ((packed)) sConfigIntGPIOReg_t;

#define VL6180X_SYSTEM_INTERRUPT_CLEAR                0x015
/*
 * SYSTEM__INTERRUPT_CLEAR
 * ---------------------------------------------------------------------------------------------
 * |    b7    |    b6    |       b5    |    b4    |    b3    |    b2    |    b1     |    b0    |
 * ---------------------------------------------------------------------------------------------
 * |                         reversed                        |          int_clear_sig          |
 * ---------------------------------------------------------------------------------------------
 *
 *
 */
typedef struct
{
	uint8_t intClearSig :3; /*  Interrupt clear bits. Writing a 1 to each bit will clear the intended interrupt.[bit:2-0]:
	 Bit [0] - Clear Range Int
	 Bit [1] - Clear ALS Int
	 Bit [2] - Clear Error Int.
	 */
	uint8_t reserved :5; /* Reserved. */
} __attribute__ ((packed)) sClearIntReg_t;

#define VL6180X_SYSTEM_FRESH_OUT_OF_RESET             0x016

#define VL6180X_SYSTEM_GROUPED_PARAMETER_HOLD         0x017

#define VL6180X_SYSRANGE_START                        0x018
/*
 * SYSRANGE__START
 * -----------------------------------------------------------------------------------------------
 * |    b7    |    b6    |       b5    |    b4    |    b3    |    b2    |    b1     |     b0     |
 * -----------------------------------------------------------------------------------------------
 * |                             reversed                               |   select  | startstop  |
 * -----------------------------------------------------------------------------------------------
 *
 *
 */
typedef struct
{
	uint8_t startstop :1; /*  StartStop trigger based on current mode and system configuration of device_ready. FW clears register automatically[bit:0]:
	 Setting this bit to 1 in single-shot mode starts a single measurement
	 Setting this bit to 1 in continuous mode will either start continuous operation (if stopped) or halt continuous operation (if started).
	 */
	uint8_t select :1; /*  Device Mode select
	 0: Ranging Mode Single-Shot
	 1: Ranging Mode Continuous
	 */
	uint8_t reserved :6; /* Reserved. */
} __attribute__ ((packed)) sRangeStartReg_t;

// High Threshold value for ranging comparison. Range 0-255mm.
#define VL6180X_SYSRANGE_THRESH_HIGH                  0x019

//Low Threshold value for ranging comparison. Range 0-255mm.
#define VL6180X_SYSRANGE_THRESH_LOW                   0x01A

// Time delay between measurements in Ranging continuous mode. Range 0-254 (0 = 10ms). Step size = 10ms.
#define VL6180X_SYSRANGE_INTERMEASUREMENT_PERIOD      0x01B

//Maximum time to run measurement in Ranging modes.Range 1 - 63 ms
#define VL6180X_SYSRANGE_MAX_CONVERGENCE_TIME         0x01C

#define VL6180X_SYSRANGE_EARLY_CONVERGENCE_ESTIMATE   0x022
#define VL6180X_SYSRANGE_MAX_AMBIENT_LEVEL_MULT       0x02C
#define VL6180X_SYSRANGE_RANGE_CHECK_ENABLES          0x02D
#define VL6180X_SYSRANGE_VHV_RECALIBRATE              0x02E
#define VL6180X_SYSRANGE_VHV_REPEAT_RATE              0x031

#define VL6180X_SYSALS_START                          0x038
/*
 * SYSALS__START
 * -----------------------------------------------------------------------------------------------
 * |    b7    |    b6    |       b5    |    b4    |    b3    |    b2    |    b1     |     b0     |
 * -----------------------------------------------------------------------------------------------
 * |                             reversed                               |   select  | startstop  |
 * -----------------------------------------------------------------------------------------------
 *
 *
 */
typedef struct
{
	uint8_t startstop :1; /*  Start/Stop trigger based on current mode and system configuration of device_ready. FW clears register automatically.[bit:0]:
	 Setting this bit to 1 in single-shot mode starts a single measurement
	 Setting this bit to 1 in continuous mode will either start continuous operation (if stopped) or halt continuous operation (if started).
	 */
	uint8_t select :1; /*  Device Mode select
	 0: ALS Mode Single-Shot
	 1: ALS Mode Continuous
	 */
	uint8_t reserved :6; /* Reserved. */
} __attribute__ ((packed)) sALSStartReg_t;
//High Threshold value for ALS comparison. Range 0-65535 codes.
#define VL6180X_SYSALS_THRESH_HIGH                    0x03A
// Low Threshold value for ALS comparison. Range 0-65535 codes.
#define VL6180X_SYSALS_THRESH_LOW                     0x03C
//Time delay between measurements in ALS continuous mode. Range 0-254 (0 = 10ms). Step size = 10ms.
#define VL6180X_SYSALS_INTERMEASUREMENT_PERIOD        0x03E

#define VL6180X_SYSALS_ANALOGUE_GAIN                  0x03F
/*
 * SYSALS_ANALOGUE_GAIN
 * ---------------------------------------------------------------------------------------------
 * |    b7    |    b6    |       b5    |    b4    |    b3    |    b2    |    b1     |    b0    |
 * ---------------------------------------------------------------------------------------------
 * |                         reversed                        |   sysals__analogue_gain_light   |
 * ---------------------------------------------------------------------------------------------
 *
 *
 */
typedef struct
{
	uint8_t gain :3; /*  ALS analogue gain[bit:2-0]:
	 0: ALS Gain = 20
	 1: ALS Gain = 10
	 2: ALS Gain = 5.0
	 3: ALS Gain = 2.5
	 4: ALS Gain = 1.67
	 5: ALS Gain = 1.25
	 6: ALS Gain = 1.0
	 7: ALS Gain = 40
	 */
	uint8_t reserved :5; /* Reserved. */
} __attribute__ ((packed)) sAnalogueGainReg_t;

// Integration period for ALS mode. 1 code = 1 ms (0 = 1 ms). Recommended setting is 100 ms (0x63).
#define VL6180X_SYSALS_INTEGRATION_PERIOD             0x040

#define VL6180X_RESULT_RANGE_STATUS                   0x04D
#define VL6180X_RESULT_ALS_STATUS                     0x04E
#define VL6180X_RESULT_INTERRUPT_STATUS_GPIO          0x04F
#define VL6180X_RESULT_ALS_VAL                        0x050
#define VL6180X_RESULT_RANGE_VAL                      0x062

#define VL6180X_READOUT_AVERAGING_SAMPLE_PERIOD       0x10A

#define VL6180X_FIRMWARE_RESULT_SCALER                0x120
#define VL6180X_I2C_SLAVE_DEVICE_ADDRESS              0x212
#define VL6180X_INTERLEAVED_MODE_ENABLE               0x2A3

#define VL6180X_ID                                    0xB4
#define VL6180X_ALS_GAIN_20                           0x00
#define VL6180X_ALS_GAIN_10                           0x01
#define VL6180X_ALS_GAIN_5                            0x02
#define VL6180X_ALS_GAIN_2_5                          0x03
#define VL6180X_ALS_GAIN_1_67                         0x04
#define VL6180X_ALS_GAIN_1_25                         0x05
#define VL6180X_ALS_GAIN_1                            0x06
#define VL6180X_ALS_GAIN_40                           0x07

#define VL6180X_NO_ERR                                0x00
#define VL6180X_EARLY_CONV_ERR                        0x06
#define VL6180X_MAX_CONV_ERR                          0x07
#define VL6180X_IGNORE_ERR                            0x08
#define VL6180X_MAX_S_N_ERR                           0x0B
#define VL6180X_RAW_Range_UNDERFLOW_ERR               0x0C
#define VL6180X_RAW_Range_OVERFLOW_ERR                0x0D
#define VL6180X_Range_UNDERFLOW_ERR                   0x0E
#define VL6180X_Range_OVERFLOW_ERR                    0x0F

#define VL6180X_DIS_INTERRUPT        0
#define VL6180X_HIGH_INTERRUPT       1
#define VL6180X_LOW_INTERRUPT        2

#define VL6180X_INT_DISABLE          0
#define VL6180X_LEVEL_LOW            1
#define VL6180X_LEVEL_HIGH           2
#define VL6180X_OUT_OF_WINDOW        3
#define VL6180X_NEW_SAMPLE_READY     4

typedef struct
{
	/* public */
	I2C_HandleTypeDef *hi2c;

	/* private */
	sModeGpio1Reg_t _modeGpio1Reg;
	sConfigIntGPIOReg_t _configIntGPIOReg;
	sClearIntReg_t _clearIntReg;
	sRangeStartReg_t _rangeStartReg;
	sALSStartReg_t _ALSStartReg;
	sAnalogueGainReg_t _analogueGainReg;
	float _gain;
	uint8_t _atime;
	uint8_t _deviceAddr;
	bool _continuousRangeMode;
	bool _continuousALSMode;

} VL6180X_t;

int8_t VL6180X_begin(VL6180X_t *tof, I2C_HandleTypeDef *hi2c);
void VL6180X_set_default_configuration(VL6180X_t *tof);

bool VL6180X_init(VL6180X_t *tof, I2C_HandleTypeDef *hi2c);
void VL6180X_set_interrupt(VL6180X_t *tof, const uint8_t mode);
uint8_t VL6180X_range_poll_measurement(VL6180X_t *tof);
void VL6180X_range_set_inter_measurement_period(VL6180X_t *tof, uint16_t periodMs);
bool VL6180X_range_config_interrupt(VL6180X_t *tof, const uint8_t mode);
bool VL6180X_als_config_interrupt(VL6180X_t *tof, const uint8_t mode);
void VL6180X_range_start_continous_mode(VL6180X_t *tof);
uint8_t VL6180X_range_get_measurement(VL6180X_t *tof);
void VL6180X_clear_als_interrupt(VL6180X_t *tof);
void VL6180X_clear_range_interrupt(VL6180X_t *tof);
float VL6180X_als_poll_measurement(VL6180X_t *tof);
float VL6180X_als_get_measurement(VL6180X_t *tof);
void VL6180X_als_start_continous_mode(VL6180X_t *tof);
void VL6180X_als_set_inter_measurement_period(VL6180X_t *tof, uint16_t periodMs);
void VL6180X_start_interleaved_mode(VL6180X_t *tof);
uint8_t VL6180X_range_get_interrupt_status(VL6180X_t *tof);
uint8_t VL6180X_als_get_interrupt_status(VL6180X_t *tof);
uint8_t VL6180X_get_range_result(VL6180X_t *tof);
void VL6180X_set_i2c_address(VL6180X_t *tof, const uint8_t newAddress);
bool VL6180X_set_als_gain(VL6180X_t *tof, const uint8_t gain);
void VL6180X_set_als_threshold_value(VL6180X_t *tof, const uint16_t thresholdL,
		const uint16_t thresholdH);
void VL6180X_set_range_threshold_value(VL6180X_t *tof, const uint16_t thresholdL,
		const uint16_t thresholdH);
void VL6180X_set_fresh_reset(VL6180X_t *tof, const uint8_t reset_state);

#endif /* VL6180X_H_ */
