/*
 * VL6180X.c
 *
 *  Created on: Dec 1, 2021
 *      Author: miftakur
 */

#include "VL6180X.h"

#if VL6180X_DEBUG_ENABLE==1
#include <stdio.h>
#	define LOG(str, ...) printf("[%04ld %s:%d] " str, HAL_GetTick()%10000, __FILE_NAME__, __LINE__, ##__VA_ARGS__)
#	define LOG_E(str, ...) printf("[%s Err:%d] " str,  __FILE_NAME__,__LINE__, ##__VA_ARGS__)
#else
#	define LOG(str, ...)
#	define LOG_E(str, ...)
#endif	//if VL6180X_DEBUG_ENABLE==1

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

/**
 * @brief  config register(8 byte)
 * @param  *tof  module object pointer
 * @param  regAddr  register address
 * @param  value  Writes the value of the register
 */
static void write8bit(VL6180X_t *tof, const uint16_t regAddr, const uint8_t value)
{
	HAL_I2C_Mem_Write(tof->hi2c, tof->_deviceAddr, regAddr, I2C_MEMADD_SIZE_16BIT, (uint8_t*) &value, 1,
	VL6180X_I2C_TIMEOUT);
}

/**
 * @brief  config register(16 byte)
 * @param  *tof  module object pointer
 * @param  regAddr  register address
 * @param  value  Writes the value of the register
 */
static void write16bit(VL6180X_t *tof, const uint16_t regAddr, const uint16_t value)
{
	uint8_t data[2];
	data[0] = value >> 8;
	data[1] = value & 0xFF;
	HAL_I2C_Mem_Write(tof->hi2c, tof->_deviceAddr, regAddr, I2C_MEMADD_SIZE_16BIT, data, 2,
	VL6180X_I2C_TIMEOUT);

}

/**
 * @brief  read register
 * @param  *tof  module object pointer
 * @param  regAddr  register address
 * @param  readNum  Number of bytes read
 * @return Read data from a register
 */
static uint16_t read(VL6180X_t *tof, const uint16_t regAddr, const uint8_t readNum)
{
	uint8_t data[2] = { 0, 0 };
	uint8_t num = constrain(readNum, 1, 2);
	uint16_t ret = 0;

	HAL_I2C_Mem_Read(tof->hi2c, tof->_deviceAddr, regAddr, I2C_MEMADD_SIZE_16BIT, data, num,
	VL6180X_I2C_TIMEOUT);

	if (num == 1)
		ret = data[0];
	else
		ret = ((uint16_t) data[0] << 8 | data[1]);

	return ret;
}

/**
 * @brief  get the identifier of sensor
 * @return Authentication information
 */
static uint8_t get_device_ID(VL6180X_t *tof)
{
	return (uint8_t) read(tof, VL6180X_IDENTIFICATION_MODEL_ID, 1);
}

///**
// * @brief  Gets validation information for ALS data
// * @return Authentication information
// */
//static void get_als_result(VL6180X_t *tof)
//{
//	return read(tof, VL6180X_RESULT_ALS_STATUS, 1) >> 4;
//}
//
///**
// * @brief disable continuous ranging mode
// */
//static void range_stop_continous_mode(VL6180X_t *tof)
//{
//	tof->_rangeStartReg.select = 0;
//	write8bit(tof, VL6180X_SYSRANGE_START, *((uint8_t*) (&tof->_rangeStartReg)));
//}
//
///**
// * @brief  disable continuous measurement of ambient light intensity mode
// */
//static void als_stop_continous_mode(VL6180X_t *tof)
//{
//	tof->_ALSStartReg.select = 0;
//	write8bit(tof, VL6180X_SYSALS_START, *((uint8_t*) (&tof->_ALSStartReg)));
//}
//
///**
// * @brief  @brief  turn off interleaved mode
// */
//static void stop_interleaved_mode(VL6180X_t *tof)
//{
//	tof->_ALSStartReg.startstop = 1;
//	tof->_ALSStartReg.select = 0;
//	write8bit(tof, VL6180X_SYSALS_START, *((uint8_t*) (&tof->_ALSStartReg)));
//	write8bit(tof, VL6180X_INTERLEAVED_MODE_ENABLE, 0x00);
//}

/**
 * @brief  @brief  stop measuremwnt
 */
static void stop_measurement(VL6180X_t *tof)
{
	tof->_ALSStartReg.startstop = 1;
	tof->_ALSStartReg.select = 0;
	write8bit(tof, VL6180X_SYSALS_START, *((uint8_t*) (&tof->_ALSStartReg)));
	tof->_rangeStartReg.startstop = 1;
	tof->_rangeStartReg.select = 0;
	write8bit(tof, VL6180X_SYSRANGE_START, *((uint8_t*) (&tof->_rangeStartReg)));
	write8bit(tof, VL6180X_INTERLEAVED_MODE_ENABLE, 0x00);
	HAL_Delay(1000);
	write8bit(tof, VL6180X_SYSTEM_INTERRUPT_CLEAR, 7);
}

/**
 * @brief  @brief  load default configuration
 */
void VL6180X_set_default_configuration(VL6180X_t *tof)
{
	write8bit(tof, VL6180X_READOUT_AVERAGING_SAMPLE_PERIOD, 0x30);
	write8bit(tof, VL6180X_SYSALS_ANALOGUE_GAIN, 0x46);
	write8bit(tof, VL6180X_SYSRANGE_VHV_REPEAT_RATE, 0xFF);
	write8bit(tof, VL6180X_SYSALS_INTEGRATION_PERIOD, 0x63);
	write8bit(tof, VL6180X_SYSRANGE_VHV_RECALIBRATE, 0x01);

	write8bit(tof, VL6180X_SYSRANGE_INTERMEASUREMENT_PERIOD, 0x09);
	write8bit(tof, VL6180X_SYSALS_INTERMEASUREMENT_PERIOD, 0x31);
	write8bit(tof, VL6180X_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x00);
//	write8bit(tof, VL6180X_SYSRANGE_MAX_CONVERGENCE_TIME, 0x31);
	write8bit(tof, VL6180X_SYSRANGE_MAX_CONVERGENCE_TIME, 0x14);
	write8bit(tof, VL6180X_INTERLEAVED_MODE_ENABLE, 0);
	write8bit(tof, VL6180X_SYSTEM_MODE_GPIO1, 0x20);
	write8bit(tof, VL6180X_SYSTEM_FRESH_OUT_OF_RESET, 0);
}

/**
 * @brief  Initialize the sensor configuration
 */
static uint8_t init(VL6180X_t *tof)
{
	uint8_t fresh_reset = read(tof, VL6180X_SYSTEM_FRESH_OUT_OF_RESET, 1);
	LOG("reset= %d\r\n", fresh_reset);

	if (fresh_reset != 0) {
		write8bit(tof, 0x0207, 0x01);
		write8bit(tof, 0x0208, 0x01);
		write8bit(tof, 0x0096, 0x00);
		write8bit(tof, 0x0097, 0xfd);
		write8bit(tof, 0x00e3, 0x00);
		write8bit(tof, 0x00e4, 0x04);
		write8bit(tof, 0x00e5, 0x02);
		write8bit(tof, 0x00e6, 0x01);
		write8bit(tof, 0x00e7, 0x03);
		write8bit(tof, 0x00f5, 0x02);
		write8bit(tof, 0x00d9, 0x05);
		write8bit(tof, 0x00db, 0xce);
		write8bit(tof, 0x00dc, 0x03);
		write8bit(tof, 0x00dd, 0xf8);
		write8bit(tof, 0x009f, 0x00);
		write8bit(tof, 0x00a3, 0x3c);
		write8bit(tof, 0x00b7, 0x00);
		write8bit(tof, 0x00bb, 0x3c);
		write8bit(tof, 0x00b2, 0x09);
		write8bit(tof, 0x00ca, 0x09);
		write8bit(tof, 0x0198, 0x01);
		write8bit(tof, 0x01b0, 0x17);
		write8bit(tof, 0x01ad, 0x00);
		write8bit(tof, 0x00ff, 0x05);
		write8bit(tof, 0x0100, 0x05);
		write8bit(tof, 0x0199, 0x05);
		write8bit(tof, 0x01a6, 0x1b);
		write8bit(tof, 0x01ac, 0x3e);
		write8bit(tof, 0x01a7, 0x1f);
		write8bit(tof, 0x0030, 0x00);
	}

	return fresh_reset;
}

/**
 * @brief  Initialization function
 * @param  *tof  pointer to tof sensor
 * @param  *hi2c  pointer to i2c driver
 * @return Whether the device is fresh start or not or failed. return 1: fresh start 0: not fresh start return -1 failed.
 */
int8_t VL6180X_begin(VL6180X_t *tof, I2C_HandleTypeDef *hi2c)
{
	tof->hi2c = hi2c;
	tof->_deviceAddr = VL6180X_I2C_ADDRESS;
	*(uint8_t*) &tof->_modeGpio1Reg = 0;
	*(uint8_t*) &tof->_configIntGPIOReg = 0;
	*(uint8_t*) &tof->_clearIntReg = 0;
	*(uint8_t*) &tof->_rangeStartReg = 0;
	*(uint8_t*) &tof->_ALSStartReg = 0;
	tof->_analogueGainReg.gain = 6;

	tof->_gain = 1.0;
	tof->_atime = 100;

	uint8_t dev_id = get_device_ID(tof);
	if (dev_id != VL6180X_ID)
		return -1;
#if VL6180X_DEBUG_ENABLE==1
	printf("dev id= %02X\r\n", dev_id);
#endif	//if VL6180X_DEBUG_ENABLE==1

	return init(tof);
}

/**
 * @brief  Initialization function
 * @param  *hi2c  pointer to i2c driver
 * @return Whether the device is on or not. return true succeed ;return false failed.
 */
bool VL6180X_init(VL6180X_t *tof, I2C_HandleTypeDef *hi2c)
{
	tof->hi2c = hi2c;
	tof->_deviceAddr = VL6180X_I2C_ADDRESS;
	tof->_modeGpio1Reg.reversed = 0;
	tof->_modeGpio1Reg.select = 0;
	tof->_modeGpio1Reg.polarity = 0;
	tof->_modeGpio1Reg.reservedBit6_7 = 0;

	tof->_configIntGPIOReg.rangeIntMode = 0;
	tof->_configIntGPIOReg.alsIntMode = 0;
	tof->_configIntGPIOReg.reservedBit6_7 = 0;

	tof->_clearIntReg.intClearSig = 0;
	tof->_clearIntReg.reserved = 0;

	tof->_rangeStartReg.startstop = 0;
	tof->_rangeStartReg.select = 0;
	tof->_rangeStartReg.reserved = 0;

	tof->_ALSStartReg.startstop = 0;
	tof->_ALSStartReg.select = 0;
	tof->_ALSStartReg.reserved = 0;

	tof->_analogueGainReg.gain = 6;

	tof->_gain = 1.0;
	tof->_atime = 100;

	uint8_t dev_id = get_device_ID(tof);
	if (dev_id != VL6180X_ID)
		return false;
	LOG("dev id= %02X\r\n", dev_id);
	init(tof);

	return true;
}

/**
 * @brief  Configure the default level of the INT pin and enable the GPIO1 interrupt function
 * @param  mode  Enable interrupt mode
 * @n            VL6180X_DIS_INTERRUPT  disabled interrupt
 * @n            VL6180X_DIS_INTERRUPT  GPIO1 interrupt enabled, INT high by default
 * @n            VL6180X_LOW_INTERRUPT  GPIO1 interrupt enabled, INT low by default
 */
void VL6180X_set_interrupt(VL6180X_t *tof, const uint8_t mode)
{
	if (mode == VL6180X_DIS_INTERRUPT)
		write8bit(tof, VL6180X_SYSTEM_MODE_GPIO1, 0x20);
	else if (mode == VL6180X_HIGH_INTERRUPT)
		write8bit(tof, VL6180X_SYSTEM_MODE_GPIO1, 0x10);
	else if (mode == VL6180X_LOW_INTERRUPT)
		write8bit(tof, VL6180X_SYSTEM_MODE_GPIO1, 0x30);
}

/**
 * @brief  A single range
 * @return   return ranging data ,uint mm
 */
uint8_t VL6180X_range_poll_measurement(VL6180X_t *tof)
{
	tof->_rangeStartReg.startstop = 1;
	tof->_rangeStartReg.select = 0;
	write8bit(tof, VL6180X_SYSRANGE_START, *((uint8_t*) (&tof->_rangeStartReg)));
	return VL6180X_range_get_measurement(tof);

}

/**
 * @brief  Configuration ranging period
 * @param  period_ms  Measurement period, in milliseconds
 */
void VL6180X_range_set_inter_measurement_period(VL6180X_t *tof, uint16_t periodMs)
{
	periodMs = constrain(periodMs, 10, 2540);
	periodMs = (periodMs / 10) - 1;
//	if (periodMs >= 10) {
//		if (periodMs < 2550) {
//			periodMs = (periodMs / 10) - 1;
//		}
//		else
//			periodMs = 254;
//	}
	write8bit(tof, VL6180X_SYSRANGE_INTERMEASUREMENT_PERIOD, periodMs);
}

/**
 * @brief  Configure the interrupt mode for ranging
 * @param  mode  Enable interrupt mode
 * @n              VL6180X_INT_DISABLE                           interrupt disable
 * @n              VL6180X_LEVEL_LOW                             value < thresh_low
 * @n              VL6180X_LEVEL_HIGH                            value > thresh_high
 * @n              VL6180X_OUT_OF_WINDOW                         value < thresh_low OR value > thresh_high
 * @n              VL6180X_NEW_SAMPLE_READY                      new sample ready
 */
bool VL6180X_range_config_interrupt(VL6180X_t *tof, const uint8_t mode)
{
	if (mode > VL6180X_NEW_SAMPLE_READY) {
		return false;
	}
	tof->_configIntGPIOReg.rangeIntMode = mode;
	write8bit(tof, VL6180X_SYSTEM_INTERRUPT_CONFIG_GPIO, *((uint8_t*) (&tof->_configIntGPIOReg)));
	return true;
}

/**
 * @brief  Configure the interrupt mode for the ambient light
 * @param  mode  Enable interrupt mode
 * @n              VL6180X_INT_DISABLE                           interrupt disable
 * @n              VL6180X_LEVEL_LOW                             value < thresh_low
 * @n              VL6180X_LEVEL_HIGH                            value > thresh_high
 * @n              VL6180X_OUT_OF_WINDOW                         value < thresh_low OR value > thresh_high
 * @n              VL6180X_NEW_SAMPLE_READY                      new sample ready
 */
bool VL6180X_als_config_interrupt(VL6180X_t *tof, const uint8_t mode)
{
	if (mode > VL6180X_NEW_SAMPLE_READY) {
		return false;
	}
	tof->_configIntGPIOReg.alsIntMode = mode;
	write8bit(tof, VL6180X_SYSTEM_INTERRUPT_CONFIG_GPIO, *((uint8_t*) (&tof->_configIntGPIOReg)));
	return true;
}

/**
 * @brief Enable continuous ranging mode
 */
void VL6180X_range_start_continous_mode(VL6180X_t *tof)
{
	stop_measurement(tof);
	tof->_rangeStartReg.startstop = 1;
	tof->_rangeStartReg.select = 1;
	write8bit(tof, VL6180X_SYSRANGE_START, *((uint8_t*) (&tof->_rangeStartReg)));
}

/**
 * @brief  Retrieve ranging data
 * @return   return ranging data ,uint mm
 */
uint8_t VL6180X_range_get_measurement(VL6180X_t *tof)
{
	return (uint8_t) read(tof, VL6180X_RESULT_RANGE_VAL, 1);
}

/**
 * @brief  Clear the ambient light interrupt
 */
void VL6180X_clear_als_interrupt(VL6180X_t *tof)
{
	tof->_clearIntReg.intClearSig = 2;
	write8bit(tof, VL6180X_SYSTEM_INTERRUPT_CLEAR, *((uint8_t*) (&tof->_clearIntReg)));
}

/**
 * @brief  Clear ranging interrupt
 */
void VL6180X_clear_range_interrupt(VL6180X_t *tof)
{
//	tof->_clearIntReg.intClearSig = 1;
	tof->_clearIntReg.intClearSig = 3;
	write8bit(tof, VL6180X_SYSTEM_INTERRUPT_CLEAR, *((uint8_t*) (&tof->_clearIntReg)));
}

/**
 * @brief Single measurement of ambient light
 * @return   return The light intensity,uint lux
 */
float VL6180X_als_poll_measurement(VL6180X_t *tof)
{
	tof->_ALSStartReg.startstop = 1;
	tof->_ALSStartReg.select = 0;
	write8bit(tof, VL6180X_SYSALS_START, *((uint8_t*) (&tof->_ALSStartReg)));
	return VL6180X_als_get_measurement(tof);
}

/**
 * @brief  Obtain measured light data
 * @return   return The light intensity,uint lux
 */
float VL6180X_als_get_measurement(VL6180X_t *tof)
{
	float value = read(tof, VL6180X_RESULT_ALS_VAL, 2);
	value = (0.32 * 100 * value) / (tof->_gain * tof->_atime);
	return value;
}

/**
 * @brief  Enable continuous measurement of ambient light intensity mode
 */
void VL6180X_als_start_continous_mode(VL6180X_t *tof)
{
	stop_measurement(tof);
	tof->_ALSStartReg.startstop = 1;
	tof->_ALSStartReg.select = 1;
	write8bit(tof, VL6180X_SYSALS_START, *((uint8_t*) (&tof->_ALSStartReg)));
}

/**
 * @brief  Configure the period for measuring light intensity
 * @param  period_ms  Measurement period, in milliseconds
 */
void VL6180X_als_set_inter_measurement_period(VL6180X_t *tof, uint16_t periodMs)
{
	if (periodMs > 10) {
		if (periodMs < 2550) {
			periodMs = (periodMs / 10) - 1;
		}
		else {
			periodMs = 254;
		}
	}
	write8bit(tof, VL6180X_SYSALS_INTERMEASUREMENT_PERIOD, periodMs);
}

/**
 * @brief  turn on interleaved mode
 */
void VL6180X_start_interleaved_mode(VL6180X_t *tof)
{
	stop_measurement(tof);
	write8bit(tof, VL6180X_INTERLEAVED_MODE_ENABLE, 0x01);
	tof->_ALSStartReg.startstop = 1;
	tof->_ALSStartReg.select = 1;
	write8bit(tof, VL6180X_SYSALS_START, *((uint8_t*) (&tof->_ALSStartReg)));
}

/**
 * @brief  Get the interrupt state of the ranging
 * @return   return status
 * @n             0                          No threshold events reported
 * @n             VL6180X_LEVEL_LOW         value < thresh_low
 * @n             VL6180X_LEVEL_HIGH        value > thresh_high
 * @n             VL6180X_OUT_OF_WINDOW     value < thresh_low OR value > thresh_high
 * @n             VL6180X_NEW_SAMPLE_READY  new sample ready
 */
uint8_t VL6180X_range_get_interrupt_status(VL6180X_t *tof)
{
	return (read(tof, VL6180X_RESULT_INTERRUPT_STATUS_GPIO, 1) & 0x07);
}

/**
 * @brief  Get the interrupt state of the measured light intensity
 * @return   return status
 * @n             0                          No threshold events reported
 * @n             VL6180X_LEVEL_LOW         value < thresh_low
 * @n             VL6180X_LEVEL_HIGH        value > thresh_high
 * @n             VL6180X_OUT_OF_WINDOW     value < thresh_low OR value > thresh_high
 * @n             VL6180X_NEW_SAMPLE_READY  new sample ready
 */
uint8_t VL6180X_als_get_interrupt_status(VL6180X_t *tof)
{
	return ((read(tof, VL6180X_RESULT_INTERRUPT_STATUS_GPIO, 1) >> 3) & 0x07);
}

/**
 * @brief  Get validation information for range data
 * @return Authentication information
 */
uint8_t VL6180X_get_range_result(VL6180X_t *tof)
{
	return read(tof, VL6180X_RESULT_RANGE_STATUS, 1) >> 4;
}

/**
 * @brief  set IIC addr
 * @param  newAddress  The IIC address to be modified
 */
void VL6180X_set_i2c_address(VL6180X_t *tof, const uint8_t newAddress)
{
	write8bit(tof, VL6180X_I2C_SLAVE_DEVICE_ADDRESS, newAddress);
	tof->_deviceAddr = newAddress;
}

/**
 * @brief  Set the ALS gain
 * @param  gain  the value of gain(range 0-7)
 * @n            20   times gain   VL6180X_ALS_GAIN_20        0
 * @n            10   times gain   VL6180X_ALS_GAIN_10        1
 * @n            5    times gain   VL6180X_ALS_GAIN_5         2
 * @n            2.5  times gain   VL6180X_ALS_GAIN_2_5       3
 * @n            1.57 times gain   VL6180X_ALS_GAIN_1_67      4
 * @n            1.27 times gain   VL6180X_ALS_GAIN_1_25      5
 * @n            1    times gain   VL6180X_ALS_GAIN_1         6
 * @n            40   times gain   VL6180X_ALS_GAIN_40        7
 * @return true :Set up the success, false :Setup faile
 */
bool VL6180X_set_als_gain(VL6180X_t *tof, const uint8_t gain)
{
	if (gain > 7) {
		return false;
	}
	else {
		tof->_analogueGainReg.gain = gain;
		switch (gain)
		{
		case VL6180X_ALS_GAIN_20:
			tof->_gain = 20;
			break;
		case VL6180X_ALS_GAIN_10:
			tof->_gain = 10;
			break;
		case VL6180X_ALS_GAIN_5:
			tof->_gain = 5.0;
			break;
		case VL6180X_ALS_GAIN_2_5:
			tof->_gain = 2.5;
			break;
		case VL6180X_ALS_GAIN_1_67:
			tof->_gain = 1.67;
			break;
		case VL6180X_ALS_GAIN_1_25:
			tof->_gain = 1.25;
			break;
		case VL6180X_ALS_GAIN_1:
			tof->_gain = 1.0;
			break;
		case VL6180X_ALS_GAIN_40:
			tof->_gain = 40;
			break;
		}
		write8bit(tof, VL6180X_SYSALS_ANALOGUE_GAIN, *((uint8_t*) (&tof->_analogueGainReg)));
	}
	return true;
}

/**
 * @brief  Set ALS Threshold Value
 * @param  thresholdL Lower Threshold
 * @param  thresholdH Upper threshold
 */
void VL6180X_set_als_threshold_value(VL6180X_t *tof, const uint16_t thresholdL, const uint16_t thresholdH)
{
	float valueL = (thresholdL * tof->_gain) / 0.32;
	float valueH = (thresholdH * tof->_gain) / 0.32;
	write16bit(tof, VL6180X_SYSALS_THRESH_LOW, (uint16_t) valueL);
	write16bit(tof, VL6180X_SYSALS_THRESH_HIGH, (uint16_t) valueH);
}

/**
 * @brief  Set Range Threshold Value
 * @param  thresholdL Lower Threshold
 * @param  thresholdH Upper threshold
 */
void VL6180X_set_range_threshold_value(VL6180X_t *tof, const uint16_t thresholdL, const uint16_t thresholdH)
{
	write8bit(tof, VL6180X_SYSRANGE_THRESH_LOW, thresholdL);
	write8bit(tof, VL6180X_SYSRANGE_THRESH_HIGH, thresholdH);
}

void VL6180X_set_fresh_reset(VL6180X_t *tof, const uint8_t reset_state)
{
	write8bit(tof, VL6180X_SYSTEM_FRESH_OUT_OF_RESET, reset_state);
}
