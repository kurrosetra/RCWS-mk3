/*
 * visca.h
 *
 *  Created on: Dec 29, 2021
 *      Author: miftakur
 */

#ifndef DRIVER_LIBVISCA_120_VISCA_H_
#define DRIVER_LIBVISCA_120_VISCA_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "visca_conf.h"
#include "visca_def.h"

/* timeout in ms */
#define VISCA_SERIAL_LONG_WAIT				5000
#define VISCA_SERIAL_WAIT					100

/* size of the local packet buffer */
#define VISCA_INPUT_BUFFER_SIZE				32

/* This is the interface for the STM platform.
 */
typedef struct
{
	// RS232 data:
	UART_HandleTypeDef *port_fd;

	// VISCA data:
	int address;
	int broadcast;

	// RS232 input buffer
	uint8_t ibuf[VISCA_INPUT_BUFFER_SIZE];
	uint32_t bytes;
	uint32_t type;

#if VISCA_USE_UART_DMA
	volatile uint8_t s_bufferRx[VISCA_INPUT_BUFFER_SIZE];
	volatile uint8_t s_bufferRxRp;
#endif	//if VISCA_USE_UART_DMA
} VISCAInterface_t;

/* CAMERA STRUCTURE */
typedef struct
{
	// VISCA data:
	int address;

	// camera info:
	uint32_t vendor;
	uint32_t model;
	uint32_t rom_version;
	uint32_t socket_num;

} VISCACamera_t;

/* TITLE STRUCTURE */
typedef struct
{
	uint32_t vposition;
	uint32_t hposition;
	uint32_t color;
	uint32_t blink;
	unsigned char title[20];

} VISCATitleData_t;

typedef struct
{
	unsigned char bytes[32];
	uint32_t length;
} VISCAPacket_t;

/* GENERAL FUNCTIONS */

uint32_t
VISCA_set_address(VISCAInterface_t *iface, int *camera_num);

uint32_t
VISCA_clear(VISCAInterface_t *iface, VISCACamera_t *camera);

uint32_t
VISCA_get_camera_info(VISCAInterface_t *iface, VISCACamera_t *camera);

uint32_t
_VISCA_write_packet_data(VISCAInterface_t *iface, VISCACamera_t *camera, VISCAPacket_t *packet);

uint32_t
_VISCA_send_packet(VISCAInterface_t *iface, VISCACamera_t *camera, VISCAPacket_t *packet);

uint32_t
_VISCA_get_packet(VISCAInterface_t *iface);

uint32_t
VISCA_unread_bytes(VISCAInterface_t *iface, unsigned char *buffer, uint32_t *buffer_size);

/* COMMANDS */

uint32_t
VISCA_set_power(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t power);

uint32_t
VISCA_get_info(VISCAInterface_t *iface, VISCACamera_t *camera);

uint32_t
VISCA_set_keylock(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t power);

uint32_t
VISCA_set_camera_id(VISCAInterface_t *iface, VISCACamera_t *camera, uint16_t id);

uint32_t
VISCA_set_zoom_tele(VISCAInterface_t *iface, VISCACamera_t *camera);

uint32_t
VISCA_set_zoom_wide(VISCAInterface_t *iface, VISCACamera_t *camera);

uint32_t
VISCA_set_zoom_stop(VISCAInterface_t *iface, VISCACamera_t *camera);

uint32_t
VISCA_set_zoom_tele_speed(VISCAInterface_t *iface, VISCACamera_t *camera, uint32_t speed);

uint32_t
VISCA_set_zoom_wide_speed(VISCAInterface_t *iface, VISCACamera_t *camera, uint32_t speed);

uint32_t
VISCA_set_zoom_value(VISCAInterface_t *iface, VISCACamera_t *camera, uint32_t zoom);

uint32_t
VISCA_set_zoom_and_focus_value(VISCAInterface_t *iface, VISCACamera_t *camera, uint32_t zoom,
		uint32_t focus);

uint32_t
VISCA_set_dzoom(VISCAInterface_t *iface, VISCACamera_t *camera, uint32_t power);

uint32_t
VISCA_set_dzoom_limit(VISCAInterface_t *iface, VISCACamera_t *camera, uint32_t limit);

uint32_t
VISCA_set_dzoom_mode(VISCAInterface_t *iface, VISCACamera_t *camera, uint32_t power);

uint32_t
VISCA_set_focus_far(VISCAInterface_t *iface, VISCACamera_t *camera);

uint32_t
VISCA_set_focus_near(VISCAInterface_t *iface, VISCACamera_t *camera);

uint32_t
VISCA_set_focus_stop(VISCAInterface_t *iface, VISCACamera_t *camera);

uint32_t
VISCA_set_focus_far_speed(VISCAInterface_t *iface, VISCACamera_t *camera, uint32_t speed);

uint32_t
VISCA_set_focus_near_speed(VISCAInterface_t *iface, VISCACamera_t *camera, uint32_t speed);

uint32_t
VISCA_set_focus_value(VISCAInterface_t *iface, VISCACamera_t *camera, uint32_t focus);

uint32_t
VISCA_set_focus_auto(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t power);

uint32_t
VISCA_set_focus_one_push(VISCAInterface_t *iface, VISCACamera_t *camera);

uint32_t
VISCA_set_focus_infinity(VISCAInterface_t *iface, VISCACamera_t *camera);

uint32_t
VISCA_set_focus_autosense_high(VISCAInterface_t *iface, VISCACamera_t *camera);

uint32_t
VISCA_set_focus_autosense_low(VISCAInterface_t *iface, VISCACamera_t *camera);

uint32_t
VISCA_set_focus_near_limit(VISCAInterface_t *iface, VISCACamera_t *camera, uint32_t limit);

uint32_t
VISCA_set_whitebal_mode(VISCAInterface_t *iface, VISCACamera_t *camera, uint32_t mode);

uint32_t
VISCA_set_whitebal_one_push(VISCAInterface_t *iface, VISCACamera_t *camera);

uint32_t
VISCA_set_rgain_up(VISCAInterface_t *iface, VISCACamera_t *camera);

uint32_t
VISCA_set_rgain_down(VISCAInterface_t *iface, VISCACamera_t *camera);

uint32_t
VISCA_set_rgain_reset(VISCAInterface_t *iface, VISCACamera_t *camera);

uint32_t
VISCA_set_rgain_value(VISCAInterface_t *iface, VISCACamera_t *camera, uint32_t value);

uint32_t
VISCA_set_bgain_up(VISCAInterface_t *iface, VISCACamera_t *camera);

uint32_t
VISCA_set_bgain_down(VISCAInterface_t *iface, VISCACamera_t *camera);

uint32_t
VISCA_set_bgain_reset(VISCAInterface_t *iface, VISCACamera_t *camera);

uint32_t
VISCA_set_bgain_value(VISCAInterface_t *iface, VISCACamera_t *camera, uint32_t value);

uint32_t
VISCA_set_shutter_up(VISCAInterface_t *iface, VISCACamera_t *camera);

uint32_t
VISCA_set_shutter_down(VISCAInterface_t *iface, VISCACamera_t *camera);

uint32_t
VISCA_set_shutter_reset(VISCAInterface_t *iface, VISCACamera_t *camera);

uint32_t
VISCA_set_shutter_value(VISCAInterface_t *iface, VISCACamera_t *camera, uint32_t value);

uint32_t
VISCA_set_iris_up(VISCAInterface_t *iface, VISCACamera_t *camera);

uint32_t
VISCA_set_iris_down(VISCAInterface_t *iface, VISCACamera_t *camera);

uint32_t
VISCA_set_iris_reset(VISCAInterface_t *iface, VISCACamera_t *camera);

uint32_t
VISCA_set_iris_value(VISCAInterface_t *iface, VISCACamera_t *camera, uint32_t value);

uint32_t
VISCA_set_gain_up(VISCAInterface_t *iface, VISCACamera_t *camera);

uint32_t
VISCA_set_gain_down(VISCAInterface_t *iface, VISCACamera_t *camera);

uint32_t
VISCA_set_gain_reset(VISCAInterface_t *iface, VISCACamera_t *camera);

uint32_t
VISCA_set_gain_value(VISCAInterface_t *iface, VISCACamera_t *camera, uint32_t value);

uint32_t
VISCA_set_bright_up(VISCAInterface_t *iface, VISCACamera_t *camera);

uint32_t
VISCA_set_bright_down(VISCAInterface_t *iface, VISCACamera_t *camera);

uint32_t
VISCA_set_bright_reset(VISCAInterface_t *iface, VISCACamera_t *camera);

uint32_t
VISCA_set_bright_value(VISCAInterface_t *iface, VISCACamera_t *camera, uint32_t value);

uint32_t
VISCA_set_aperture_up(VISCAInterface_t *iface, VISCACamera_t *camera);

uint32_t
VISCA_set_aperture_down(VISCAInterface_t *iface, VISCACamera_t *camera);

uint32_t
VISCA_set_aperture_reset(VISCAInterface_t *iface, VISCACamera_t *camera);

uint32_t
VISCA_set_aperture_value(VISCAInterface_t *iface, VISCACamera_t *camera, uint32_t value);

uint32_t
VISCA_set_exp_comp_up(VISCAInterface_t *iface, VISCACamera_t *camera);

uint32_t
VISCA_set_exp_comp_down(VISCAInterface_t *iface, VISCACamera_t *camera);

uint32_t
VISCA_set_exp_comp_reset(VISCAInterface_t *iface, VISCACamera_t *camera);

uint32_t
VISCA_set_exp_comp_value(VISCAInterface_t *iface, VISCACamera_t *camera, uint32_t value);

uint32_t
VISCA_set_exp_comp_power(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t power);

uint32_t
VISCA_set_auto_exp_mode(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t mode);

uint32_t
VISCA_set_slow_shutter_auto(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t power);

uint32_t
VISCA_set_backlight_comp(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t power);

uint32_t
VISCA_set_zero_lux_shot(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t power);

uint32_t
VISCA_set_ir_led(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t power);

uint32_t
VISCA_set_wide_mode(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t mode);

uint32_t
VISCA_set_mirror(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t power);

uint32_t
VISCA_set_freeze(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t power);

uint32_t
VISCA_set_picture_effect(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t mode);

uint32_t
VISCA_set_digital_effect(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t mode);

uint32_t
VISCA_set_digital_effect_level(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t level);

uint32_t
VISCA_memory_set(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t channel);

uint32_t
VISCA_set_cam_stabilizer(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t power);

uint32_t
VISCA_memory_recall(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t channel);

uint32_t
VISCA_memory_reset(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t channel);

uint32_t
VISCA_set_display(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t power);

uint32_t
VISCA_set_date_time(VISCAInterface_t *iface, VISCACamera_t *camera, uint32_t year, uint32_t month,
		uint32_t day, uint32_t hour, uint32_t minute);

uint32_t
VISCA_set_date_display(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t power);

uint32_t
VISCA_set_time_display(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t power);

uint32_t
VISCA_set_title_display(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t power);

uint32_t
VISCA_set_title_clear(VISCAInterface_t *iface, VISCACamera_t *camera);

uint32_t
VISCA_set_title_params(VISCAInterface_t *iface, VISCACamera_t *camera, VISCATitleData_t *title);

uint32_t
VISCA_set_title(VISCAInterface_t *iface, VISCACamera_t *camera, VISCATitleData_t *title);

uint32_t
VISCA_set_irreceive_on(VISCAInterface_t *iface, VISCACamera_t *camera);

uint32_t
VISCA_set_irreceive_off(VISCAInterface_t *iface, VISCACamera_t *camera);

uint32_t
VISCA_set_irreceive_onoff(VISCAInterface_t *iface, VISCACamera_t *camera);

/*  pan_speed should be in the range 01 - 18.
 tilt_speed should be in the range 01 - 14 */

uint32_t
VISCA_set_pantilt_up(VISCAInterface_t *iface, VISCACamera_t *camera, uint32_t pan_speed,
		uint32_t tilt_speed);

uint32_t
VISCA_set_pantilt_down(VISCAInterface_t *iface, VISCACamera_t *camera, uint32_t pan_speed,
		uint32_t tilt_speed);

uint32_t
VISCA_set_pantilt_left(VISCAInterface_t *iface, VISCACamera_t *camera, uint32_t pan_speed,
		uint32_t tilt_speed);

uint32_t
VISCA_set_pantilt_right(VISCAInterface_t *iface, VISCACamera_t *camera, uint32_t pan_speed,
		uint32_t tilt_speed);

uint32_t
VISCA_set_pantilt_upleft(VISCAInterface_t *iface, VISCACamera_t *camera, uint32_t pan_speed,
		uint32_t tilt_speed);

uint32_t
VISCA_set_pantilt_upright(VISCAInterface_t *iface, VISCACamera_t *camera, uint32_t pan_speed,
		uint32_t tilt_speed);

uint32_t
VISCA_set_pantilt_downleft(VISCAInterface_t *iface, VISCACamera_t *camera, uint32_t pan_speed,
		uint32_t tilt_speed);

uint32_t
VISCA_set_pantilt_downright(VISCAInterface_t *iface, VISCACamera_t *camera, uint32_t pan_speed,
		uint32_t tilt_speed);

uint32_t
VISCA_set_pantilt_stop(VISCAInterface_t *iface, VISCACamera_t *camera, uint32_t pan_speed,
		uint32_t tilt_speed);

/*  pan_speed should be in the range 01 - 18.
 tilt_speed should be in the range 01 - 14
 pan_position should be in the range -880 - 880 (0xFC90 - 0x370)
 tilt_position should be in range -300 - 300 (0xFED4 - 0x12C)  */
uint32_t
VISCA_set_pantilt_absolute_position(VISCAInterface_t *iface, VISCACamera_t *camera,
		uint32_t pan_speed, uint32_t tilt_speed, int pan_position, int tilt_position);

uint32_t
VISCA_set_pantilt_relative_position(VISCAInterface_t *iface, VISCACamera_t *camera,
		uint32_t pan_speed, uint32_t tilt_speed, int pan_position, int tilt_position);

uint32_t
VISCA_set_pantilt_home(VISCAInterface_t *iface, VISCACamera_t *camera);

uint32_t
VISCA_set_pantilt_reset(VISCAInterface_t *iface, VISCACamera_t *camera);

/*  pan_limit should be in the range -880 - 880 (0xFC90 - 0x370)
 tilt_limit should be in range -300 - 300 (0xFED4 - 0x12C)  */
uint32_t
VISCA_set_pantilt_limit_upright(VISCAInterface_t *iface, VISCACamera_t *camera, int pan_limit,
		int tilt_limit);

uint32_t
VISCA_set_pantilt_limit_downleft(VISCAInterface_t *iface, VISCACamera_t *camera, int pan_limit,
		int tilt_limit);

uint32_t
VISCA_set_pantilt_limit_downleft_clear(VISCAInterface_t *iface, VISCACamera_t *camera);

uint32_t
VISCA_set_pantilt_limit_upright_clear(VISCAInterface_t *iface, VISCACamera_t *camera);

uint32_t
VISCA_set_datascreen_on(VISCAInterface_t *iface, VISCACamera_t *camera);

uint32_t
VISCA_set_datascreen_off(VISCAInterface_t *iface, VISCACamera_t *camera);

uint32_t
VISCA_set_datascreen_onoff(VISCAInterface_t *iface, VISCACamera_t *camera);

uint32_t
VISCA_set_spot_ae_on(VISCAInterface_t *iface, VISCACamera_t *camera);

uint32_t
VISCA_set_spot_ae_off(VISCAInterface_t *iface, VISCACamera_t *camera);

uint32_t
VISCA_set_spot_ae_position(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t x_position,
		uint8_t y_position);

/* INQUIRIES */

uint32_t
VISCA_get_power(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t *power);

uint32_t
VISCA_get_dzoom(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t *power);

uint32_t
VISCA_get_dzoom_limit(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t *value);

uint32_t
VISCA_get_zoom_value(VISCAInterface_t *iface, VISCACamera_t *camera, uint16_t *value);

uint32_t
VISCA_get_focus_auto(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t *power);

uint32_t
VISCA_get_focus_value(VISCAInterface_t *iface, VISCACamera_t *camera, uint16_t *value);

uint32_t
VISCA_get_focus_auto_sense(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t *mode);

uint32_t
VISCA_get_focus_near_limit(VISCAInterface_t *iface, VISCACamera_t *camera, uint16_t *value);

uint32_t
VISCA_get_whitebal_mode(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t *mode);

uint32_t
VISCA_get_rgain_value(VISCAInterface_t *iface, VISCACamera_t *camera, uint16_t *value);

uint32_t
VISCA_get_bgain_value(VISCAInterface_t *iface, VISCACamera_t *camera, uint16_t *value);

uint32_t
VISCA_get_auto_exp_mode(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t *mode);

uint32_t
VISCA_get_slow_shutter_auto(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t *mode);

uint32_t
VISCA_get_shutter_value(VISCAInterface_t *iface, VISCACamera_t *camera, uint16_t *value);

uint32_t
VISCA_get_iris_value(VISCAInterface_t *iface, VISCACamera_t *camera, uint16_t *value);

uint32_t
VISCA_get_gain_value(VISCAInterface_t *iface, VISCACamera_t *camera, uint16_t *value);

uint32_t
VISCA_get_bright_value(VISCAInterface_t *iface, VISCACamera_t *camera, uint16_t *value);

uint32_t
VISCA_get_exp_comp_power(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t *power);

uint32_t
VISCA_get_exp_comp_value(VISCAInterface_t *iface, VISCACamera_t *camera, uint16_t *value);

uint32_t
VISCA_get_backlight_comp(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t *power);

uint32_t
VISCA_get_aperture_value(VISCAInterface_t *iface, VISCACamera_t *camera, uint16_t *value);

uint32_t
VISCA_get_zero_lux_shot(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t *power);

uint32_t
VISCA_get_ir_led(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t *power);

uint32_t
VISCA_get_wide_mode(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t *mode);

uint32_t
VISCA_get_mirror(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t *power);

uint32_t
VISCA_get_freeze(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t *power);

uint32_t
VISCA_get_picture_effect(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t *mode);

uint32_t
VISCA_get_digital_effect(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t *mode);

uint32_t
VISCA_get_digital_effect_level(VISCAInterface_t *iface, VISCACamera_t *camera, uint16_t *value);

uint32_t
VISCA_get_memory(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t *channel);

uint32_t
VISCA_get_display(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t *power);

uint32_t
VISCA_get_id(VISCAInterface_t *iface, VISCACamera_t *camera, uint16_t *id);

uint32_t
VISCA_get_videosystem(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t *system);

uint32_t
VISCA_get_pantilt_mode(VISCAInterface_t *iface, VISCACamera_t *camera, uint16_t *status);

uint32_t
VISCA_get_pantilt_maxspeed(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t *max_pan_speed,
		uint8_t *max_tilt_speed);

uint32_t
VISCA_get_pantilt_position(VISCAInterface_t *iface, VISCACamera_t *camera, int *pan_position,
		int *tilt_position);

uint32_t
VISCA_get_datascreen(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t *status);

/* SPECIAL FUNCTIONS FOR D30/31 */
uint32_t
VISCA_set_wide_con_lens(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t power);

uint32_t
VISCA_set_at_mode_onoff(VISCAInterface_t *iface, VISCACamera_t *camera);

uint32_t
VISCA_set_at_mode(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t power);

uint32_t
VISCA_set_at_ae_onoff(VISCAInterface_t *iface, VISCACamera_t *camera);

uint32_t
VISCA_set_at_ae(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t power);

uint32_t
VISCA_set_at_autozoom_onoff(VISCAInterface_t *iface, VISCACamera_t *camera);

uint32_t
VISCA_set_at_autozoom(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t power);

uint32_t
VISCA_set_atmd_framedisplay_onoff(VISCAInterface_t *iface, VISCACamera_t *camera);

uint32_t
VISCA_set_atmd_framedisplay(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t power);

uint32_t
VISCA_set_at_frameoffset_onoff(VISCAInterface_t *iface, VISCACamera_t *camera);

uint32_t
VISCA_set_at_frameoffset(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t power);

uint32_t
VISCA_set_atmd_startstop(VISCAInterface_t *iface, VISCACamera_t *camera);

uint32_t
VISCA_set_at_chase(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t power);

uint32_t
VISCA_set_at_chase_next(VISCAInterface_t *iface, VISCACamera_t *camera);

uint32_t
VISCA_set_md_mode_onoff(VISCAInterface_t *iface, VISCACamera_t *camera);

uint32_t
VISCA_set_md_mode(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t power);

uint32_t
VISCA_set_md_frame(VISCAInterface_t *iface, VISCACamera_t *camera);

uint32_t
VISCA_set_md_detect(VISCAInterface_t *iface, VISCACamera_t *camera);

uint32_t VISCA_set_at_entry(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t power);

uint32_t VISCA_set_at_lostinfo(VISCAInterface_t *iface, VISCACamera_t *camera);

uint32_t VISCA_set_md_lostinfo(VISCAInterface_t *iface, VISCACamera_t *camera);

uint32_t VISCA_set_md_adjust_ylevel(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t power);

uint32_t VISCA_set_md_adjust_huelevel(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t power);

uint32_t VISCA_set_md_adjust_size(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t power);

uint32_t VISCA_set_md_adjust_disptime(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t power);

uint32_t VISCA_set_md_adjust_refmode(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t power);

uint32_t VISCA_set_md_adjust_reftime(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t power);

uint32_t VISCA_set_md_measure_mode1_onoff(VISCAInterface_t *iface, VISCACamera_t *camera);

uint32_t VISCA_set_md_measure_mode1(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t power);

uint32_t VISCA_set_md_measure_mode2_onoff(VISCAInterface_t *iface, VISCACamera_t *camera);

uint32_t VISCA_set_md_measure_mode2(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t power);

uint32_t VISCA_get_keylock(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t *power);

uint32_t VISCA_get_wide_con_lens(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t *power);

uint32_t VISCA_get_atmd_mode(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t *power);

uint32_t VISCA_get_at_mode(VISCAInterface_t *iface, VISCACamera_t *camera, uint16_t *value);

uint32_t VISCA_get_at_entry(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t *power);

uint32_t VISCA_get_md_mode(VISCAInterface_t *iface, VISCACamera_t *camera, uint16_t *value);

uint32_t VISCA_get_md_ylevel(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t *power);

uint32_t VISCA_get_md_huelevel(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t *power);

uint32_t VISCA_get_md_size(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t *power);

uint32_t VISCA_get_md_disptime(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t *power);

uint32_t VISCA_get_md_refmode(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t *power);

uint32_t VISCA_get_md_reftime(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t *power);

uint32_t VISCA_get_at_obj_pos(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t *xpos,
		uint8_t *ypos, uint8_t *status);

uint32_t VISCA_get_md_obj_pos(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t *xpos,
		uint8_t *ypos, uint8_t *status);

uint32_t VISCA_set_register(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t reg_num,
		uint8_t reg_val);

uint32_t VISCA_get_register(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t reg_num,
		uint8_t *reg_val);

///* Utility */
//uint32_t VISCA_usleep(uint32_t useconds);

#ifdef __cplusplus
} /* closing brace for extern "C" */
#endif

#endif /* DRIVER_LIBVISCA_120_VISCA_H_ */
