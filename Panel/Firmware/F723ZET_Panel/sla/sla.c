/*
 * sla.c
 *
 *  Created on: Jun 24, 2022
 *      Author: 62812
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "sla.h"
#include "usart.h"

#if DEBUG_SLA_ENABLE==1
#define LOG(str, ...) printf("[%ld %s:%d] " str, HAL_GetTick()%10000, __FILE_NAME__, __LINE__, ##__VA_ARGS__)
#define LOG_E(str, ...) printf("[%s Err:%d] " str,  __FILE_NAME__,__LINE__, ##__VA_ARGS__)
#else
#define LOG(str, ...)
#define LOG_E(str, ...)
#endif	//if DEBUG_SLA_ENABLE==1

typedef struct
{
	float rws_imu[2];
	float rws_body[2];
	uint16_t lrf_val;
} sla_t;

typedef struct
{
	uint8_t active_id;
	int16_t dx;
	int16_t dy;
	uint8_t new_data;
} tracker_data_t;

typedef struct
{
	uint8_t available;
	uint8_t counter;
	int32_t az;
	int32_t el;
	uint16_t distance;
	uint32_t timestamp;
} gsl_command_t;

static sla_t sla;
static gsl_command_t gsl = { 0, 0, 0, 0, 0 };
static tracker_data_t tracker_data = { .new_data = 0 };

void sla_set_attitude(const float az, const float el)
{
	sla.rws_body[0] = az;
	sla.rws_body[1] = el;
}

void sla_set_distance(const uint16_t d)
{
	sla.lrf_val = d;
}

void sla_init()
{
	tracker_init();
}

uint8_t sla_gsl_available()
{
	return gsl.available;
}

void sla_gsl_get_command(uint8_t *counter, int32_t *az, int32_t *el, uint16_t *distance)
{
	*counter = gsl.counter;
	*az = gsl.az;
	*el = gsl.el;
	*distance = gsl.distance;

	gsl.available = 0;
}

void sla_gsl_start()
{
	uint16_t bufLen;
	char buf[RING_BUFFER_TX_SIZE];

	bufLen = sprintf(buf, "$GSLACK,1*");
	sla_print_text(buf, bufLen);
}

static char** str_split(char *a_str, const char a_delim)
{
	char **result = 0;
	size_t count = 0;
	char *tmp = a_str;
	char *last_comma = 0;
	char delim[2];
	delim[0] = a_delim;
	delim[1] = 0;

	/* Count how many elements will be extracted. */
	while (*tmp) {
		if (a_delim == *tmp) {
			count++;
			last_comma = tmp;
		}
		tmp++;
	}

	/* Add space for trailing token. */
	count += last_comma < (a_str + strlen(a_str) - 1);

	/* Add space for terminating null string so caller
	 knows where the list of returned strings ends. */
	count++;

	result = malloc(sizeof(char*) * count);

	if (result) {
		size_t idx = 0;
		char *token = strtok(a_str, delim);

		while (token) {
			assert(idx < count);
			*(result + idx++) = strdup(token);
			token = strtok(0, delim);
		}
		assert(idx == count - 1);
		*(result + idx) = 0;
	}

	return result;
}

static void sla_parsing(char *cmd)
{
	char *s;
	char **tokens;
	float f;

	LOG("pcE: %s\r\n", cmd);
	s = strstr(cmd, "$GSL,");
	if (s) {
		tokens = str_split(cmd, ',');
		if (tokens) {
			for ( int i = 0; *(tokens + i); i++ ) {
				s = *(tokens + i);

				if (i == 1)
					gsl.counter = atoi(s);
				else if (i == 2) {
					f = atof(s);
					gsl.az = (int32_t) (f * 1000);
				}
				else if (i == 3) {
					f = atof(s);
					gsl.el = (int32_t) (f * 1000);
				}
				else if (i == 4)
					gsl.distance = (uint16_t) atoi(s);

				free(*(tokens + i));
			}
			free(tokens);

			gsl.available = 1;
			gsl.timestamp = HAL_GetTick();
		}
	}

	s = strstr(cmd, "$TRKUD,");
	if (s) {
		tokens = str_split(cmd, ',');
		if (tokens) {
			for ( int i = 0; *(tokens + i); i++ ) {
				s = *(tokens + i);

				if (i == 1)
					tracker_data.active_id = atoi(s);
				else if (i == 2)
					tracker_data.dx = atoi(s);
				else if (i == 3)
					tracker_data.dy = atoi(s);

				free(*(tokens + i));
			}
			free(tokens);

			tracker_data.new_data = 1;
		}
	}
}

void sla_handler()
{
	static uint32_t send_timer = 0;
	char c[2] = { 0, 0 };

	static char pce_buf[RING_BUFFER_RX_SIZE];
	uint8_t pce_completed = 0;

	uint16_t bufLen = 0;
	char buf[RING_BUFFER_TX_SIZE];

	if (serial_available(&pcE) > 0) {
		c[0] = serial_read(&pcE);

		if (c[0] == '$')
			memset(pce_buf, 0, RING_BUFFER_RX_SIZE);
		else if (c[0] == '*')
			pce_completed = 1;

		strcat(pce_buf, c);
	}

	if (pce_completed == 1)
		sla_parsing(pce_buf);

	if (HAL_GetTick() >= send_timer) {
		send_timer = HAL_GetTick() + 250;

		bufLen = sprintf(buf, "$LRVAL,%d*$AZVAL,%.2f*$ELVAL,%.3f*", sla.lrf_val, sla.rws_body[0], sla.rws_body[1]);
		sla_print_text(buf, bufLen);
	}

	if (HAL_GetTick() >= gsl.timestamp + 3000) {
		gsl.available = 0;
		gsl.az = gsl.el = gsl.distance = 0;
	}
}

void tracker_init()
{
	tracker_data.active_id = 0;
	tracker_data.dx = tracker_data.dy = 0;
	tracker_data.new_data = 0;
}

HAL_StatusTypeDef tracker_available()
{
	if (tracker_data.new_data == 1)
		return HAL_OK;

	return HAL_ERROR;
}

HAL_StatusTypeDef tracker_read(uint8_t *id, int16_t *dx, int16_t *dy)
{
	if (tracker_available() == HAL_OK) {
		*id = tracker_data.active_id;
		*dx = tracker_data.dx;
		*dy = tracker_data.dy;

		tracker_data.new_data = 0;
		return HAL_OK;
	}

	return HAL_ERROR;
}

void tracker_change_gate_size(const int8_t resize)
{
	char buf[16];
	uint16_t bufLen;
	int8_t size = TRK_GATE_NONE;

	if (resize > 0)
		size = TRK_GATE_BIGGER;
	else if (resize < 0)
		size = TRK_GATE_SMALLER;

	bufLen = sprintf(buf, "$GATE,%d*", size);
	sla_print_text(buf, bufLen);
}

void tracker_change_target(const int8_t retarget)
{
	char buf[16];
	uint16_t bufLen;
	int8_t select = TRK_SEL_NONE;

	if (retarget > TRK_SEL_NONE)
		select = TRK_SEL_NEXT;
	else if (retarget < TRK_SEL_NONE)
		select = TRK_SEL_PREV;

	bufLen = sprintf(buf, "$TRKSEL,%d*", select);
	sla_print_text(buf, bufLen);
}

void sla_print_text(const char *str, const uint16_t len)
{
	serial_write_str(&pcE, str, len);
}
