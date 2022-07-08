/*
 * pc.c
 *
 *  Created on: Dec 9, 2021
 *      Author: miftakur
 */

#include <stdio.h>
#include <string.h>

#include "pc.h"
#include "bus_button_config.h"

PC_t comp;
uint8_t rxPcBuffer[RING_BUFFER_RX_SIZE];
uint8_t txPcBuffer[RING_BUFFER_TX_SIZE];
TSerial pcSerial;

uint32_t sla_time_on = 0;

static void sla_power(const GPIO_PinState state)
{
	HAL_GPIO_WritePin(SLA_POWER_EN_GPIO_Port, SLA_POWER_EN_Pin, state);
}

void pc_init(void)
{
	comp.port = &pcSerial;
	serial_init(comp.port, (char*) &rxPcBuffer, sizeof(rxPcBuffer), (char*) &txPcBuffer,
			sizeof(txPcBuffer), &hlpuart1);

	comp.send.lrfRange = 1000UL;
	comp.send.munCounter = 0;
	*(uint8_t*) &comp.send.state = 1;
//	/* turn on the sightline */
//	sla_power(GPIO_PIN_SET);
	sla_time_on = HAL_GetTick() + 60000;
}

static int pc_parsing_data(const char *str)
{
	int ret = -1;
	char *s;

	s = strstr(str, "$TRKUD,");
	if (s) {

		ret = 1;
	}

	return ret;
}

void pc_handler(void)
{
	/* send vars */
	char pcSend[RING_BUFFER_TX_SIZE];
	uint16_t pcLen = 0;
	static uint32_t pcTimer = 0;
	/* recv vars */
	uint8_t pcCompleted = 0;
	static char pcStr[RING_BUFFER_RX_SIZE];
	char c[2] = { 0, 0 };
	int parsingRet = 0;

#if PC_DEBUG==1
	char buf[RING_BUFFER_TX_SIZE];
	uint16_t bufLen;
#endif	//if PC_DEBUG==1

	if (sla_time_on > 0) {
		if (HAL_GetTick() >= sla_time_on) {
			sla_time_on = 0;
			sla_power(GPIO_PIN_SET);
		}
	}
	else {

		if (serial_available(comp.port) > 0) {
			c[0] = serial_read(comp.port);

			if (c[0] == '$')
				memset(pcStr, 0, RING_BUFFER_RX_SIZE);
			else if (c[0] == '*')
				pcCompleted = 1;

			strcat(pcStr, c);
		}

		if (pcCompleted == 1) {
#if PC_DEBUG==1
			bufLen = sprintf(buf, "[PC]: %s\r\n", pcStr);
			serial_write_str(&debug, buf, bufLen);
#endif	//if PC_DEBUG==1

			parsingRet = pc_parsing_data(pcStr);
			if (parsingRet > 0) {

			}
		}

		if (HAL_GetTick() >= pcTimer) {
			pcTimer = HAL_GetTick() + 500;

//			static uint16_t munCounter = 0;
//			pcLen = sprintf(pcSend, "$LRVAL,%d*$DISPSTR,1,5,30,MUN=%d*", comp.send.lrfRange,
//					munCounter++);
			pcLen = sprintf(pcSend, "$LRVAL,d=%dm*$DISPSTR,1,5,30,MUN=%d*", comp.send.lrfRange,
					comp.send.munCounter);
			serial_write_str(comp.port, pcSend, pcLen);
#if PC_DEBUG==1
			serial_write_str(&debug, pcSend, pcLen);
#endif	//if PC_DEBUG==1

			pcLen = sprintf(pcSend, "$AZVAL,%.2f*$ELVAL,%.2f*", (float) comp.send.angleAz / 100,
					(float) comp.send.angleEl / 100);
			serial_write_str(comp.port, pcSend, pcLen);
		}
	}
}

void LPUART1_IRQHandler(void)
{
	USARTx_IRQHandler(&pcSerial);
}

