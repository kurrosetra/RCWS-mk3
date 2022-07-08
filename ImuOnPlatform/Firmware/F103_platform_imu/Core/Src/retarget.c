/*
 * retarget.c
 *
 *  Created on: Mar 31, 2021
 *      Author: miftakur
 */

#include <stdio.h>

#include "main.h"

//#define LOG(str, ...) printf("[%ld %s:%d] " str, HAL_GetTick(), __FILENAME__, __LINE__, ##__VA_ARGS__)
//#define LOG_E(str, ...) printf("[%s Err:%d] " str,  __FILENAME__,__LINE__, ##__VA_ARGS__)

UART_HandleTypeDef *terminal;

void retarget_init(UART_HandleTypeDef *serial)
{
	setbuf(stdin, NULL);
	setbuf(stdout, NULL);
	setbuf(stderr, NULL);

	terminal = serial;
}

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int c)
#else
#define PUTCHAR_PROTOTYPE int fputc(int c, FILE *f)
#endif /* __GNUC__ */

PUTCHAR_PROTOTYPE
{
//	serial_write(terminal, c);
//	HAL_UART_Transmit_IT(terminal, (uint8_t*) &c, 1);

	HAL_UART_Transmit(terminal, (uint8_t*) &c, 1, 1);

	return 1;
}

uint8_t retarget_recv()
{
//	while(serial_available(terminal)==0)
//		HAL_Delay(10);
//
//	return serial_read(terminal);
	uint8_t u8;
	while (HAL_UART_Receive(terminal, &u8, 1, 10) != HAL_OK)
		;

	return u8;
}

#ifdef __GNUC__
#define GETCHAR_PROTOTYPE int __io_getchar(void)
#else
#define GETCHAR_PROTOTYPE int fgetc(FILE *f)
#endif /* __GNUC__ */

GETCHAR_PROTOTYPE
{
	return retarget_recv();
}
