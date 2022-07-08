/*
 * retarget.c
 *
 *  Created on: Mar 31, 2021
 *      Author: miftakur
 */

#include <stdio.h>

#include "main.h"
#include "stm_hal_serial.h"

//#define LOG(str, ...) printf("[%ld %s:%d] " str, HAL_GetTick(), __FILE_NAME__, __LINE__, ##__VA_ARGS__)
//#define LOG_E(str, ...) printf("[%s Err:%d] " str,  __FILE_NAME__,__LINE__, ##__VA_ARGS__)

TSerial *terminal;

void retarget_init(TSerial *serial)
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
	serial_write(terminal, c);

	return 1;
}

uint8_t retarget_recv()
{
	while (serial_available(terminal) == 0)
		HAL_Delay(10);

	return serial_read(terminal);
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
