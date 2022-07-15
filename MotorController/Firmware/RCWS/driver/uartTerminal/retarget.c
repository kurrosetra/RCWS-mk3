/*
 * retarget.c
 *
 *  Created on: Mar 31, 2021
 *      Author: miftakur
 */

#include <stdio.h>

#include "main.h"
#include "uartTerminal.h"

void retarget_init()
{
	uartTerminal_init(&huart1);
	setbuf(stdin, NULL);
	setbuf(stdout, NULL);
	setbuf(stderr, NULL);
}

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

PUTCHAR_PROTOTYPE
{
	uartTerminal_send(ch);

	return 1;
}

#ifdef __GNUC__
#define GETCHAR_PROTOTYPE int __io_getchar(void)
#else
#define GETCHAR_PROTOTYPE int fgetc(FILE *f)
#endif /* __GNUC__ */

GETCHAR_PROTOTYPE
{
	return uartTerminal_recv();
}
