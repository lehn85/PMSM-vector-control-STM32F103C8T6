/*
 * itm.c
 *
 *  Created on: 6 Jul 2017
 *      Author: PhuongLe
 */
#include <stdio.h>
#include <stdarg.h>
#include <stm32f1xx_hal.h>

void ITM_print(char* s)
{
	while (*s)
	{
		ITM_SendChar(*s++);
	}
}
