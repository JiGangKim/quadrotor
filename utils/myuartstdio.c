/*
 * myuartstdio.c - UART utility providing printf functionality of C
 *
 *  Created on: 2015. 9. 5.
 *      Author: jgkim2020
 *
 * I do not take any responsibility for any consequences resulting from the
 * use of this software. USE AT YOUR OWN RISK!
 *
 */

//*****************************************************************************
#include <stdbool.h>
#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_uart.h"
#include "driverlib/debug.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
//*****************************************************************************

void strrev(char *str)
{
    char *p = str, *q, c;
	q = p + strlen(p) - 1;
	while(p < q)
	{
		c = *p; *p = *q; *q = c; // swap *p and *q
		++p; --q; // forward and backward
	}
}

void itoa(int n, char *p)
{
    int sign = n;
    char *str = p;
    if(n < 0) n = -n;  // make positive
    do
    {
    	*p = n % 10 + '0';
    	p++; // get next digit
    }
    while((n/=10) > 0); // delete digit
    if(sign < 0)
    {
    	*p = '-';
    	p++;
    }
    *p = '\0';
    strrev(str);
}

void myUARTprintf(uint32_t ui32Base, char *usData, ...)
{
	char *p, *q, *sval, buffer[100];
	int i, ival;
	double dval;
	va_list ap;
	va_start(ap, usData);
	for(p = usData; *p; p++)
	{
		if(*p != '%')
		{
			UARTCharPut(ui32Base, *p);
			continue;
		}
		switch(*++p)
		{
		case 'd': // integer
			ival = va_arg(ap, int);
			itoa(ival, buffer);
			for(q = buffer; *q; q++) UARTCharPut(ui32Base, *q);
			break;
		case 'f': // float (3digits)
			dval = va_arg(ap, double);
			if((dval < 0.0) && (dval > -1.0)) myUARTprintf(ui32Base, "-%d.", (int)dval);
			else myUARTprintf(ui32Base, "%d.", (int)dval);
			for(i = 10; i < 10000; i *= 10) myUARTprintf(ui32Base, "%d", (abs((int)(dval*i)) - abs(((int)dval)*i))%10);
			break;
		case 'l': // double (6digits)
			dval = va_arg(ap, double);
			if((dval < 0.0) && (dval > -1.0)) myUARTprintf(ui32Base, "-%d.", (int)dval);
			else myUARTprintf(ui32Base, "%d.", (int)dval);
			for(i = 10; i < 10000000; i *= 10) myUARTprintf(ui32Base, "%d", (abs((int)(dval*i)) - abs(((int)dval)*i))%10);
			break;
		case 's': // string
			for(sval = va_arg(ap,char *); *sval; sval++) UARTCharPut(ui32Base, *sval);
			break;
		case 'n': // line break
			UARTCharPut(ui32Base, '\n');
			break;
		default:
			UARTCharPut(ui32Base, *p);
			break;
		}
	}
	va_end(ap);
}
