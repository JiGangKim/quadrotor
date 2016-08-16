/*
 * myparser.c - parser
 *
 *  Created on: 2016. 7. 18.
 *      Author: jgkim2020
 *
 * I do not take any responsibility for any consequences resulting from the
 * use of this software. USE AT YOUR OWN RISK!
 *
 */

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "inc/hw_memmap.h"
#include "myuartstdio.h"
#include "myvariable.h"

int CommandHandler(char* str, int mode)
{
	if(XBEEIntflag == 1)
	{
		int len = strlen(str);
		XBEEIntflag = 0;
		if(XBEEIntmsg_pointer == len)
		{
			XBEEIntmsg_pointer = 0;
			int cmp = 0;
			for(i = 0; i < len; i++)
			{
				if(str[i] != XBEEIntmsg[i]) cmp = 1;
			}
			myUARTprintf(UART1_BASE, "%n");
			if(cmp == 0) return 2*mode;
			else return 1 + 3*mode;
		}
		else return 1;
	}
	else return 1;
}

int CUIHandler(char* cmd, char* msg, int mode)
{
	int status = 1;
	myUARTprintf(UART1_BASE, msg);
	while(status%2) status = CommandHandler(cmd, mode);
	if(status/2 < 2) return 1;
	else return 0;
}
