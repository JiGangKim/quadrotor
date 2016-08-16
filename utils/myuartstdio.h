/*
 * myuartstdio.h - UART utility providing printf functionality of C
 *
 *  Created on: 2015. 9. 5.
 *      Author: jgkim2020
 *
 * I do not take any responsibility for any consequences resulting from the
 * use of this software. USE AT YOUR OWN RISK!
 *
 */

void strrev(char *str);
void itoa(int n, char *p);
void myUARTprintf(uint32_t ui32Base, char *usData, ...);
