/// File: usart.c  driver for the serial RS232 interface
// Microcontroller: ATmega88
// Author:	Wilfried Waetzig
// Version:	16.10.2010

#ifndef _UART_H
#define _UART_H

#define BUFF_SIZE	10
//Baudrate of the serial interface
#define BAUDRATE 9600

#include <stdlib.h>
#include <stdarg.h>
#include <ctype.h>
#include <string.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/io.h>

#define SWITCH_3 (PIND&(1<<PD4))

//----------------------------------------------------------------------------
	
void init_USART(void);
void stop_USART(void);
void writec_USART(char cc);
void writestr_USART(char *str, unsigned char cr);
void writetext_USART(char *str);
// receive a line of characters from RX 
//	in buffer: RX_buffer[] when  RX_line_done = 1
char *get_line_USART (void);

//----------------------------------------------------------------------------

#endif //_UART_H
