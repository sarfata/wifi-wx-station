// File: usart.c  driver for the serial RS232 interface
// Microcontroller: ATmega88
// Author:	Wilfried Waetzig
// Version:	16.11.2010
// Check, if SWITCH_3 is pressed during the data-transfer,
//  if yes, return character 'x' to terminate the communication

#include "USART_driver.h"

volatile unsigned char buffcnt = 0;
volatile unsigned char enable_USART;
volatile unsigned char RX_line_done;
static char RX_buffer[BUFF_SIZE];

//-------------------------------------------------------------
//Init. serial RS232 interface
void init_USART(void) 
{ 
	// UDR0: RX/TX data-register
	// UCSR0A: control and status; double TX-rate: F_CPU=1 MHz!!!
	UCSR0A = (1<<U2X0);
  	// UCSR0B: Enable TXEN0+RXEN0 + interrupt(RXCIE0)
	UCSR0B = (1<<TXEN0) | (1<<RXEN0) | (1<<RXCIE0);
	// UCSR0C: mode=async.: UMSEL01/00 = 00
	//	parity=disabled:  UPM01/00 = 00
	//	1 stop-bit; 8 data-bits
	UCSR0C = (0<<USBS0)|(0<<UCSZ02)|(1<<UCSZ01)|(1<<UCSZ00);
	//set baudrate in (UBRR0L,UBRR0H);  correct for double TX-rate
//	UBRR0 = (F_CPU / (BAUDRATE * 16L) - 1);
	UBRR0 = (F_CPU / (BAUDRATE * 8L) - 1);
	RX_line_done = 0;
	buffcnt = 0;
	enable_USART = 1;
}
//-------------------------------------------------------------
//Stop serial RS232 interface
void stop_USART(void) 
{ 
  	// UCSR0B: Enable TXEN0+RXEN0 + interrupt(RXCIE0)
	UCSR0B = 0;
	enable_USART = 0;
	RX_line_done = 0;
	buffcnt = 0;
}
//---------------------------------------------------------------
//Output 1 character
void writec_USART(char cc)
{
	if(enable_USART) {
		// Wait for last character
		while(!(UCSR0A & (1<<UDRE0))) ;
		// Output 1 character
		UDR0 = cc;
	}
	return;
}
//----------------------------------------------------------------
//Output a string
void writestr_USART(char *str,unsigned char cr)
{
	register char cc;
	while ((cc=*str++)!=0) {
		writec_USART(cc);
	}
	if (cr != 0) {
		writec_USART('\r');	
		writec_USART('\n');	
	}		
}
//----------------------------------------------------------------
//Output a string from memory
void writetext_USART(char *str)
{
	register char cc;
	while ((cc=pgm_read_byte(&*str++))!=0) {
		writec_USART(cc);
	}
	writec_USART('\r');
	writec_USART('\n');	
}
//----------------------------------------------------------------
// receive a line of characters from RX (UDR0)
//	in buffer: RX_buffer[] when  RX_line_done = 1
//Receive a character, end-of-line by ch < ' '
ISR (USART_RX_vect)
{
	if((enable_USART != 0) && (RX_line_done == 0))
	{
		register char receive_char;
		receive_char = (UDR0);
		if (receive_char < ' ') 
			receive_char = '\0';
		RX_buffer [buffcnt++] = receive_char;
		if (buffcnt > BUFF_SIZE-2)
			buffcnt--;
		if (receive_char == '\0') {
			RX_line_done = 1;
			buffcnt = 0;
		}
	}
	return;
}
//----------------------------------------------------------------
// get input-line from UART
//	return pointer to receiver-buffer
// Check, if SWITCH_3 is pressed during the data-transfer,
//  if yes, return character 'x' to terminate the communication
char *get_line_USART (void)
{
	buffcnt = 0;
	RX_line_done = 0;
	while ((RX_line_done == 0) ) {
		if (SWITCH_3 == 0) {
			RX_buffer[0] = 'x';
			RX_buffer[1] = '\0';
			RX_line_done = 1;
		}
	}
	return (&RX_buffer[0]);
}
//----------------------------------------------------------------
