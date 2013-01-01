/****************************************************************************
 Title	:   HD44780U LCD driver library for EA DOGM162 display-modules 2*16 char.
 Author:    Wilfried Waetzig
 File:	    $Id: lcd-driver.c,v 1.0 12.11.2009/28.09.2010
 Software:  AVR-GCC 3.3
 Target:    any AVR device

 DESCRIPTION
       Basic routines for interfacing a HD44780U-based text LCD display;
       only 4-bit datamode, R/W-line tied to ground,
       displays strings in RAM and text from ROM. 
       See the C include lcd.h file for a description of each function

*****************************************************************************/
#include <inttypes.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>			// _delay_us(x) and _delay_ms(x)
#include "lcd_driver.h"

/*
** constants/macros
*/
#define set_iobit(x,y) x |= (1<<y)
#define clr_iobit(x,y) x &= ~(1<<y)
/*
** define port and position of DATA-bits, RS-bit and EN-bit
** configuration for project ====== Weather-Station ======
**	PB0: (output)	  LCD-D4	==> set values in lcd.h
**	PB1: (output)	  LCD-D5
**	PB2: (output)	  LCD-D6
**	PB3: (output)	  LCD-D7 | MOSI
**	PB4: (output)	  LCD-EN | MISO
**	PB5: (output)	  LCD-RS | SCL
*/
#define LCD_DAPO PORTB
#define LCD_DADR DDRB
#define LCD_DSHFT 0
#define LCD_MASK (0x0F<<LCD_DSHFT)
#define LCD_RSPO PORTB
#define LCD_RSDR DDRB
#define LCD_RSBIT 5
#define LCD_ENPO PORTB
#define LCD_ENDR DDRB
#define LCD_ENBIT 4
/*
** local functions
*/
/*************************************************************************
Low-level function to write nibble to LCD controller
Input:    data   byte to write to LCD
          rs     1: write data
                 0: write instruction
Returns:  none
*************************************************************************/
static void lcd_w4bit(uint8_t nibble,uint8_t rs)
{
	if (rs)    /* write data        (RS=1, RW=0) */
       set_iobit (LCD_RSPO,LCD_RSBIT);
	else    /* write instruction (RS=0, RW=0) */
       clr_iobit (LCD_RSPO,LCD_RSBIT);
/* output data */
	LCD_DAPO = (LCD_DAPO & ~LCD_MASK) | ((nibble<<LCD_DSHFT) & LCD_MASK);
/* strobe EN-bit */
	set_iobit (LCD_ENPO,LCD_ENBIT);
	_delay_us (100);
	clr_iobit (LCD_ENPO,LCD_ENBIT);
	_delay_us (100);
}
/*************************************************************************
Low-level function to write byte to LCD controller
Input:    data   byte to write to LCD
          rs     1: write data
                 0: write instruction
Returns:  none
*************************************************************************/
static void lcd_wbyte(uint8_t data,uint8_t rs)
{
/* write upper nibble */
	lcd_w4bit((data>>4), rs);
/* write lower nibble */
	lcd_w4bit( data , rs);
}
/*
** PUBLIC FUNCTIONS
*/
/*************************************************************************
Send LCD controller instruction command
Input:   instruction to send to LCD controller, see HD44780 data sheet
Returns: none
*************************************************************************/
void lcd_command(uint8_t comand)
{
	lcd_wbyte (comand,0);
	_delay_ms (2);
}
/*************************************************************************
Send data byte to LCD controller
Input:   data to send to LCD controller, see HD44780 data sheet
Returns: none
*************************************************************************/
void lcd_wchar(uint8_t lchar)
{
	lcd_wbyte (lchar,1);
}
/*************************************************************************
Set cursor to line or line 2
Input:    x  line-number
Returns:  none
*************************************************************************/
void lcd_line(uint8_t x)
{
	if (x==1)
		lcd_command (0b00000010); /* line 1 */
	else
		lcd_command (0b11000000); /* line 2 */
}
/*************************************************************************
Display string in RAM terminated by zero
Input:    pointer to string to be displayed
Returns:  none
*************************************************************************/
void lcd_wstring(const char *s)
{
    register char c;
    while ( (c = *s++) ) {
        lcd_wchar(c);
    }
}
/*************************************************************************
Display text-string from program memory terminated by zero
Input:     pointer to string from program memory
Returns:   none
*************************************************************************/
void lcd_wtext(const char *progmem_s)
{
    register char c;
    while ( (c = pgm_read_byte(progmem_s++)) ) {
        lcd_wchar(c);
    }
}
/*************************************************************************
Initialize PORT-states and the display for EA DOGM162 display-modules
with 2*16 char. and 3.3 Volt
Input:    none
Returns:  none
*************************************************************************/
void lcd_init(void)
{
	/* initalize the PORT-states */
	LCD_DADR |= LCD_MASK;	/* data nibble as output */
	set_iobit (LCD_RSDR, LCD_RSBIT);	/* RS-bit */
	clr_iobit (LCD_RSPO, LCD_RSBIT);	
	set_iobit (LCD_ENDR, LCD_ENBIT);	
	clr_iobit (LCD_ENPO, LCD_ENBIT);	/* EN-bit */		
	/*  Initialize LCD to 4 bit I/O mode  */
	_delay_ms (250);
	lcd_w4bit (3,0);
	lcd_w4bit (3,0);
	lcd_w4bit (3,0);
	lcd_w4bit (2,0);
	lcd_command (0b00101001);		// DOGM-function-set: DL=4;N=2;DH=0;IS=01
	lcd_command (0b00010100);		// TAB1: Bias Set: BS=1/5;ZL=2
	lcd_command (0b01010101);		// TAB1: Power Control: booster=on;Contrast=c5
	lcd_command (0b01101101);		// TAB1: Follower Control: follow+ampl.
//	lcd_command (0b01111000);		// TAB1: Contrast set: contrast=C3,C2,C1 for 3V3
	lcd_command (0b01111111);		// TAB1: Contrast set: contrast=C3,C2,C1 for 3V0
	lcd_command (0b00101000);		// DOGM-function-set: DL=4;N=2;DH=0;IS=00
	lcd_command (0b00001000);	/* display off, cursor off, blink off */	
	lcd_command (0b00001100);	/* display on,cursor off, blink off */
	lcd_command (0b00000110);	/* cursor position moves to the right */
	lcd_command (0b00000010);	/* DDRam-adress=0, (most left position) */
	lcd_command (0b00000001);	/* clear the Display */
}
