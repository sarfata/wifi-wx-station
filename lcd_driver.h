#ifndef LCD_DRIVER_H
#define LCD_DRIVER_H
/*************************************************************************
 Title	:   HD44780U LCD driver library
 Author:    Wilfried Waetzig
 File:	    $Id: lcd-driver.h,v 1.0 06.11.2009
 Software:  AVR-GCC 3.3
 Target:    any AVR device

 DESCRIPTION
       Basic routines for interfacing a HD44780U-based text LCD display;
       only 4-bit datamode, R/W-line tied to ground,
       displays strings in RAM and text from ROM. 
**************************************************************************/
// Send LCD controller instruction command
void lcd_command(uint8_t comand);

// Send data byte to LCD controller
void lcd_wchar(uint8_t lchar);

// Set cursor to line or line 2
void lcd_line(uint8_t x);

// Clear display and set cursor to home position
#define lcd_clrscr() lcd_command(0b00000001)

// Display string in RAM terminated by zero
void lcd_wstring(const char *s);

// Display text-string from program memory terminated by zero
void lcd_wtext(const char *progmem_s);

// Initialize the PORT-states and the display
void lcd_init(void);
// 
#endif //LCD_DRIVER_H