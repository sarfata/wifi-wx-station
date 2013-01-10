// FILE:  weather_station.c   Version-5
//	Author:	W. Waetzig										05.11.2011
//
//  Compiler: WIN-AVR 20080610
//
// Controlling:	the Barometer Module: HP03S
// 		and the Humidity Sensor Module: HH10D
// The correction-parameters are read out with the TWI-interface (I2c)
//	from the serial EEPROMs on the modules.
// Display of the values with LCD-Module: EA DOGM 162WA.
// The measurements (humidity, temperature and pressure) can be stored
//	in regular intervals in an EEPROM 25AA512. 
//	With 64kB a maximum of 8191 measurements is possible.
// A serial to USB-interface (FT232RL) allows to read out the stored values of the 
//	measurements in text-format with end of line by CR/LF.
// Special care is taken to reduce the power-consumption of the system 
//	by reducing the clock-frequency of the microprocessor from 8 MHz to 1 MHz and by 
//	introducing SLEEP-modes during the waiting-cycles.
//	During long-term data-taking the measurements are made only at every minute.
//
// Correction for Version-2:
//	=> calc_temp_pres(): 2-nd order correction for P and T
// Correction for Version-3:
//	=> lcd-driver.c - lcd_init(): set full contrast for 3,0V-operation
// Correction for Version-4:
//	To terminate the communication e.g if the USB-connection is stuck,
//	press SWITCH_3 during the data-transfer,
//	=> print_meas(): SWITCH_3 terminates the printing of data in USB-mode
//	=> USART_driver.c - get_line_USART():
//		if SWITCH_3 is pressed during the data-transfer,
//		return character 'x' to terminate the communication
// Correction for Version-5:
//	=> calc_temp_pres(): correct offset-calculation
//		according to datasheet HP03S Version 1.3
//
//	Processor:	ATmega88
//	Fuses:	EXT.	0xF9	8Mhz internal oscillator / 8 => clock = 1 MHz
//			HIGH	0xDF	CKDIV8 enabled, brown-out disabled
//			LOW 	0x62	65ms startup
//
// The display is controlled by 3 switches:
// SW-1: step through the functions 0..5
//  0:	normal display:	SW-2: show value pres.	SW-3: show value humidity
//  1:	set clock:		SW-2: increment hours	SW-3: increment minutes
//  2:	set mint/nmeas	SW-2: incr. mint: 0..6	SW-3: clear nmeas
//  3:	UART-control:	SW-2: continue			SW-3: exit
//  4:  display meas.	SW-2: continue			SW-3: exit
//
// flagmint: set interval to store the measurements into the big EEPROM:
// flagmint = 0	no storing
// flagmint = 1	store measurements 1 time per hour  (every 60 mins)
// flagmint = 2	store measurements 2 times per hour (every 30 mins)
// flagmint = 3	store measurements 3 times per hour (every 20 mins)
// flagmint = 4	store measurements 4 times per hour (every 15 mins)
// flagmint = 5	store measurements 5 times per hour (every 12 mins)
// flagmint = 6	store measurements 6 times per hour (every 10 mins)
//
// Commands from the serial interface (USB):
//	h=help		print commands
//	a=show-p	print number of measurements
//	p#=print#	p0 print measurements with names; p1 print only values; p2 print x-scale in hours
//	m#=set-fm#	m# set measurement interval #=0..4
//	c=clear		clear measurement buffer
//	x=exit		terminate the serial communication
//
//  The waiting-cycle is performed in IDLE/POWER_SAVE-mode;
//	In the POWER_SAVE-mode runs only TIMER-2 with the clock-program
//		and the 1-second-interrupt;
//		the LCD displays only the clock-time. 		
//
//	Format to store the measurements in EEPROM:
//	byte 7:	low(pres)
//	byte 6: high(pres)	- pressure in hPa * 10
//	byte 5: low(temp)
//	byte 4: high(temp)	- temperature in degC * 10
//	byte 3: humidity %	- humidity in %
//	byte 2: hour/minute	- measuring time: hour/part
//	byte 1: low(hour)
//	byte 0: high(hour)	- hours of measurement/interval
//
//	The values of a long-term measurement can be plotted with "gnuplot".
//	Format-0:	nnnn hh:mm:00 T=ttt.t* H=hh% P=pppp.p hPa	=> numbers and units
//	Format-1:	nnnn hh:mm:00 tttt hh ppppp		=> numbers only
//	Format-2:	hhhh.hh tttt hh ppppp		=> output for GNUplot
//	In GNUplot:
//	the three diagrams from <file> are plotted with the commands (example):
//		plot <file> using 1:($3/10) title "temperature in degree C"
//		plot <file> using 1:4 title "humidity in percent"
//		plot <file> using 1:($5/10) title "pressure in hPa"
//
//	Port-assignment:
//	================
//PORTB:
// PB0 [14] (output) = LCD-D4
// PB1 [15] (output) = LCD-D5
// PB2 [16] (output) = LCD-D6
// PB3 [17] (output) = LCD-D7  (MOSI)	
// PB4 [18] (output) = LCD-RS  (MISO)
// PB5 [19] (output) = LCD-EN  (SCK)
// PB6  [9] = TOSC1 quartz 32168 Hz
// PB7 [10] = TOSC2 quartz 32168 Hz  => MCLK for Barometer Module; ADC-clock

//PORTC:
// PC0 [23] (input)  = 
// PC1 [24] (input)  = 
// PC2 [25] (input)  = 
// PC3 [26] (output) => XCLR for Barometer Module; reset ADC
// PC4 [27] (input)  = TWI SDA (pull-up)
// PC5 [28] (input)  = TWI SCL (pull-up)
// PC6  [1] (input)  = RESET

//PortD:
// PD0  [2] (input)  = (RXD)
// PD1  [3] (input)  = (TXD)
// PD2  [4] (input)  = SWITCH-1 (pull-up)
// PD3  [5] (input)  = SWITCH-2 (pull-up)
// PD4  [6] (input)  = SWITCH-3 (pull-up)
// PD5 [11] (input)  = T1-TIMER-1  <= FOUT from Humidity Sensor Module
// PD6 [12] (input)  = USB-ON  sensor for USB 3V3
// PD7 [13] (output) = test-signal

#define SWITCH_1 (PIND&(1<<PD2))
#define SWITCH_2 (PIND&(1<<PD3))
#define SWITCH_3 (PIND&(1<<PD4))
#define USB_ON   (PIND&(1<<PD6))

#define set_iobit(x,y) x |= (1<<y)
#define clr_iobit(x,y) x &= ~(1<<y)

#define BIGEEPROM 0b10101110
#define BARO_DEV  0b11101110
#define BAREEPROM 0b10100000
#define HUMEEPROM 0b10100010

#define SLEEPTIME 100

#ifndef  F_CPU
#define  F_CPU 1000000   //8 Mhz ===> 1 MHz
#endif 

#include <stdio.h>
#include <string.h> 
#include <stdint.h> 
#include <stdlib.h> 
#include <inttypes.h>
#include <avr/io.h> 
#include <avr/interrupt.h> 
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <util/delay.h> 

#include "LCD_Driver.h"
#include "TWI_Driver.h"
#include "USART_driver.h"

typedef union _DWORD {
    unsigned long int _dword;
    struct {
        unsigned char byte0;
        unsigned char byte1;
        unsigned char byte2;
        unsigned char byte3;
    };
    struct {
        unsigned int word0;
        unsigned int word1;
    };
} DWORD;

typedef union _WORD {
    unsigned int _word;
    struct {
        unsigned char byte0;
        unsigned char byte1;
    };
} WORD;

volatile unsigned char flag1sec = 0; 	// indicator for 1 second
volatile unsigned char flagmstor = 0;	// indicator for storing into EEPROM
volatile unsigned char flagmint;		// indicator for storing-interval
volatile unsigned char flagpart;		// indicator for part of interval

volatile unsigned char WsecCount = 0; 	// variable for wall-clock seconds counter
volatile unsigned char WminCount = 0; 	// variable for wall-clock minutes counter
volatile unsigned char WhrsCount = 0; 	// variable for wall-clock hours counter

volatile unsigned int  humidfreq;	// variable for frequency of humidity sensor

static long int baropc[8];			// parameters C1..C7 for Barometer Module
static unsigned char baroabcd [4];	// sensor correction-parameters A,B,C,D
static long int barod1,  barod2;	// measured values: press.,temp.
static long int barodut, barooff;	// auxiliary variables
static long int barosens, baroxx;	// auxiliary variables
static      int barotemp, baropres;	// temperature, pressure

static      int humsens, humoffs;	// parameters for Humidity Sensor Module		
static unsigned char humvalu;		// relative humidity 1..99 %

static char textbuffer[20];			// LCD and UART textbuffer
static char *text_buf_pnt;

static unsigned char twi_buffer[12];
static unsigned char savemode, sleepcount, sleepmode;	// control power-saving	

static unsigned int  measnumb, meashour;	// number of measurements in EEPROM
static unsigned int  prnthour;
static unsigned char prntpart, prntmint, prntwhrs;	// part of hour

static   DWORD saveDword;
static   WORD saveword;

// Strings in PGM-space
char MSG_title[]  PROGMEM = "#Weather Logger5";
char MSG_clock[]  PROGMEM = "clock hours/mins";
char MSG_intvl[]  PROGMEM = "M-interval/M-buf";
char MSG_serial[] PROGMEM = "UART-control>USB";
char MSG_commd[]  PROGMEM = "# h=help/a=show-p/p#=print#/m#=set-fm#/c=clear/x=exit";
char MSG_error[]  PROGMEM = "USB not connectd";
char MSG_active[] PROGMEM = "USB-UART active ";
char MSG_clear[]  PROGMEM = "Clear meas.buff.";
char MSG_switch[] PROGMEM = "SW2=cont.SW3=ret";

//################ interruptvector seconds counter - CTC-mode ####################### 
ISR(TIMER2_COMPA_vect)      
{
	register unsigned char fpart;
	flag1sec = 1;		// flag to indicate the 1 second-interrupt
// measure the frequency of humidity sensor at pin T1
//	TCCR1A = 0;
	TCCR1B = (0<<CS12) | (0<<CS11) | (0<<CS10);	// stop counter
	humidfreq = TCNT1;	// get frequency
	TCNT1 = 0;			// clear counter
	TCCR1B = (1<<CS12) | (1<<CS11) | (1<<CS10);	// restart counter rising edge	
// increment the wall-clock
	WsecCount++;				// count wall-clock-seconds
	if (WsecCount == 60) {
		WsecCount=0;
		WminCount++;				// count wall-clock-minutes
		if (WminCount == 60) {
			WminCount=0;
			WhrsCount++;			// count wall-clock-hours
		}
		if (WhrsCount == 24) 
			WhrsCount=0;
		// set flagmstor/flagpart in the intervals to store temp., humid., press.
		if (flagmint != 0)
			for (fpart=0; fpart<flagmint; fpart++) 
				if (WminCount == (60/flagmint)*fpart) {
					flagmstor = 1;
					flagpart = fpart;
				}
	}
}	// end of ISR(TIMER2_COMPA_vect)


//################ initialization seconds counter (asynchron mode) - TIMER-2 #######
void init_TimerSEC(void)
{
	ASSR   = (1<<AS2);			//enable ext. crystal (32.768 Hz) asynchron mode
	TCCR2B = (1<<CS20) | (0<<CS21) | (1<<CS22);	//prescaler: fCLOCK/128=256Hz/256=1Hz>1s
	TIMSK2 = (1<<OCIE2A);     // enable TIMER-2 interrupt, start the seconds counter
}

void init_ports(void)
{
	DDRB = 0x3F;	// set PORTB to output for LCD-display 
	PORTB = 0;
	DDRC  = (1<<PC3);	// XCLR-output
	PORTC = (0<<PC3) | (1<<PC4) | (1<<PC5);	// pull-up for TWI-ports
	DDRD  = (1<<PD7);	// input PD0..PD6; output PD7
	PORTD = (1<<PD2) | (1<<PD3) | (1<<PD4); // pull-up for SWITCH_1..3
	ACSR  = (1<<ACD);	// disable the analog-comparator
}

//############################## Utiliy-routines ###################################

long int read_TWI_eeprom (unsigned char dev, unsigned char adr)
{
	unsigned char status;
	twi_buffer[0] = dev;	// Eprom device-number
	twi_buffer[1] = adr;	// address
	status = Read_Data_TWI (1, twi_buffer, 4);
	saveDword.byte1 = twi_buffer[2];	// MSB
	saveDword.byte0 = twi_buffer[3];	// LSB
	saveDword.word1 = 0;
	return (saveDword._dword);
}

long int read_TWI_ADCvalue (void)
{
	unsigned char status;
	twi_buffer[0] = BARO_DEV;	// barometer device-number
	twi_buffer[1] = 0b11111101;	// address
	twi_buffer[2] = 0b11101111;	// function
	status = Read_Data_TWI (2, twi_buffer, 5);	
	saveDword.byte1 = twi_buffer[3];	// MSB
	saveDword.byte0 = twi_buffer[4];	// LSB
	saveDword.word1 = 0;	
	return (saveDword._dword);
}

void read_temp_pres(void)
{
	unsigned char status;
	set_iobit(PORTC,PC3);	// ADC-XCLR
	// read ADC with pressure value
	twi_buffer[0] = BARO_DEV;	// barometer device-number
	twi_buffer[1] = 0b11111111;	// address
	twi_buffer[2] = 0b11110000;	// function=select+start-conversion PRES
	status = Send_Data_TWI (twi_buffer, 3);
	_delay_ms(40);	// delay for ADC-conversion
	barod1 = read_TWI_ADCvalue();
	// read ADC with temperature value
	twi_buffer[0] = BARO_DEV;	// barometer device-number
	twi_buffer[1] = 0b11111111;	// address
	twi_buffer[2] = 0b11101000;	// function=select+start-conversion TEMP
	status = Send_Data_TWI (twi_buffer, 3);
	_delay_ms(40);	// delay for ADC-conversion
	barod2 = read_TWI_ADCvalue();
	clr_iobit(PORTC,PC3);	// ADC-XCLR					
}

void calc_temp_pres(void)
{
	signed long int barod2c5;
	unsigned char barocorr;
	// Temperature and Pressure calculations from HP03S version 1.3
	// with 2nd-order corrections using the parameters A,B,C,D
	// measured values are: barod1(pressure) and barod2(temperature)
	//	correct offset-calculation (barooff)
	//	according to datasheet HP03S Version 1.3	
	barod2c5 = barod2 - baropc[5];
	if (barod2c5 >= 0) 
		barocorr = baroabcd[0];		// A
	else
		barocorr = baroabcd[1];		// B
	barodut = barod2c5 - (((barod2c5>>7)*(barod2c5>>7)*(long)barocorr)>>baroabcd[2]); //C
//	barooff = ( (baropc[4]-2048) * barodut / 16384 + baropc[2] ) * 4;
//	barooff = ((( (baropc[4]-2048) * barodut ) >> 14 ) + baropc[2] ) << 2;
	barooff = ((( (baropc[4]-1024) * barodut ) >> 14 ) + baropc[2] ) << 2;
//	barosens = baropc[3] * barodut / 1024 + baropc[1] ;
	barosens = (( baropc[3] * barodut ) >> 10 ) + baropc[1] ;	
//	baroxx = (barod1-7168) * barosens / 16384 - barooff;
	baroxx = (((barod1-7168) * barosens ) >> 14 ) - barooff;	
//	baropres = baroxx * 10 / 32 + baropc[7];
	baropres = (( baroxx * 10 ) >> 5 ) + baropc[7];
//	barotemp = barodut * baropc[6] / 65536 + 250;
	barotemp = (( barodut * baropc[6] ) >> 16 ) - (barodut>>baroabcd[3]) + 250;	//D
}

// load flagmint, measnumb and meashour from EEPROM[3:7]
void load_params (void)
{
	unsigned char status;
	twi_buffer[0] = BIGEEPROM;		// EEPROM-device
	twi_buffer[1] = 0;
	twi_buffer[2] = 3;	// EEPROM-address[3] 
	status = Read_Data_TWI (2, twi_buffer, 8);
	flagmint = twi_buffer[3];	
	saveword.byte1 = twi_buffer[4];
	saveword.byte0 = twi_buffer[5];
	measnumb = saveword._word;
	saveword.byte1 = twi_buffer[6];
	saveword.byte0 = twi_buffer[7];
	meashour = saveword._word;
}

// store flagmint, measnumb and meashour into EEPROM[3:7]
void store_params (void)
{
	unsigned char status;
	twi_buffer[0] = BIGEEPROM;		// EEPROM-device
	twi_buffer[1] = 0;
	twi_buffer[2] = 3;	// address [3] in EEPROM
	twi_buffer[3] = flagmint;	
	saveword._word = measnumb;
	twi_buffer[4] = saveword.byte1;
	twi_buffer[5] = saveword.byte0;	// number of measurement
	saveword._word = meashour;
	twi_buffer[6] = saveword.byte1;
	twi_buffer[7] = saveword.byte0;	// number of measured hours	
	status = Send_Data_TWI (twi_buffer, 8);	
	Poll_NAK_TWI(BIGEEPROM);	// wait for completion
}

// read measurements from EEPROM
//  1.  2 bytes	- prnthour*8+prntmint - measuring interval in hours
//  2.  1 byte  - prntwhrs*8+prntpart - measuring time: hour/part
//  3.  1 byte  - humvalu  - humidity in %
//  4.  2 bytes	- barotemp - temperature in degC * 10
//  5.  2 bytes	- baropres - pressure in hPa * 10
void load_meas (int numb)
{
	unsigned char status;
	twi_buffer[0] = BIGEEPROM;		// EEPROM-device
	saveword._word = numb<<3;
	twi_buffer[1] = saveword.byte1;
	twi_buffer[2] = saveword.byte0;	// address in EEPROM = measindx*8
	status = Read_Data_TWI (2, twi_buffer, 11);
	saveword.byte1 = twi_buffer[3];
	saveword.byte0 = twi_buffer[4];
	prnthour = saveword._word>>3;			// prnthour = meashour for print
	prntmint = saveword.byte0 & 7;
	prntwhrs = twi_buffer[5]>>3;
	prntpart = twi_buffer[5] & 7;
	humvalu  = twi_buffer[6];
	saveword.byte1 = twi_buffer[7];
	saveword.byte0 = twi_buffer[8];
	barotemp = saveword._word;	
	saveword.byte1 = twi_buffer[9];
	saveword.byte0 = twi_buffer[10];
	baropres = saveword._word;
}

// store measurements in EEPROM
//  1.  2 bytes	- meashour*8+flagmint - measuring interval in hours
//  2.  1 byte  - WhrsCount*8+flagpart  - measuring time: hour/part
//  3.  1 byte  - humvalu  - humidity in %
//  4.  2 bytes	- barotemp - temperature in degC * 10
//  5.  2 bytes	- baropres - pressure in hPa * 10
void store_meas (void)
{
	unsigned char status;
	twi_buffer[0] = BIGEEPROM;		// EEPROM-device
	saveword._word = measnumb<<3;
	twi_buffer[1] = saveword.byte1;
	twi_buffer[2] = saveword.byte0;	// address in EEPROM = measnumb*8
	if (WminCount == 0)
		meashour++;
	saveword._word = (meashour<<3) + flagmint;	
	twi_buffer[3] = saveword.byte1;
	twi_buffer[4] = saveword.byte0;	// meashour = measuring interval in hours
	twi_buffer[5] = (WhrsCount<<3) + flagpart;	// timing
	twi_buffer[6] = humvalu;			// humidity in %
	saveword._word = barotemp;
	twi_buffer[7] = saveword.byte1;
	twi_buffer[8] = saveword.byte0;		// temperature in degC * 10
	saveword._word = baropres;
	twi_buffer[9]  = saveword.byte1;
	twi_buffer[10] = saveword.byte0;	// pressure in hPa * 10
	status = Send_Data_TWI (twi_buffer, 11);
	Poll_NAK_TWI(BIGEEPROM);	// wait for completion
	store_params ();	// store flagmint, measnumb and meashour into EEPROM[3:7]
	sprintf(textbuffer,"%4i %03i",measnumb,WhrsCount);
	lcd_line(1);
	lcd_wstring(textbuffer);	
}

// display LCD line-2 and wait
void disp_l2wait(void)
{
	lcd_line(2);
	lcd_wstring(textbuffer);
	_delay_ms(500);
}

//  print stored measurements; mode=0: with names; mode=1: numbers only; mode=2: x-scale in hours
//	Format-0:	nnnn hh:mm:00 T=ttt.t degC H=hh% P=pppp.p hPa
//	Format-1:	nnnn hh:mm:00 tttt hh ppppp
//	Format-2:	hhhh.hh tttt hh ppppp
//  prnthour = measuring interval in hours
//    exit if SWITCH_3 is pressed
void print_meas (unsigned char mode)
{
	unsigned int  measindx;
	load_params(); // load flagmint, measnumb and meashour from EEPROM[3:7]
	sprintf (textbuffer,"%4i measurements  ",measnumb);
	if (mode > 0)
		writec_USART('#');	// comment for "GNUplot"
	writestr_USART (textbuffer,0);	
	sprintf (textbuffer, "%02i:%02i:%02i", WhrsCount, WminCount, WsecCount);
	writestr_USART (textbuffer,1);
	// print measured values from EEPROM
	for (measindx=1; measindx<=measnumb; measindx++) {
		load_meas (measindx);	// load: prnthour,timing,humvalu,barotemp,baropres
		if (mode != 2)
			sprintf (textbuffer,"%4i %02i:%02i:00",measindx,prntwhrs,(60/prntmint)*prntpart);
		else 
			sprintf(textbuffer,"%4i.%02i",prnthour,(int)prntpart*100/prntmint);		
		writestr_USART(textbuffer,0);		
		if (mode == 0) {
			sprintf (textbuffer, " T=%3i.%i degC", barotemp/10,abs(barotemp%10));
			writestr_USART(textbuffer,0);
			sprintf (textbuffer, " H=%2i%% P=%4i.%i hPa",humvalu,baropres/10,baropres%10);
		}
		else 
			sprintf (textbuffer," %4i %2i %5i", barotemp,humvalu,baropres);
		writestr_USART(textbuffer,1);
		if (SWITCH_3 == 0)
			break;
	}
}

// display temperature/humidity/pressure values
void display_thp (void)
{
	lcd_line(1);
	lcd_wstring(textbuffer);
	if(sleepmode == SLEEP_MODE_IDLE) {
		// display the temperature
		sprintf (textbuffer, "T%3i.%i\xF2", barotemp/10,abs(barotemp%10));
		lcd_wstring(textbuffer);
		// display the humidity and the pressure
		sprintf (textbuffer, "H=%2i%% P=%3i.%ihPa ",humvalu,baropres/10,baropres%10);
	}
	else 
		sprintf (textbuffer, "%4i%4i", measnumb, meashour);
	lcd_line(2);
	lcd_wstring(textbuffer);
}

//  display stored measurements
//    exit if SWITCH_3 is pressed
void display_meas (void)
{
	unsigned int measindx;
	load_params(); // load flagmint, measnumb and meashour from EEPROM[3:7]
	sprintf(textbuffer,"SHOW %4i meas. ",measnumb);
	lcd_line(1);
	lcd_wstring(textbuffer);
	lcd_line(2);
	lcd_wtext(MSG_switch);		
	_delay_ms(1000);
	// display measured values
	while ((SWITCH_2 != 0) && (SWITCH_3 != 0))
		_delay_ms(100);
	if (SWITCH_2 == 0)
		for (measindx=1; measindx<=measnumb; measindx++) {
			load_meas(measindx);	// load: prnthour,timing,humvalu,barotemp,baropres
			sprintf(textbuffer,"%3i %02i.%i ",measindx,prntwhrs,prntpart*10/prntmint);
			display_thp();		// display: humvalu,barotemp,baropres
			_delay_ms(2000);
			if (SWITCH_3 == 0)
				break;
		}	
}

// display values of measurement
void disp_values (void)
{
	char control;
	// display the clock-time + special character if data-recording
	control = ':';
	if ((WsecCount&1) && (flagmint != 0)) 
		control = ' ';	
	lcd_clrscr();
	sprintf (textbuffer,"%02i%c%02i%c%02i ",WhrsCount,control,WminCount,control,WsecCount);
	display_thp();		// display: humvalu,barotemp,baropres
	// wake up from sleep-mode if any switch is pressed
	if((savemode==1) && ((SWITCH_1 == 0)||(SWITCH_2 == 0)||(SWITCH_3 == 0))) {
		savemode = 0;
		sleepmode = SLEEP_MODE_IDLE;
		sleepcount = SLEEPTIME;
	}
	//=== display ADC-values from pressure-sensor
	if(sleepmode == SLEEP_MODE_IDLE) {
		if (SWITCH_2 == 0)
			sprintf(textbuffer,"P %04X %04X %04X",(int)barod1,(int)barod2,baropres);			
		if (SWITCH_3 == 0)
			sprintf(textbuffer,"H %04X %04X %04X", humidfreq,humoffs,humsens);
		lcd_line(2);
		lcd_wstring(textbuffer);
	}
}

// number of measurements; mode=0 display; mode=1 UART-output
void print_nummeas(unsigned char mode)
{
	load_params(); // load flagmint + measnumb + meashour from EEPROM[3..7]
	sprintf(textbuffer, "%i/hour%4i meas.",flagmint,measnumb);
	if (mode == 0)
		disp_l2wait();		
	else {
		writec_USART('#');	// comment line in "gnuplot"
		writec_USART(' ');
		writestr_USART(textbuffer,1);
	}
}

void send_meas(void)
{
	// textbuffer is only 20 bytes long - we need a bigger buffer
	char buf[50];
	init_USART();
	
	// Wake up the wifly module with a byte and give it some time to freshen up
	writestr_USART("&", 0);
	_delay_ms(100);
	
	// Prepare a string with all parameters.
	sprintf (buf, "temp=%i&humidity=%i&pressure=%i", barotemp, humvalu, baropres);
	writestr_USART(buf, 0);
	
	stop_USART();
}

//############################## MAIN ##############################################

int main(void)
{	
	char ccser;
	unsigned char sw1on, dispmod1, pmode;
	unsigned char status;
	
	init_ports();
	init_TimerSEC();
	lcd_init();
	Init_TWI();
	sei();
	
	savemode = 0; // standard savemode = IDLE
	sleepmode = SLEEP_MODE_IDLE;	// the I/O-clock must run for TIMER-1
	sleepcount = SLEEPTIME;		// counts the seconds before going to POWER_SAVE
	
	// load parameters for Humidity-Sensor	
	//    humsens = read_TWI_eeprom (HUMEEPROM, 10);
	//    humoffs = read_TWI_eeprom (HUMEEPROM, 12);	
	// load parameters for Pressure-Sensor: C1..C7
	clr_iobit(PORTC,PC3);	// ADC-XCLR		
	for (pmode=1; pmode<8; pmode++) 
		baropc[pmode] = read_TWI_eeprom (BAREEPROM, 14+pmode+pmode);
	// load extra parameters for Pressure-Sensor: A,B,C,D
	twi_buffer[0] = BAREEPROM;	// EEPROM device-number
	twi_buffer[1] = 30;	// address
	status = Read_Data_TWI (1, twi_buffer, 6);
	for (pmode=0; pmode<4; pmode++) 
		baroabcd[pmode] = twi_buffer[pmode+2];	
// print title
	lcd_line(1);
	lcd_wtext(MSG_title);
	print_nummeas(0);	// load and print: measnumb and flagmint
	_delay_ms(2000);

	sw1on = 0;
	dispmod1 = 0;
	
	// ====================== Main Loop =====================================
	// dispmode
	//  0:	normal display:	SW-2: show value pres.	SW-3: show value humidity 
	//  1:	set clock:		SW-2: increment hours	SW-3: increment minutes
	//  2:	set mint/nmeas	SW-2: incr. mint: 0..4	SW-3: clear nmeas
	//  3:	UART-control:	SW-2: continue			SW-3: exit
	//  4:  display meas.	SW-2: continue			SW-3: exit
	//
	for (;;) {
		while (!flag1sec) {
			// ====== set the CPU into POWER-SAVE-MODE ======
			set_iobit(PORTD,PD7);	// test-pulse
			cli();
//			set_sleep_mode(SLEEP_MODE_PWR_SAVE);
//			set_sleep_mode(SLEEP_MODE_IDLE);	// the I/O-clock must run for TIMER-1
			set_sleep_mode(sleepmode);	// SLEEP_MODE_IDLE / SLEEP_MODE_PWR_SAVE
			sleep_enable();
			sei();
			sleep_cpu();	// ===> wait here until an interrupt (TIMER-2) comes
			sleep_disable();
			clr_iobit(PORTD,PD7);	// test-pulse			
		}
		if ((sw1on != 0) && (SWITCH_1 != 0)) {
			sw1on = 0;
			dispmod1++;
		}
		else if (SWITCH_1 == 0) { 
			sw1on = 1;
		}
		// ===================== display ======================
		switch (dispmod1) {
		case 0:
		// ------ standard display -------
			flag1sec = 0;
			if (savemode == 1) {
				if((WsecCount == 0) || (WsecCount == 59))
					sleepmode = SLEEP_MODE_IDLE;
				else
					sleepmode = SLEEP_MODE_PWR_SAVE;
			}
			if(sleepmode == SLEEP_MODE_IDLE) {
				// convert the temperature, the pressure and the humidity
				read_temp_pres();
				calc_temp_pres();	
				// load parameters for Humidity-Sensor **************************
				humsens = read_TWI_eeprom (HUMEEPROM, 10);
				humoffs = read_TWI_eeprom (HUMEEPROM, 12);	
				humvalu = (((long)humoffs-(long)humidfreq)*(long)humsens)>>12;	
				if(savemode == 0)
					sleepcount--;		// count down to go to SLEEP
			}
			// Upload the measures every minutes
			if (WsecCount == 0) {
				send_meas();
			}
			disp_values();
			// store measurement-Values into EEPROM
			if ((flagmint != 0) && (flagmstor != 0)) {
				flagmstor = 0;
				measnumb ++;				
				store_meas(); // store: meashour,timing,humvalu,barotemp,baropres
			}
			// check sleep-mode
			if(sleepcount == 0)
				savemode =1;
			break;
		case 1:
		//  set clock:		SW-2: increment hours; SW-3: increment minutes
			lcd_line(1);
			lcd_wtext(MSG_clock);
			if (SWITCH_2 == 0) 
				WhrsCount++;
			if (WhrsCount > 23)
				WhrsCount = 0;
			if (SWITCH_3 == 0) 
				WminCount++;
			if (WminCount > 59)
				WminCount = 0;
			sprintf(textbuffer, "hours=%02i mins=%02i",WhrsCount,WminCount);
			disp_l2wait();
			break;
		case 2:
		//  set mint/nmeas	SW-2: incr. mint: 0..6	SW-3: clear nmeas
			if (SWITCH_2 == 0) {
				flagmint++;
				if (flagmint > 6)
					flagmint = 0;
				store_params(); // store flagmint, measnumb and meashour into EEPROM[3:7]
			}
			if (SWITCH_3 == 0) {
				measnumb = 0;
				meashour = 0;
				store_params(); // store flagmint, measnumb and meashour into EEPROM[3:7]
			}
			lcd_line(1);
			lcd_wtext(MSG_intvl);
			print_nummeas(0);
			break;
		case 3:
		//	UART-control:	SW-2: start output;		SW-3: stop output and exit
			lcd_clrscr();
			lcd_line(1);
			lcd_wtext(MSG_serial);
			if (USB_ON != 0) {	// USB is connected
				lcd_line(2);
				lcd_wtext(MSG_switch);		
				while ((SWITCH_2 != 0) && (SWITCH_3 != 0))	
					_delay_ms(100);		// wait for SW2 or SW3
				if (SWITCH_2 == 0) {	// init USART; USB-power is on
					lcd_line(2);
					lcd_wtext(MSG_active);			
					init_USART();
					writetext_USART(MSG_title);
					// ========== read command-line from USART ==============
					while ( SWITCH_3 != 0 ) {
						writec_USART('#');	// comment line in "gnuplot"
						writec_USART('>');
						text_buf_pnt = get_line_USART ();	// <== wait for line-input
						ccser = *text_buf_pnt++;
						if (ccser == 'h') {
							// "# h=help/a=show-p/p#=print#/m#=set-fm#/c=clear/x=exit"			
							writetext_USART (MSG_commd);			
						}
						else
						if (ccser == 'a') {		// print flagmint and measnumb
							print_nummeas(1);
						}
						else						
						if (ccser == 'p') {		// print measured values
							ccser = *text_buf_pnt++;
							pmode = 0;
							if ((ccser >= '0') && (ccser < '3'))
								pmode = ccser - '0';						
							print_meas (pmode);
						}
						else
						if (ccser == 'm') {		// set flagmint
							ccser = *text_buf_pnt++;
							if ((ccser >= '0') && (ccser < '7')) {
								flagmint = ccser - '0';
								store_params(); // store flagmint, measnumb and meashour into EEPROM[3:7]
							}
							print_nummeas(1);	
						}
						else						
						if (ccser == 'c') {		// clear measnumb
							writetext_USART (MSG_clear);
							measnumb = 0;
							meashour = 0;
							store_params(); // store flagmint, measnumb and meashour into EEPROM[3:7]
						}
						else
						if (ccser == 'x')		// EXIT with command
							break;
					}	// end while
				}
			}
			stop_USART();
			lcd_line(2);
			lcd_wtext (MSG_error);	// 	"USB not connectd"	
			dispmod1++;
			_delay_ms(2000);			
			break;
		case 4:
		// display measurements	SW-2: start display; SW-3: stop display	
			display_meas();
			dispmod1++;
			break;
		default:
			dispmod1 = 0;
			break;
		}
	}
}


