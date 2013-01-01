/********************** TWI-Driver for Weatherstation   W.Waetzig   3.10.2010
using parts of AVR155 : Accessing I2C LCD display using the AVR TWI ********
Device      : ATmega88
File name   : TWI_driver.h
Ver nr.     : 1.0
Description : Header file for TWI_driver.c
Author      : Asmund Saetre
Change log  : Created 01.05.2000  AS
Modified    :   12.10.2010  W.W.  
****************************************************************************/

/****************************************************************************
	Include Files for WIN-AVR 
****************************************************************************/
#include <avr/io.h> 
#include <stdio.h>
#include <avr/interrupt.h> 

/****************************************************************************
	Function definitions
****************************************************************************/
extern char Init_TWI(void);		//Initialize TWI
extern unsigned char Send_Data_TWI(unsigned char *data_buf, unsigned char leng);
extern unsigned char Read_Data_TWI(unsigned char mode, unsigned char *data_buf, unsigned char leng);
extern unsigned char Poll_NAK_TWI(unsigned char device);

/****************************************************************************
Bit and byte definitions
****************************************************************************/
#define W 		0		//Data transfer direction WRITE
#define R 		1		//Data transfer direction READ
#define OWN_ADR 	60  		//Own slave address
#define SUCCESS 	0xFF

/****************************************************************************
TWI Status register definitions
****************************************************************************/
//General Master status codes											
#define START		0x08		//START has been transmitted	
#define	REP_START	0x10		//Repeated START has been 
					//transmitted		
//Master Transmitter staus codes											
#define	MTX_ADR_ACK	0x18		//SLA+W has been tramsmitted
					//and ACK received	
#define	MTX_ADR_NACK	0x20		//SLA+W has been tramsmitted
					//and NACK received		
#define	MTX_DATA_ACK	0x28		//Data byte has been tramsmitted
					//and ACK received			
#define	MTX_DATA_NACK	0x30		//Data byte has been tramsmitted
					//and NACK received			
#define	MTX_ARB_LOST	0x38		//Arbitration lost in SLA+W or 
					//data bytes	
//Master Receiver staus codes	
#define	MRX_ARB_LOST	0x38		//Arbitration lost in SLA+R or 
					//NACK bit
#define	MRX_ADR_ACK	0x40		//SLA+R has been tramsmitted
					//and ACK received	
#define	MRX_ADR_NACK	0x48		//SLA+R has been tramsmitted
					//and NACK received		
#define	MRX_DATA_ACK	0x50		//Data byte has been received
					//and ACK tramsmitted
#define	MRX_DATA_NACK	0x58		//Data byte has been received
					//and NACK tramsmitted
