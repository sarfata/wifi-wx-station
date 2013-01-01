/*********** TWI-Driver for Weatherstation   W.Waetzig   3.10.2010
using parts of: AVR155 : Accessing I2C LCD display using the AVR TWI ***********
Device      : 	ATmega88
File name   : 	TWI_driver.c
Ver nr.     : 	1.0
Description : 	TWI_driver.c is a driver to ease the approach to AVRs TWI 
		module. It requires a package containing slave adresse, 
		number of bytes to handle and a pointer. The pointer tells 
		where to find the bytes to send and where the temp buffer for
		bytes to receive are located. Dependent on a succesfull 
		communication or not it will return SUCCESS if no problems or 
		the AVR TWSR if error occured. The driver will handle all 
		signaling and handeling of START/STOP and bus error.
Author      : 	Asmund Saetre
Change log  : 	Created 12.05.2000
Modified    :   12.10.2010  W.W. for WIN-AVR ===> 1 Mhz xtal
****************************************************************************/
#include "TWI_driver.h"												
#if F_CPU < 1600000
#define TWBRX 0
#else
#define TWBRX F_CPU/200000-8
#endif

/****************************************************************************
	Function : char Init_TWI(void)
	Setup the TWI module
	Bitrate 	: 100kHz @ 8MHz system clock  => TWBR=32
	Bitrate 	: 62.6kHz @ 1MHz system clock => TWBR=0
	Own address : OWN_ADR (Defined in TWI_driver.h)
****************************************************************************/
char Init_TWI(void)
{
	TWAR = OWN_ADR;		//Set own slave address
	// bit_rate = F_CPU /(16 + 2*(TWBR)*(prescaler))
	// prescaler in (TWSR): (TWPS1,TWPS0) = (0,0) => prescaler=1
	// TWclock = F_OSC / (16 + 2 * TWBR)
	// TWBR = F_OSC / (2*TWclock) - 8
	TWBR = (TWBRX);		//Set bit-rate to 62.5 KHz with 1 MHz xtal
	TWCR = (1<<TWEN);	//Enable TWI-interface
    return 1;
}    
/****************************************************************************
	Function : void Wait_TWI_int(void)
	Loop until TWI interrupt flag is set
****************************************************************************/
void Wait_TWI_int(void)
{
	while (!(TWCR & (1<<TWINT)))
	    ; 
}    
/****************************************************************************
	Function :unsigned char	Send_start(void)
	Send a START condition to the bus and wait for the TWINT get set set to 
	see the result. If it failed return the TWSR value, if succes return 
	SUCCESS.
****************************************************************************/
unsigned char Send_start(void)
{
	TWCR = ((1<<TWINT)+(1<<TWSTA)+(1<<TWEN));	//Send START
	Wait_TWI_int();			//Wait for TWI interrupt flag set
	if((TWSR != START)&&(TWSR != REP_START))	//If status other than START 
		return TWSR;	//transmitted(0x08) or Repeated
	return SUCCESS;		//START transmitted(0x10) 
    					//-> error  and return TWSR.
    					//If success return	SUCCESS
}							
/****************************************************************************
	Function :
	Send a STOP condition to the bus
****************************************************************************/
void Send_stop(void)
{
	TWCR = ((1<<TWEN)+(1<<TWINT)+(1<<TWSTO));	//Send STOP condition
}        
/****************************************************************************
	Function : unsigned char Send_byte(unsigned char data)
	Send one byte to the bus.
****************************************************************************/
unsigned char Send_byte(unsigned char data)
{
	Wait_TWI_int();				//Wait for TWI interrupt flag set
	TWDR = data;
 	TWCR = ((1<<TWINT)+(1<<TWEN));   	//Clear int flag to send byte 
	Wait_TWI_int();				//Wait for TWI interrupt flag set
	if(TWSR != MTX_DATA_ACK)	//If NACK received return TWSR
		return TWSR;											
	return SUCCESS;				//Else return SUCCESS
}	
/****************************************************************************
	Function : unsigned char Send_adr(unsigned char adr)							
	Send a SLA+W/R to the bus
****************************************************************************/
unsigned char Send_adr(unsigned char adr)
{
	Wait_TWI_int();				//Wait for TWI interrupt flag set
	TWDR = adr;
	TWCR = ((1<<TWINT)+(1<<TWEN));   	//Clear int flag to send byte 
	Wait_TWI_int();				//Wait for TWI interrupt flag set
	if((TWSR != MTX_ADR_ACK)&&(TWSR != MRX_ADR_ACK))
		return TWSR;			//If NACK received return TWSR
	return SUCCESS;				//Else return SUCCESS
}	
/****************************************************************************
	Function : unsigned char Get_byte(unsigned char *dat_ptr,char last_byte)
	Wait for TWINT to receive one byte from the slave and send ACK. 
	If this is the last byte the master will send NACK to tell 
	the slave that it shall stop transmitting.  
****************************************************************************/
unsigned char Get_byte(unsigned char *dat_ptr,char last_byte)
{
	Wait_TWI_int();			//Wait for TWI interrupt flag set
	/* When receiving the last byte from the slave,
	 it will be sent a NACK to make the slave stop transmitting, 
	 all bits before the last will get an ACK*/
	if(last_byte==0)		//Not the last byte
		//Clear int flag and enable acknowledge to receive data.
		TWCR = ((1<<TWINT)+(1<<TWEA)+(1<<TWEN));
	else					//Last byte
		/*Clear int flag to and do not enable acknowledge to tell the slave 
		to stop transmitting*/
		TWCR = ((1<<TWINT)+(1<<TWEN)); 			
	Wait_TWI_int();				//Wait for TWI interrupt flag set

	*dat_ptr = TWDR;			//Save received byte
	/*If ACK has been transmitted or this was the last byte and NACK has been
	sent -> return SUCCESS, else return TWSR*/	
 	if(((TWSR == MRX_DATA_NACK)&&(last_byte != 0))||(TWSR == MRX_DATA_ACK))
		return SUCCESS;	  
	return TWSR;
}
/****************************************************************************
Function : unsigned char Send_Data_TWI(unsigned char *data_buf, unsigned char leng)
	sends leng-1 bytes with data to the slave
	data_buf[0] = slave device address
	data_buf[1] = slave data(1)
	........
	data_buf[leng-1] = slave data(n-1)
****************************************************************************/
unsigned char Send_Data_TWI(unsigned char *data_buf, unsigned char leng)
{
	unsigned char state,i;
	state = Send_start();	//Send START 
	if (state == SUCCESS)				
		state = Send_adr(data_buf[0]+W); //Send slave address+W/R
	for (i=1;(i<leng)&&(state == SUCCESS);i++)
		state = Send_byte(data_buf[i]); //Send slave data
 	Send_stop();	//Send STOP
	return state;	//Return SUCCESS if OK, else return TWSR
} 
/****************************************************************************
Function :	unsigned char Read_Data_TWI(unsigned char mode, 
			unsigned char *data_buf, unsigned char leng)
	reads leng-2 bytes with data to the slave
	data_buf[0] = slave device address
	data_buf[1] = slave function1/address1
	data_buf[2] = slave function2/address2 -- if mode==2
	data_buf[3] = slave data(1)
	data_buf[4] = slave data(2)
		mode == 1:	send slave data-1
		mode == 2:	send slave data-1 + data-2 
****************************************************************************/
unsigned char Read_Data_TWI(unsigned char mode, unsigned char *data_buf, unsigned char leng)
{
	unsigned char state,i,j;
	state = Send_start();	//Send START 
	if (state == SUCCESS)				
		state = Send_adr(data_buf[0]+W); //Send slave address+W/R
	if (state == SUCCESS)				
		state = Send_byte(data_buf[1]);	//Send slave data-1
	if (mode == 2) {
		if (state == SUCCESS)				
			state = Send_byte(data_buf[2]);	//Send slave data-2
	}
	if (state == SUCCESS)
		state = Send_start();	//Send repeated START
	if (state == SUCCESS)				
		state = Send_adr(data_buf[0]+R); //Send slave address+W/R
	for (i=mode+1; (i<leng)&&(state == SUCCESS); i++) {
		j = (i<leng-1) ? 0 : 1;
		state = Get_byte(&data_buf[i],j); //get slave data
	}
 	Send_stop();				//Send STOP
	return state;	//Return SUCCESS if OK, else return TWSR
}
/****************************************************************************
Function : unsigned char Poll_NAK_TWI(unsigned char device)
	polls for the NAK-signal if device is busy
****************************************************************************/
unsigned char Poll_NAK_TWI(unsigned char device)
{
	unsigned char state;
	state = Send_start();	//Send START 
	state = Send_adr(device+W);
 	Send_stop();	
	state = 0;

	while (state != SUCCESS) {	// no ACK
		state = Send_start();	
		state = Send_adr(device+W);
	}
 	Send_stop();	
	return state;	//Return SUCCESS if OK
}



