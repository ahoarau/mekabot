#ifdef USE_AS5510

#include <i2c.h>
#include "setup.h"
#include "encoder_as5510.h"

//Global variables:
unsigned char as5510_i2c_buffer = 0x00;	//I2C data
short as5510_raw_value = 0x00;			//10-bit output value (0~1023), 511 is the middle point @ 0mT
short as5510_value = 0x00;				//Decoded value, in mT

/*=============================================================================
AS5510 Encoder Get Value
=============================================================================*/ 
short get_as5510_raw()
{
	return as5510_raw_value;
}

/*=============================================================================
AS5510 Encoder Reading
=============================================================================*/  
void step_as5510()
{
	as5510_value = get_magnetic_field(0);
	//as5510_raw_value=as5510_raw_value+1;		//Test only
}
/*=============================================================================
AS5510 Encoder Setup
=============================================================================*/  
void setup_as5510(void)
{
	unsigned char mode;
	
	//Configure I2C
	setup_I2C1();

	//Setup sensor
	mode = (AS5510_MODE_SLOW | AS5510_POLARITY_NORMAL | AS5510_POWERDOWN_OFF);	//Default
	configure_as5510(mode, AS5510_SENSITIVITY_12MT);

	//Note: must wait 1.5ms between config and first reading
//	ms_delay(2);
}

/*=============================================================================
AS5510 Encoder Setup
=============================================================================*/ 
void configure_as5510(unsigned char mode, unsigned char sensitivity)
{
	write_byte_i2c(AS5510_I2C_ADDR, 0x02, mode);
	write_byte_i2c(AS5510_I2C_ADDR, 0x0B, sensitivity);
}


/*=============================================================================
Read a byte from I2C slave
=============================================================================*/  
unsigned char read_byte_i2c(unsigned char slave_adress, unsigned char offset)
{
	//Write slave adress
	IdleI2C1();
	StartI2C1();
	while(I2C1CONbits.SEN);					//Wait till Start sequence is completed
	MasterWriteI2C1(slave_adress & 0xFE);	//Write Slave address and set master for writing
	while(I2C1STATbits.TBF);				//Wait till address is transmitted
	while(I2C1STATbits.ACKSTAT);
	
	//Write memory positon (offset)
	IdleI2C1();
	MasterWriteI2C1(offset);
	while(I2C1STATbits.TBF);				//Wait till data is transmitted
	while(I2C1STATbits.ACKSTAT);

	//Write slave adress
	IdleI2C1();
	RestartI2C1();							//Restart
	while(I2C1CONbits.RSEN);				//Wait till Start sequence is completed
	MasterWriteI2C1(slave_adress | 0x01);	//Write Slave address and set master for reading
	while(I2C1STATbits.TBF);				//Wait till address is transmitted
	while(I2C1STATbits.ACKSTAT);
	
	//Read a byte
	as5510_i2c_buffer = MasterReadI2C1();

	//End communication
	NotAckI2C1();
	StopI2C1();

	//Return value
	return as5510_i2c_buffer;
}

/*=============================================================================
Write a byte to an I2C slave
=============================================================================*/  
void write_byte_i2c(unsigned char slave_adress, unsigned char offset, char data)
{
	//Write slave adress
	IdleI2C1();
	StartI2C1();
	while(I2C1CONbits.SEN);					//Wait till Start sequence is completed
	MasterWriteI2C1(slave_adress & 0xFE);	//Write Slave address and set master for writing
	while(I2C1STATbits.TBF);				//Wait till address is transmitted
	while(I2C1STATbits.ACKSTAT);
	
	//Write memory positon (offset)
	IdleI2C1();
	MasterWriteI2C1(offset);
	while(I2C1STATbits.TBF);				//Wait till data is transmitted
	while(I2C1STATbits.ACKSTAT);	
	
	//Write data
	IdleI2C1();
	MasterWriteI2C1(data);
	while(I2C1STATbits.TBF);				//Wait till data is transmitted
	while(I2C1STATbits.ACKSTAT);
	
	//End communication
	NotAckI2C1();
	StopI2C1();
}

/*=============================================================================
AS5510 Encoder Read
=============================================================================*/  
//Modified from code by Austria Micro 
short get_magnetic_field(unsigned char sensitivity_mode)
{
	unsigned char Data_LSB, Data_MSB;
							// 
	float magnetic_field = 0;	// Value of the magnetic field in mT

	Data_LSB = read_byte_i2c(AS5510_I2C_ADDR, 0x00);	// Read D7..0
	Data_MSB = read_byte_i2c(AS5510_I2C_ADDR, 0x01);	// Read D9..8 + OCF + Parity
	as5510_raw_value = ((Data_MSB & 0x03)<<8) + Data_LSB;

	//Uncomment to get value in mT:
//	switch (sensitivity_mode)	// Sensitivity_Mode is the value stored in register 0Bh
//	{
//		case 0: // Register [0Bh] <= 0 (+- 50mT range, 97.66uT/LSB)
//			magnetic_field = (as5510_raw_value - 511) * 0.09766;
//			break;
//		case 1: // Register [0Bh] <= 0 (+- 25mT range, 48.83uT/LSB)
//			magnetic_field = (as5510_raw_value - 511) * 0.04883;
//			break;
//		case 2: // Register [0Bh] <= 0 (+- 12.5mT range, 24.41uT/LSB)
//			magnetic_field = (as5510_raw_value - 511) * 0.02441;
//			break;
//		case 3: // Register [0Bh] <= 0 (+- 18.7mT range, 36.62uT/LSB)
//			magnetic_field = (as5510_raw_value - 511) * 0.03662;
//			break;
//	}
	//printf("Decimal 10-bit value = %u \n", as5510_raw_value);
	//printf("Magnetic field value = %imT \n", magnetic_field);

	return magnetic_field;
}

/*=============================================================================
I2C Peripheral Initialisation
=============================================================================*/   
void setup_I2C1(void)
{   
	// Configure ASCL1/ASDA1 pin as open-drain (RB5 & RB6)
	ODCBbits.ODCB5 =1 ;
	ODCBbits.ODCB6 = 1;

	//I2C1 Master configuration 100kHz
	I2C1CONbits.I2CEN = 0;		//Disable module
	I2C1CONbits.A10M = 0;		//7-bit adress
	I2C1CONbits.SCLREL = 1;		//Stretch clock if slave
	I2C1BRG = 0x21;				//0x21 = 1MHz, 0x2B = 800kHz, 0x5D = 400kHz, 0x188 = 100kHz
	I2C1ADD = 0;				//Address
	I2C1MSK = 0;				//No mask

	//Enable and INT On
	I2C1CONbits.I2CEN = 1;
	IEC1bits.MI2C1IE = 1;
  	IFS1bits.MI2C1IF = 0;
}

/*=============================================================================
I2C Master Interrupt Service Routine
=============================================================================*/
void __attribute__((interrupt, no_auto_psv)) _MI2C1Interrupt(void)
{
	IFS1bits.MI2C1IF = 0;		//Clear the DMA0 Interrupt Flag;

}

/*=============================================================================
I2C Slave Interrupt Service Routine
=============================================================================*/
void __attribute__((interrupt, no_auto_psv)) _SI2C1Interrupt(void)
{	
	IFS1bits.SI2C1IF = 0;		//Clear the DMA0 Interrupt Flag
}

#endif

/*
To add to other project files:
==============================

//Alternate I2C
_FPOR(ALTI2C_ON); // ALTI2C = 0: I2C mapped to ASDA1/ASCL1
*/
