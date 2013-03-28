/* 
M3 -- Meka Robotics Real-Time Control System
Copyright (c) 2010 Meka Robotics
Author: edsinger@mekabot.com (Aaron Edsinger)

M3 is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

M3 is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with M3.  If not, see <http://www.gnu.org/licenses/>.
*/


#include "p33Fxxxx.h"
#include "setup.h"

void setup_oscillator(void);
void setup_ports(void);
void setup_peripheral_pin_select(void);
void setup_interrupt_priorities(void);


void setup_interrupt_priorities(void)
{
	//Higher number = Higher priority

	//Ethercat Master AL interrupt on INT0

	//Apps inherit Bootloader settings so need set here for Bootloader too.
	//ToDo:FIX!

	_INT0IP=3;	//Ethercat Interrupt
	_INT2IP=2;	//SYNC0 Interrupt

	_T3IP=5;	//Timer3 

}


void setup_oscillator(void)
{
	/*
	Need to Multiply FIN by factor of 3.2
	FIN (INPUT FREQ)	25MHZ	FOR OSCILLATOR FOR ET1200	
	N1	25	PLLPRE	
	N2	2	PLLPOST	
	M	160	PLLDIV	
	FOSC=FIN*(M/(N1*N2))	80MHZ		
	FCY=FOSC/2	GIVES A 40 MIPS	OUTPUT FREQUENCY	
	*/
	PLLFBD=160;
	CLKDIVbits.PLLPOST=0;//Gives divide by 2
	CLKDIVbits.PLLPRE=23; //Gives divide by 25
	// the CPU will automatically switch when all is stable....
	while(OSCCONbits.LOCK!=1) {};  // Wait for PLL to lock
	return;
}

void setup_ports(void)
{

	// Clear All Ports Prior to defining I/O
	PORTA=0;
	PORTB=0;
	PORTC=0;

	/* 
	TRIS<X>=1 MAKES PIN AS INPUT	
	TRIS<X>=0 MAKES PIN AS OUTPUT	
	AFTER RESET, ALL PORT PINS ARE INPUTS			
	PORT<X> READS OR WRITES TO PIN			
	ODC<X>=1 SETS PIN TO OPEN DRAIN OUTPUT			
	OPEN DRAIN WITH 5V PULLUP ALLOWS 5V OUTPUT			
	FOR ADC MUST SET TRIS<X>=1		

	TRISA			
	RA0	INPUT	PIN19	AN0
	RA1	INPUT	PIN20	AN1
	RA2	INPUT	PIN30	OSCI
	RA3	INPUT	PIN31	NC
	RA4	OUTPUT	PIN34	HEARTBEAT LED
	RA7	INPUT	PIN13	NC
	RA8	INPUT	PIN32	EEPROM_LOADED
	RA9	INPUT	PIN35	NC
	RA10 INPUT	PIN12	NC

	TRISB			
	RB0	INPUT	PIN21	AN2
	RB1	INPUT	PIN22	AN3
	RB2	INPUT	PIN23	AN4
	RB3	INPUT	PIN24	AN5
	RB4	INPUT	PIN33	RP4
	RB5	INPUT	PIN41	RP5
	RB6	INPUT	PIN42	RP6
	RB7	INPUT	PIN43	SPI_IRQ
	RB8	INPUT	PIN44	RP8_UART_RX
	RB9	OUTPUT	PIN1	RP9_UART_TX
	RB10	INPUT	PIN8	PGM_PGD
	RB11	INPUT	PIN9	PGM_PGC
	RB12	OUTPUT	PIN10	RP12_PWM
	RB13	OUTPUT	PIN11	RP13_PWM
	RB14	OUTPUT	PIN14	RP14_PWM
	RB15	OUTPUT	PIN15	RP15_PWM

	TRISC			
	RC0	INPUT	PIN25	AN6
	RC1	INPUT	PIN26	AN7
	RC2	INPUT	PIN27	AN8
	RC3	INPUT	PIN36	SYNC_1
	RC4	INPUT	PIN37	SYNC_0
	RC5	INPUT	PIN38	NC
	RC6	OUTPUT	PIN2	SPI_SEL
	RC7	OUTPUT	PIN3	SPI_CLK
	RC8	OUTPUT	PIN4	SPI_DO
	RC9	INPUT	PIN5	SPI_DI
	*/


	// Hardwired pins
	TRISAbits.TRISA4=0;		//RA4	OUTPUT	PIN34	HEARTBEAT LED
	TRISAbits.TRISA8=1;		//RA8	INPUT	PIN32	EEPROM_LOADED
	TRISBbits.TRISB7=1;		//RB7	INPUT	PIN43	SPI_IRQ
	TRISBbits.TRISB10=1;	//RB10	INPUT	PIN8	PGM_PGD
	TRISBbits.TRISB11=1;	//RB10	INPUT	PIN8	PGM_PGD
	TRISCbits.TRISC3=0;		//RC3	OUTPUT	PIN36	LATCH_1
	TRISCbits.TRISC4=1;		//RC4	INPUT	PIN37	SYNC_0
	TRISCbits.TRISC6=0;		//RC6	OUTPUT	PIN2	SPI_SEL
	TRISCbits.TRISC7=0;		//RC7	OUTPUT	PIN3	SPI_CLK
	TRISCbits.TRISC8=0;		//RC8	OUTPUT	PIN4	SPI_DO
	TRISCbits.TRISC9=1;		//RC9	INPUT	PIN5	SPI_DI

	


//Controller for the 16 channels LED Drivers
	TRISCbits.TRISC1=0;		//RC1	OUTPUT	PIN26	RP17_AN7		LED_MDRV_DATA
	TRISCbits.TRISC0=0;		//RC0	OUTPUT	PIN25	RP16_AN6		LED_MDRV_SCLK
	TRISBbits.TRISB3=0;		//RB3	OUTPUT	PIN24	RP3_AN5			LED_MDRV_DCSEL
	TRISBbits.TRISB2=0;		//RB2	OUTPUT	PIN23	RP2_AN4			LED_MDRV_XLAT
	TRISCbits.TRISC2=0;		//RC2	OUTPUT	PIN27	RP18_AN8		LED_MDRV_BLANK
	TRISBbits.TRISB12=0;	//RB12	OUTPUT	PIN10	RP12_PWM1H2		LED_MDRV_GSCLK
	TRISBbits.TRISB6=1;		//RB6	INTPUT	PIN42	RP6				LED_MDRV_READ
	TRISBbits.TRISB5=1;		//RB5	INTPUT	PIN41	RP5				LED_MDRV_XERR

//Decoder outputs for row selection
	TRISBbits.TRISB4=0;		//RB4	OUTPUT	PIN33	RP4				LED_MDRV_A_DEC
	TRISBbits.TRISB8=0;		//RB8	OUTPUT	PIN44	RP8				LED_MDRV_B_DEC
	TRISBbits.TRISB9=0;		//RB9	OUTPUT	PIN1	RP9				LED_MDRV_C_DEC
	TRISBbits.TRISB13=0;	//RB13	OUTPUT	PIN11	RP13			LED_MDRV_EN_DEC

	ODCBbits.ODCB4=1;		//RB4 open-drain for 5V output (pull-up resistor to D5V on the board)
	ODCBbits.ODCB8=1;		//RB8 open-drain for 5V output (pull-up resistor to D5V on the board)
	ODCBbits.ODCB9=1;		//RB9 open-drain for 5V output (pull-up resistor to D5V on the board)
	ODCBbits.ODCB13=1;		//RB13 open-drain for 5V output (pull-up resistor to D5V on the board)



	return;
}

//Note: If using bootloader, only the first call (Bootloader) works
//Second call by app doesn't work. So for now, bootloader peripheral config.
//must support all devices.
//TODO: leave PPS unlocked after one call. 

void setup_peripheral_pin_select(void)
{
	//Unlock ala datasheet #OSCONL 
	asm volatile (	"mov #0x742, w1 \n"
					"mov #0x46, w2 \n"
					"mov #0x57, w3 \n"
					"mov.b w2, [w1] \n"
					"mov.b w3, [w1] \n"
					"bclr 0x742, #0x6");

	//RPINRX AND RPORX ARE SET TO 0X0000 AFTER RESET
	//THEREFORE ALL INPUTS INITIALLY TIED TO RP0
	//WHICH IS AN2  IN THIS DESIGN.
	//THIS SEEMS OK...
	//RP21 IS NC SO THIS ALL UNUSED PERIPHERAL INPUTS
	//SHOULD BE DISABLED AND MAPPED TO THIS PIN
	//ALL UNUSED OUTPUTS GET MAPPED TO VSS
	

	////// Inputs  /////
	//RPINR0bits.INT1R=19;	//(SYNC1) INT1 ON RP19 :					RPINR0
	RPINR1bits.INT2R=20;	//(SYNC0) INT2 ON RP20 :					RPINR1	
	RPINR20bits.SDI1R=25;	//SPI DATA IN ON RP25 :						RPINR20	
	////// Outputs /////
	RPOR11bits.RP22R = 0b01001;  //SPI SLAVE SELECT OUT ON RP22 :			RPOR11
	RPOR11bits.RP23R = 0b01000; //SPI CLOCK OUT ON RP23 :					RPOR11	
	RPOR12bits.RP24R = 0b00111; //SPI DATA OUT ON RP24  :					RPOR12	



	//Lock ala datasheet
	asm volatile (	"mov #0x742, w1 \n"
				"mov #0x46, w2 \n"
				"mov #0x57, w3 \n"
				"mov.b w2, [w1] \n"
				"mov.b w3, [w1] \n"
				"bset 0x742, #0x6");
	return;
}


void us_delay(int n) {
  int i,j;
  for (i=0;i<n;i++) {
    for(j=0;j<US_DELAY_CONST;j++) ;
  }
}
void ms_delay(int n) {
  int i,j;
  for (i=0;i<n;i++) {
    for(j=0;j<MS_DELAY_CONST;j++) ;
  }
}

void us100_delay(int n) {
  int i,j;
  for (i=0;i<n;i++) {
    for(j=0;j<US100_DELAY_CONST;j++) ;
  }
}
