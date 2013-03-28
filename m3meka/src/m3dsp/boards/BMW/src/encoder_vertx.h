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


#ifndef _ENCODER_VERTX_H
#define _ENCODER_VERTX_H
#ifdef USE_ENCODER_VERTX

//Setup for use with ContElec VERTX Absolute Encoder
//http://www.novotechnik.com/products/rotary/non-contact_rotary.html

void setup_vertx(void);
unsigned int vertx_pos(int chid);
int vertx_error(int chid);
unsigned int get_avg_vertx(int ch);
#define NUM_VERTX_CH 2

#define VERTX_NUM_SMOOTH	2 //Must be even
#define VERTX_SHIFT_SMOOTH 1 //2^ADC_SHIFT_SMOOTH

void step_vertx();

#define VERTX_CH_A 0
#define VERTX_CH_B 1

#define VERTX_CH_SEAS VERTX_CH_A
#define VERTX_CH_ENC VERTX_CH_B
#define VERTX_BOTH

//ToDo set pins
#define	BB_SPI_SS_A					LATBbits.LATB4		//RB4	OUTPUT	PIN33	SPI_SS_SEAS/
#define	BB_SPI_SS_B					LATBbits.LATB3		//RB3	OUTPUT	PIN24	SPI_SS_ENC/
#define	BB_SPI_CLK_A				LATBbits.LATB9		//RB9	OUTPUT	PIN1	SPI_CLK_SEAS
#define	BB_SPI_CLK_B				LATCbits.LATC1		//RC1	OUTPUT	PIN26	SPI_CLK_ENC
#define	BB_SPI_DIO_A				LATBbits.LATB5		//RB5	INPUT	PIN41	SPI_DIO_SEAS
#define	BB_SPI_DIO_B				LATBbits.LATB6		//RB6	INPUT	PIN42	SPI_DIO_ENC
#define BB_PIN_SPI_DIO_A			PORTBbits.RB8
#define BB_PIN_SPI_DIO_B			PORTBbits.RB6

#define BB_SPI_SET_MOSI_A	TRISBbits.TRISB8=0; ODCBbits.ODCB8=1; //Make pin output, open collector
#define BB_SPI_SET_MISO_A	TRISBbits.TRISB8=1; ODCBbits.ODCB8=0; //Make pin input, non open collector
#define BB_SPI_SET_MOSI_B	TRISBbits.TRISB6=0; ODCBbits.ODCB6=1; //Make pin output, open collector
#define BB_SPI_SET_MISO_B	TRISBbits.TRISB6=1; ODCBbits.ODCB6=0; //Make pin input, non open collector

#if defined VERTX_CH_A_ONLY

#define BB_SPI_SS_SET				BB_SPI_SS_A=1; 
#define BB_SPI_SS_CLR				BB_SPI_SS_A=0; 
#define BB_SPI_CLK_SET				BB_SPI_CLK_A=1; 
#define BB_SPI_CLK_CLR				BB_SPI_CLK_A=0; 
#define BB_SPI_DIO_SET				BB_SPI_DIO_A=1; 
#define BB_SPI_DIO_CLR				BB_SPI_DIO_A=0; 
#define BB_SPI_SET_MOSI		BB_SPI_SET_MOSI_A 
#define BB_SPI_SET_MISO		BB_SPI_SET_MISO_A 

//Takes ~2500NS
#define BB_SPI_BIT_OUT(x,y) {				\
							BB_SPI_CLK_SET\
							BB_SPI_DIO_A=x;	\
							DELAY_1200NS\
							BB_SPI_CLK_CLR\
							DELAY_1200NS\
							}

#define BB_SPI_BIT_IN(x,y) {				\
							BB_SPI_CLK_SET\
							DELAY_1200NS\
							BB_SPI_CLK_CLR\
							x=x|BB_PIN_SPI_DIO_A;	\
							y=0;	\
							DELAY_1200NS\
							}
#endif

#if defined VERTX_CH_B_ONLY

#define BB_SPI_SS_SET				BB_SPI_SS_B=1; 
#define BB_SPI_SS_CLR				BB_SPI_SS_B=0; 
#define BB_SPI_CLK_SET				BB_SPI_CLK_B=1; 
#define BB_SPI_CLK_CLR				BB_SPI_CLK_B=0; 
#define BB_SPI_DIO_SET				BB_SPI_DIO_B=1; 
#define BB_SPI_DIO_CLR				BB_SPI_DIO_B=0; 
#define BB_SPI_SET_MOSI		BB_SPI_SET_MOSI_B 
#define BB_SPI_SET_MISO		BB_SPI_SET_MISO_B 

//Takes ~2500NS
#define BB_SPI_BIT_OUT(x,y) {				\
							BB_SPI_CLK_SET\
							BB_SPI_DIO_B=x;	\
							DELAY_1200NS\
							BB_SPI_CLK_CLR\
							DELAY_1200NS\
							}

#define BB_SPI_BIT_IN(x,y) {				\
							BB_SPI_CLK_SET\
							DELAY_1200NS\
							BB_SPI_CLK_CLR\
							x=x|BB_PIN_SPI_DIO_B;	\
							y=0;	\
							DELAY_1200NS\
							}
#endif

#if defined VERTX_BOTH

#define BB_SPI_SS_SET				BB_SPI_SS_A=1; BB_SPI_SS_B=1; 
#define BB_SPI_SS_CLR				BB_SPI_SS_A=0; BB_SPI_SS_B=0; 
#define BB_SPI_CLK_SET				BB_SPI_CLK_A=1; BB_SPI_CLK_B=1; 
#define BB_SPI_CLK_CLR				BB_SPI_CLK_A=0; BB_SPI_CLK_B=0; 
#define BB_SPI_DIO_SET				BB_SPI_DIO_A=1; BB_SPI_DIO_B=1; 
#define BB_SPI_DIO_CLR				BB_SPI_DIO_A=0; BB_SPI_DIO_B=0; 
#define BB_SPI_SET_MOSI		BB_SPI_SET_MOSI_A BB_SPI_SET_MOSI_B
#define BB_SPI_SET_MISO		BB_SPI_SET_MISO_A BB_SPI_SET_MISO_B

//Takes ~2500NS
#define BB_SPI_BIT_OUT(x,y) {				\
							BB_SPI_CLK_SET\
							BB_SPI_DIO_A=x;	\
							BB_SPI_DIO_B=y;	\
							DELAY_1200NS\
							BB_SPI_CLK_CLR\
							DELAY_1200NS\
							}

#define BB_SPI_BIT_IN(x,y) {				\
							BB_SPI_CLK_SET\
							DELAY_1200NS\
							BB_SPI_CLK_CLR\
							x=x|BB_PIN_SPI_DIO_A;	\
							y=y|BB_PIN_SPI_DIO_B;	\
							DELAY_1200NS\
							}
#endif



#endif
#endif
