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


#ifndef _TACTILE_PPS_H
#define _TACTILE_PPS_H
#ifdef USE_TACTILE_PPS


void setup_pps(void);
void step_pps(int chid);
unsigned int get_taxel(int chid, int tid);




#ifdef M3_GMB_G1R1
#define NUM_PPS_CH 2
#define NUM_PPS_TAXEL 22

/* SPI COMMANDS FOR TACTILE SENSOR */

//#define CAPSENSEON 0x10
//#define CAPSENSEOFF 0x11
//#define SCANON 0x12
//#define SCANOFF 0x13
//#define READDATA 0x14


#define	PPS_SPI_DI	LATBbits.LATB8		//RB8	INPUT	PIN44	RP8 (MISO)
#define	PPS_SPI_DO	LATBbits.LATB9		//RB9	OUTPUT	PIN1	RP9 (MOSI)
#define	PPS_SPI_CLK	LATBbits.LATB4		//RB4	OUTPUT	PIN33	RP4 (CLK)
#define	PPS_SPI_SS0	LATBbits.LATB11		//RB11	OUTPUT	PIN9	RP11 (SEL)
#define	PPS_SPI_SS1	LATBbits.LATB10		//RB10	OUTPUT	PIN8	RP10 (SEL)
#define PIN_PPS_SPI_DI	PORTBbits.RB8

#endif
//Writes bit x onto RP9 (MOSI) with the proper clock timing 
#define PPS_SPI_BIT_OUT(x) {				\
							PPS_SPI_DO=x;	\
							PPS_SPI_CLK=1;	\
							DELAY_75NS\
							PPS_SPI_CLK=0;	\
							DELAY_75NS\
							}
//Reads bit x from RB8 (MISO) with the proper clock timing
/*#define PPS_SPI_BIT_IN(x) {				\
							PPS_SPI_CLK=1;	\
							x=x|PIN_PPS_SPI_DI;	\
							DELAY_75NS\
							DELAY_75NS\
							PPS_SPI_CLK=0;	\
							DELAY_75NS\
							DELAY_75NS\
							}*/
#define PPS_SPI_BIT_IN(x) {				\
							PPS_SPI_CLK=1;	\
							x=PIN_PPS_SPI_DI;	\
							DELAY_75NS\
							PPS_SPI_CLK=0;	\
							DELAY_75NS\
							}
#endif
#endif
