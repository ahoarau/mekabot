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


#ifndef _ADC_SPI_H
#define _ADC_SPI_H
#ifdef USE_ADC_SPI

//Setup for use with XYZ SPI ADC unit

void setup_adc_spi(void);
void step_adc_spi();


unsigned int get_avg_adc_spi(int chid);
void setup_adc_spi(void);
unsigned int get_adc_spi(int chid);

#if defined MAX2_BDC_0_2_T2R3 || defined MAX2_BLDC_0_2_T2R3

#define ADC_SPI_NUM_CH 1
#define ADC_SPI_NUM_SMOOTH	2 //Must be even
#define ADC_SPI_SHIFT_SMOOTH 1 //2^ADC_SHIFT_SMOOTH
//Define Bit-bang pins for XYZ SPI ADC unit

#define	BB_ASPI_SS				LATBbits.LATB3		//RB3	OUTPUT	PIN24	SPI_SS_ADC/
#define	BB_ASPI_CLK				LATCbits.LATC1		//RC1	OUTPUT	PIN26	SPI_CLK_ADC
#define BB_PIN_ASPI_DI			PORTBbits.RB6		//RB6	INPUT	PIN42	SPI_DI_ADC

#define BB_ASPI_SS_SET			BB_ASPI_SS=1; 
#define BB_ASPI_SS_CLR			BB_ASPI_SS=0; 
#define BB_ASPI_CLK_SET			BB_ASPI_CLK=1; 
#define BB_ASPI_CLK_CLR			BB_ASPI_CLK=0; 


////Unlike the Vert-X encoder, the MCP3201 outputs dat on the FALLING EDGDE of CLK. But the CLK should still be initialized at 0.
////The Bit-bang sequence below is "time optimized" and reads the sensor ADC value during the low level of CLK 

//Max CLK is 0.8MHz (period of 1250NS, so half period of 625NS)
//Also, minimum 200NS between falling edge of CLK and valid output of the ADC sensor
//The specs below should respect those criterias
#define BB_ASPI_BIT_IN(x)	{				\
							BB_ASPI_CLK_SET\
							DELAY_700NS\
							BB_ASPI_CLK_CLR\
							DELAY_225NS\
							x=x|BB_PIN_ASPI_DI;	\
							DELAY_500NS\
							}
#endif

#endif
#endif
