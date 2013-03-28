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



#ifndef __DAC_H__
#define __DAC_H__ 
#ifdef USE_DAC

void setup_dac();
void set_dac(int val); 
int dac_cmd();


#ifdef M3_DEV
#define DAC_MAX_VAL   4095	
#define DAC_ZERO_VAL 2048
#define DAC_MAX_DUTY 2048
#define	BB_DAC_SEL					LATBbits.LATB4		//RB4		OUTPUT	PIN33	SPI_SEL
#define	BB_DAC_CLK					LATBbits.LATB15		//RB15	OUTPUT	PIN15	SPI_CLK
#define	BB_DAC_DI					LATBbits.LATB13		//RB14	OUTPUT	PIN13	SPI_DI
#define BB_DAC_CS0					LATBbits.LATB8		//RB8	OUTPUT	PIN44	SPI_MUX_CS0
#define BB_DAC_CS1					LATBbits.LATB9		//RB9	OUTPUT	PIN1	SPI_MUX_CS1
#define BB_PIN_SPI_DI				PORTBbits.RB13
#endif

#if defined M3_DAC_0_1 || defined M3_ELMO_RNA_R0 || defined M3_ELMO_B1R1 || defined M3_ELMO_Z1R1


#define DAC_NUM_SMOOTH	64 //Must be even
#define DAC_SHIFT_SMOOTH 6 //2^ADC_SHIFT_SMOOTH


#define DAC_MAX_VAL   4095	
#define DAC_ZERO_VAL 2048
#define DAC_MAX_DUTY 2048
#define	BB_DAC_CLK					LATBbits.LATB15		//RB15	OUTPUT	PIN15	RP15_PWM (SPI_CLK_DAC)
#define	BB_DAC_SEL					LATBbits.LATB14		//RB14	OUTPUT	PIN14	RP14_PWM (SPI_SS0/_DAC)
#define	BB_DAC_DI					LATBbits.LATB13		//RB13	OUTPUT	PIN11	RP13_PWM (SPI_DI_DAC)
#define BB_PIN_SPI_DI				PORTBbits.RB13
#endif

#endif
#endif
