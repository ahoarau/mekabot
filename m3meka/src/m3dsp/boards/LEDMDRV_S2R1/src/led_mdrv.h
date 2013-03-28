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



#ifndef __LED_MDRV_H__
#define __LED_MDRV_H__ 


void dot_correction_mdrv();
void overcurrent_safety_mdrv();
void setup_led_mdrv();
void select_row_mdrv(int row);
void step_led_mdrv();
int load_led_mdrv();

//Controller for the 16 channels LED Drivers
#define LED_MDRV_NUM_ROWS 8
#define LED_MDRV_NUM_COLUMNS 16
#define LED_MDRV_NUM_COLUMNS_SHIFT 4

#define LED_MDRV_NUM_COLORS 3

#define LED_MDRV_DC_NBITS 6		//Number of bits to code the dot correction is 6
#define LED_MDRV_GS_NBITS 12	//Number of bits to code the gray scale is 12

#define LED_MDRV_MAX_BRIGHTNESS   4095  // This will be compared to R+G+B<<4 (R,G and B are unsigned char, but the brightness variable is coded in 12bits.
									   // 4095 (full 12 bits), corresponds to 24mA. If each pixel (R+G+B) draws more than 24mA, the row driver will reach absolute maximum ratings (~500mA)

#define	LED_MDRV_DATA				LATCbits.LATC1		//RC1	OUTPUT	PIN26	RP17_AN7		LED_MDRV_DATA
#define	LED_MDRV_SCLK				LATCbits.LATC0		//RC0	OUTPUT	PIN25	RP16_AN6		LED_MDRV_SCLK
#define	LED_MDRV_DCSEL				LATBbits.LATB3		//RB3	OUTPUT	PIN24	RP3_AN5			LED_MDRV_DCSEL
#define	LED_MDRV_XLAT				LATBbits.LATB2		//RB2	OUTPUT	PIN23	RP2_AN4			LED_MDRV_XLAT
#define	LED_MDRV_BLANK				LATCbits.LATC2		//RC2	OUTPUT	PIN27	RP18_AN8		LED_MDRV_BLANK
#define	LED_MDRV_GSCLK				LATBbits.LATB12		//RB12	OUTPUT	PIN10	RP12_PWM1H2		LED_MDRV_GSCLK
#define	LED_MDRV_READ				PORTBbits.RB6		//RB6	INTPUT	PIN42	RP6				LED_MDRV_READ
#define	LED_MDRV_XERR				PORTBbits.RB5		//RB5	INTPUT	PIN41	RP5				LED_MDRV_XERR

//Decoder outputs for row selection
#define	LED_MDRV_A_DEC				LATBbits.LATB4		//RB4	OUTPUT	PIN33	RP4				LED_MDRV_A_DEC  !!! Make sure this pin is configured as open collector
#define	LED_MDRV_B_DEC				LATBbits.LATB8		//RB8	OUTPUT	PIN44	RP8				LED_MDRV_B_DEC  !!! Make sure this pin is configured as open collector
#define	LED_MDRV_C_DEC				LATBbits.LATB9		//RB9	OUTPUT	PIN1	RP9				LED_MDRV_C_DEC  !!! Make sure this pin is configured as open collector
#define	LED_MDRV_EN_DEC				LATBbits.LATB13		//RB13	OUTPUT	PIN11	RP13			LED_MDRV_EN_DEC !!! Make sure this pin is configured as open collector

//TSU0 = 5ns min , TSU4 = 10ns min
//TWH0 = 10ns min



#define LED_MDRV_BIT_OUT(x) {				\
							LED_MDRV_DATA=x;\
							DELAY_25NS	\
							LED_MDRV_SCLK=1;\
							DELAY_25NS\
							LED_MDRV_SCLK=0;\
							}
//fclk(gsclk)= 33 MHz max
//fclk(gsclk)= 33 MHz max

#define LED_MDRV_BB_PWM		{						\
								DELAY_25NS			\
								LED_MDRV_GSCLK=1;	\
						   		DELAY_25NS			\
								LED_MDRV_GSCLK=0;	\
							}
							


#endif
