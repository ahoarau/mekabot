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



#ifndef __LED_RGB_H__
#define __LED_RGB__ 
#ifdef USE_LED_RGB

void setup_led_rgb(); 
void step_led_rgb();

#if defined M3_LEDX2_S1R1 || defined M3_LEDX2XN_S2R1
//Controller for the MaceTech Megabrite controller
#define LED_RGB_NUM_BRANCH 2
#if defined M3_LEDX2_S1R1
#define LED_RGB_BOARDS_PER_BRANCH 2
#endif
#if defined M3_LEDX2XN_S2R1
#define LED_RGB_BOARDS_PER_BRANCH 12
#endif
#define LED_RGB_MAX_VAL   1023	
#define	LED_LAT_A					LATBbits.LATB15		//RB15	OUTPUT	PIN15	RP15_PWM		LED_LAT_A
#define	LED_LAT_B					LATBbits.LATB5		//RB5	OUTPUT	PIN41	RP5				LED_LAT_B
#define	LED_CLK_A					LATBbits.LATB13		//RB13	OUTPUT	PIN11	RP13_PWM		LED_CLK_A
#define LED_CLK_B					LATBbits.LATB14		//RB14	OUTPUT	PIN14	RP14_PWM		LED_CLK_B
#define LED_DIO_A					LATBbits.LATB9		//RB9	OUTPUT	PIN1	RP9_UART_TX		LED_DIO_A
#define LED_DIO_B					LATBbits.LATB8		//RB8	OUTPUT	PIN44	RP8_UART_RX		LED_DIO_B
#define LED_EN_A					LATBbits.LATB6		//RB6	OUTPUT	PIN42	RP6				LED_EN_A
#define LED_EN_B					LATBbits.LATB12		//RB12	OUTPUT	PIN10	RP12_PWM		LED_EN_B
//Pins are inverted due to opto-isolation.
#define LED_CLK_SET  LED_CLK_A=0;LED_CLK_B=0;
#define LED_CLK_CLR  LED_CLK_A=1;LED_CLK_B=1;
#define LED_LAT_SET  LED_LAT_A=0;LED_LAT_B=0;
#define LED_LAT_CLR  LED_LAT_A=1;LED_LAT_B=1;
#define LED_AB_BIT_OUT(a,b) {				\
		LED_DIO_A=!a;	\
		LED_DIO_B=!b;	\
		DELAY_25NS\
		LED_CLK_SET\
		DELAY_25NS\
		LED_CLK_CLR\
		DELAY_25NS\
}
#endif

#endif
#endif
