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


#ifndef _BLDC_H
#define _BLDC_H
#ifdef USE_BLDC

//Setup for use with BLDC commutation

void setup_bldc(void);
void set_bldc_dir(unsigned int fwd);
int get_hall_state();
int get_bldc_dir();
void set_bldc_mode(int bldc_bdc_mode);

void set_bldc_open();
void set_bldc_brake();
void set_bldc_commutation();

extern unsigned int bldc_hall_val;

#define BLDC_PIN_1  0
#define BLDC_PIN_2  1
#define BLDC_PIN_3  2

#define BLDC_CN_PIN_1 4
#define BLDC_CN_PIN_2 5
#define BLDC_CN_PIN_3 6

#define CAT2(a,b)  a ## b
#define CAT(a,b) CAT2(a,b)

// ec 32 flat ribbon cable
#define BLDC_HALL_1 CAT(PORTBbits.RB,BLDC_PIN_1)	//RB4	INPUT	PIN33	RP4/CN1 (HALL1)
#define BLDC_HALL_2 CAT(PORTBbits.RB,BLDC_PIN_2)		//RB9	INPUT	PIN1	RP9/CN21 (HALL2)
#define BLDC_HALL_3 CAT(PORTBbits.RB,BLDC_PIN_3)		//RB8	INPUT	PIN44	RP8/CN22 (HALL3)

#define BLDC_PU_1 CAT(CAT(CNPU1bits.CN,BLDC_CN_PIN_1),PUE)  //Enable weak pull-up on CN1
#define BLDC_PU_2 CAT(CAT(CNPU1bits.CN,BLDC_CN_PIN_2),PUE)
#define BLDC_PU_3 CAT(CAT(CNPU1bits.CN,BLDC_CN_PIN_3),PUE)
#define BLDC_CN_1 CAT(CAT(CNEN1bits.CN,BLDC_CN_PIN_1),IE)
#define BLDC_CN_2 CAT(CAT(CNEN1bits.CN,BLDC_CN_PIN_2),IE)
#define BLDC_CN_3 CAT(CAT(CNEN1bits.CN,BLDC_CN_PIN_3),IE)

#define BLDC_HALL_STATE (BLDC_HALL_3<<2)|(BLDC_HALL_2<<1)|BLDC_HALL_1
#endif
#endif
