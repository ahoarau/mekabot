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
extern unsigned int bldc_hall_val;

#define BLDC_HALL_1 PORTBbits.RB4		//RB4	INPUT	PIN33	RP4/CN1 (HALL1) 
#define BLDC_HALL_2 PORTBbits.RB9		//RB9	INPUT	PIN1	RP9/CN21 (HALL2)
#define BLDC_HALL_3 PORTBbits.RB8		//RB8	INPUT	PIN44	RP8/CN22 (HALL3)

#define BLDC_HALL_STATE (BLDC_HALL_3<<2)|(BLDC_HALL_2<<1)|BLDC_HALL_1
#endif
#endif
