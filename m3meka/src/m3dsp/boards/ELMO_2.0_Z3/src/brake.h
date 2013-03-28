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

#ifndef __BRAKE_H__
#define __BRAKE_H__ 

#ifdef USE_BRAKE

void setup_brake();
//Off: brake is disabled and motor can turn
//On: brake is enabled, motor is locked
void step_brake(int off);
void force_brake_on();
void reset_force_brake_on();

#if defined MAX2_BDC_0_3_T2R2
#define SetBrakeOff 	LATCbits.LATC0 = 1; //RC0	OUTPUT	PIN25	Brake enable
#define SetBrakeOn		LATCbits.LATC0 = 0;
#endif

#if defined MAX2_BDC_0_2_T2R3
#define SetBrakeOff 	LATBbits.LATB1=1; //RB1	OUTPUT	PIN22	Brake enable
#define SetBrakeOn		LATBbits.LATB1=0;
#endif

#ifdef M3_ELMO_Z1R1
#define SetBrakeOff 	LATCbits.LATC2=1; //RC2	OUTPUT	PIN27	Brake enable
#define SetBrakeOn		LATCbits.LATC2=0;
#endif

#endif
#endif
