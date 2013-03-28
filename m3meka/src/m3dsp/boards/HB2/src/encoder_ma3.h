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


#ifndef _ENCODER_MA3_H
#define _ENCODER_MA3_H
#ifdef USE_ENCODER_MA3

//Setup for use with US Digital MA3 Absolute Encoder

//10 Bit version
//Min pulse width: 1us = 5 ticks
//Max pulse width: 1024us = 5120 ticks
//Period 1025 us =  5125 ticks
//Angle=((Ton*5125)/(Ton+Toff))-1

//12 Bit version
//Min pulse width: 1us = 5 ticks
//Max pulse width: 4096us = 20480 ticks
//Period 4097 us =  20485 ticks
//Angle=((Ton*20485)/(Ton+Toff))-1

//http://www.usdigital.com/products/ma3/


//#define MA3_10BIT
#define MA3_12BIT


#ifdef MA3_10BIT
#define T2_TICKS_PER_US 5
#define T2_PERIOD_US	2000
#define T2_PERIOD_TICKS	T2_PERIOD_US*T2_TICKS_PER_US
#define MA3_MIN_PULSE_US 1
#define MA3_MAX_PULSE_US 1025
#define MA3_MIN_PULSE_TICKS MA3_MIN_PULSE_US*T2_TICKS_PER_US
#define MA3_MAX_PULSE_TICKS MA3_MAX_PULSE_US*T2_TICKS_PER_US
#define MA3_PERIOD_US  MA3_MAX_PULSE_US+1
#define MA3_PERIOD_TICKS MA3_PERIOD_US*T2_TICKS_PER_US
#define MA3_ROLLOVER_THRESH = MA3_MAX_PULSE_TICKS -500;
#endif

#ifdef MA3_12BIT
#define T2_TICKS_PER_US 5
#define T2_PERIOD_US	5000
#define T2_PERIOD_TICKS	T2_PERIOD_US*T2_TICKS_PER_US
#define MA3_MIN_PULSE_US 1
#define MA3_MAX_PULSE_US 4097
#define MA3_MIN_PULSE_TICKS MA3_MIN_PULSE_US*T2_TICKS_PER_US
#define MA3_MAX_PULSE_TICKS MA3_MAX_PULSE_US*T2_TICKS_PER_US
#define MA3_PERIOD_US  MA3_MAX_PULSE_US+1
#define MA3_PERIOD_TICKS MA3_PERIOD_US*T2_TICKS_PER_US
#define MA3_ROLLOVER_THRESH  MA3_MAX_PULSE_TICKS -1000
#endif

void setup_ma3(void);

int ma3_pos(int chid);
int  ma3_period(int chid);
int  ma3_on(int chid);
int ma3_at_lower_bound(int chid);
int ma3_at_upper_bound(int chid);
int ma3_rollover(int chid);

#define MA3_BOUND_HYSTERISIS 0 //Keep from bouncing at limit

#if defined HB2_H2R1_J0J1 || defined HB2_H2R2_J0J1 || defined HB2_0_2_H2R3_J0J1 
#define NUM_MA3_CH 2
#endif

#if defined HB2_H2R1_J2J3J4 || defined HB2_H2R2_J2J3J4 || defined HB2_0_2_H2R3_J2J3J4
#define NUM_MA3_CH 3
#endif

#endif
#endif
