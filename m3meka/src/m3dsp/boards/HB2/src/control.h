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

#ifndef __CONTROL_H__
#define __CONTROL_H__ 

#ifdef USE_CONTROL

void setup_control();
void step_control();



#if defined HB2_H2R2_J0J1 || \
	defined HB2_H2R2_J2J3J4 || defined HB2_H2R1_J0J1 || defined HB2_H2R1_J2J3J4 || defined HB2_0_2_H2R3_J0J1 || defined HB2_0_2_H2R3_J2J3J4

//Modes
#define MODE_OFF 0
#define MODE_PWM 1
#define MODE_PID 2
//FSA states
#define CTRL_OFF 0
#define CTRL_PWM 1
#define CTRL_OFF_TO_PID 2
#define CTRL_PID_TO_OFF 3
#define CTRL_PID 4
#define	CTRL_ABORT 5

#define GAIN_LIMIT 32000			//Safety check on signed 16 bit gains
#define RAMP_UPDATE_RATE 250		//Slew rate for gains
//#define NUM_TORQUE_DOT_SAMPLES 128	//Buffering of error derivative
#define NUM_TORQUE_DOT_SAMPLES 32	//Buffering of error derivative
extern long p_term[];
extern long i_term[];
extern long d_term[];
extern long ff_term[];


#if defined HB2_H2R1_J0J1 || defined HB2_H2R2_J0J1 || defined HB2_0_2_H2R3_J0J1 
#define NUM_CTRL_CH 2
#endif

#if defined HB2_H2R1_J2J3J4 || defined HB2_H2R2_J2J3J4 || defined HB2_0_2_H2R3_J2J3J4
#define NUM_CTRL_CH 3
#endif

#endif

#endif

#endif
