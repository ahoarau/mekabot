/* 
M3 -- Meka Robotics Real-Time Control System
Copyright (c) 2011 Meka Robotics
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

#ifndef __CURRENT_H__
#define __CURRENT_H__ 

#ifdef USE_CURRENT

void setup_current();
void step_current();
int get_current_state();
int get_current_ma();
long get_current_rms_mom_sq_ma();
long get_current_rms_cont_sq_ma();
int current_fault_mom_flag();
int current_fault_cont_flag();
void reset_current_buf();
unsigned int correct_mA(unsigned int pwm, unsigned int max_pwm, unsigned int current);

enum 
{
  	CURRENT_STARTUP,
  	CURRENT_READY,
  	CURRENT_FAULT_MOM,
  	CURRENT_FAULT_CONT,
	CURRENT_HOLD
};

#ifdef BMW_0_5_A2R4
// ACS711 w/ amp (G = -1.65): 4.44mA/bit
// current(mA) = (ADC*I_GAIN) >> I_SHIFT; //equivalent to 4.5mA/bit
#define I_GAIN 					9
#define I_SHIFT					1
//Valid auto-zero (1.65V*1.65 ±10%)
#define AMP_MIN		2450
#define AMP_MAX		2995
#endif //BMW_0_5_A2R4

#ifdef BMW_0_6_A2R4
// ACS711 w/ amp (G = -5): 1.465mA/bit
// current(mA) = (ADC*I_GAIN) >> I_SHIFT; //equivalent to 1.5mA/bit
#define I_GAIN 					3
#define I_SHIFT					1
//Valid auto-zero (1.65V ±10%)
#define AMP_MIN		1843
#define AMP_MAX		2253
#endif //BMW_0_6_A2R4

//Current limits
#define MAX_MOM_CURRENT			5000		//5A
#define MAX_CONT_CURRENT 		2000		//2A
#define CURRENT_MAX_MOM_RMS_SQ  25000000	//MAX_MOM_CURRENT*MAX_MOM_CURRENT
#define CURRENT_MAX_CONT_RMS_SQ 4000000		//MAX_CONT_CURRENT*MAX_CONT_CURRENT

#endif
#endif
