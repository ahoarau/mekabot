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

#ifndef __ADC_H__
#define __ADC_H__ 
#include "setup.h"

#ifdef USE_ADC

#if defined HB2_H2R1_J0J1 
#define ADC_NUM_SMOOTH	8 //Must be even
#define ADC_SHIFT_SMOOTH 3 //2^ADC_SHIFT_SMOOTH
#define ADC_NUM_CH 8 
#define ADC_SEAS_A 2
#define ADC_MOTOR_TEMP_A 3
#define ADC_AMP_TEMP_A 7
#define ADC_SEAS_B 0
#define ADC_MOTOR_TEMP_B 1
#define ADC_AMP_TEMP_B 6
#endif

#if defined HB2_H2R2_J0J1  || defined HB2_0_2_H2R3_J0J1 
#define ADC_NUM_SMOOTH	8 //Must be even
#define ADC_SHIFT_SMOOTH 3 //2^ADC_SHIFT_SMOOTH
#define ADC_NUM_CH 6 
#define ADC_SEAS_B 0
#define ADC_CURRENT_B 1
#define ADC_SEAS_A 2
#define ADC_CURRENT_A 3
#define ADC_AMP_TEMP_B 4
#define ADC_AMP_TEMP_A 5
#define  MAX_CHNUM	 			5		// Highest Analog input number in Channel Scan
#define  SAMP_BUFF_SIZE	 		8		// Size of the input buffer per analog input
#endif

#if defined HB2_H2R1_J2J3J4
#define ADC_NUM_SMOOTH	8 //Must be even
#define ADC_SHIFT_SMOOTH 3 //2^ADC_SHIFT_SMOOTH
#define ADC_NUM_CH 8 
#define ADC_SEAS_A 0
#define ADC_MOTOR_TEMP_A 1
#define ADC_AMP_TEMP_A 6
#define ADC_SEAS_B 2
#define ADC_MOTOR_TEMP_B 3
#define ADC_AMP_TEMP_B 7
#define ADC_SEAS_C 4
#define ADC_MOTOR_TEMP_C 5
#define ADC_AMP_TEMP_C 7 //shared with AMP_TEMP_B
#endif

#if defined HB2_H2R2_J2J3J4 || defined HB2_0_2_H2R3_J2J3J4
#define ADC_NUM_SMOOTH	8 //Must be even
#define ADC_SHIFT_SMOOTH 3 //2^ADC_SHIFT_SMOOTH
#define ADC_NUM_CH 7 
#define ADC_SEAS_A 0
#define ADC_CURRENT_A 1
#define ADC_SEAS_B 2
#define ADC_CURRENT_B 3
#define ADC_SEAS_C 4
#define ADC_CURRENT_C 5
#define ADC_AMP_TEMP_A 6 //shared among all
#define ADC_AMP_TEMP_B 6
#define ADC_AMP_TEMP_C 6
#define  MAX_CHNUM	 			6		// Highest Analog input number in Channel Scan
#define  SAMP_BUFF_SIZE	 		8		// Size of the input buffer per analog input
#endif

void setup_adc(void);
unsigned int get_avg_adc(int idx);
extern unsigned int adc_raw[];

#endif
#endif
