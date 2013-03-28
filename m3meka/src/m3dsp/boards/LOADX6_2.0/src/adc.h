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


#if 0
#if defined M3_LOADX6_A2R2  || defined M3_LOADX6_A2R3	
//#define ADC_USE_AVREF
#define ADC_NUM_SMOOTH	32 //Must be even
#define ADC_SHIFT_SMOOTH 5 //2^ADC_SHIFT_SMOOTH
#define ADC_NUM_CH 8
#define ADC_LOAD_0 1
#define ADC_LOAD_1 2
#define ADC_LOAD_2 3
#define ADC_LOAD_3 4
#define ADC_LOAD_4 5
#define ADC_LOAD_5 6
#endif

#define DMA_BUF_DEPTH 			6
#endif

//Can afford heavy smoothing
#define ADC_NUM_SMOOTH	32 //Must be even
#define ADC_SHIFT_SMOOTH 5 //2^ADC_SHIFT_SMOOTH
#define ADC_NUM_CH 9  // Number of channels enabled for channel scan
#define ADC_BUS_VOLTAGE 0
#define ADC_CURRENT_DIGITAL 1
#define ADC_EXT 2
#define  MAX_CHNUM	 			8		// Highest Analog input number in Channel Scan
#define  SAMP_BUFF_SIZE	 		8		// Size of the input buffer per analog input



void setup_adc(void);
unsigned int get_avg_adc(int idx);
extern unsigned int adc_raw[];

#endif
#endif
