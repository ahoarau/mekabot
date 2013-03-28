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


#define ADC_NUM_SMOOTH			32	// Must be even
#define ADC_SHIFT_SMOOTH		5	// 2^ADC_SHIFT_SMOOTH
#define ADC_NUM_SMOOTH_FAST		2	// Must be even
#define ADC_SHIFT_SMOOTH_FAST	1	// 2^ADC_SHIFT_SMOOTH_FAST
#define ADC_NUM_CH				3
#define ADC_AMP_TEMP_A			0
#define ADC_MOTOR_TEMP			1
#define ADC_AMP_TEMP_B			2

#define DMA_BUF_DEPTH	ADC_NUM_CH

void setup_dma1(void);
void setup_adc(void);
unsigned int get_avg_adc(int idx);
extern unsigned int adc_raw[];

#endif
