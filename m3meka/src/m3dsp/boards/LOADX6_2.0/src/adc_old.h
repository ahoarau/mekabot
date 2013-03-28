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



#if defined M3_DAC_0_1
#define ADC_NUM_SMOOTH	64 //Must be even
#define ADC_SHIFT_SMOOTH 6 //2^ADC_SHIFT_SMOOTH
#define ADC_NUM_SMOOTH_FAST	64 //Must be even
#define ADC_SHIFT_SMOOTH_FAST 6
 //2^ADC_SHIFT_SMOOTH_FAST
#define ADC_NUM_CH 3 
#define ADC_NC 0	//Not connected
#define ADC_MOTOR_TEMP 1
#define ADC_EXT 2
#endif


#if defined M3_ELMO_RNA_R0 
#define ADC_NUM_SMOOTH	32 //Must be even
#define ADC_SHIFT_SMOOTH 5 //2^ADC_SHIFT_SMOOTH
#define ADC_NUM_SMOOTH_FAST	16 //Must be even
#define ADC_SHIFT_SMOOTH_FAST 4 //2^ADC_SHIFT_SMOOTH_FAST
#define ADC_NUM_CH 3 
#define ADC_SEAS 0
#define ADC_MOTOR_TEMP 1
#define ADC_EXT 2
#endif

#if defined M3_ELMO_B1R1 || defined M3_ELMO_Z1R1
#define ADC_NUM_SMOOTH	32 //Must be even
#define ADC_SHIFT_SMOOTH 5 //2^ADC_SHIFT_SMOOTH
#define ADC_NUM_CH 2 
#define ADC_EXT 0
#define ADC_MOTOR_TEMP 1
#endif

#ifdef M3_BMA_A1R1
#define ADC_NUM_SMOOTH	64 //Must be even
#define ADC_SHIFT_SMOOTH 6 //2^ADC_SHIFT_SMOOTH
#define ADC_NUM_SMOOTH_FAST	16 //Must be even
#define ADC_SHIFT_SMOOTH_FAST 4 //2^ADC_SHIFT_SMOOTH_FAST
#define ADC_NUM_CH 6 
#define ADC_SEAS 0
#define ADC_MOTOR_TEMP 1
#define ADC_EXT 2
#define ADC_AMP_TEMP 3
#define ADC_CURRENT_A 4
#define ADC_CURRENT_B 5
#endif
  
#if defined M3_MAX2_BDC_A2R1 || defined M3_MAX2_BLDC_A2R1 || defined M3_MAX2_BDC_T2R1 || defined M3_MAX2_BLDC_T2R1
#define ADC_NUM_SMOOTH	32 //Must be even
#define ADC_SHIFT_SMOOTH 5 //2^ADC_SHIFT_SMOOTH
#define ADC_NUM_SMOOTH_FAST	16 //Must be even
#define ADC_SHIFT_SMOOTH_FAST 4 //2^ADC_SHIFT_SMOOTH_FAST
#define ADC_NUM_CH 5 
#define ADC_CURRENT_A 0
#define ADC_CURRENT_B 1
#define ADC_AMP_TEMP 2
#define ADC_MOTOR_TEMP 3
#define ADC_EXT 4
#endif

#if defined M3_MAX2_BDC_A2R2 || defined M3_MAX2_BLDC_A2R2 || defined  M3_MAX2_BDC_S1R1 || \
	defined  M3_MAX2_BDC_ARMH || defined M3_MAX2_BDC_A2R3 || defined M3_MAX2_BLDC_A2R3 
#define ADC_NUM_SMOOTH	32 //Must be even
#define ADC_SHIFT_SMOOTH 5 //2^ADC_SHIFT_SMOOTH
#define ADC_NUM_SMOOTH_FAST	16 //Must be even
#define ADC_SHIFT_SMOOTH_FAST 4 //2^ADC_SHIFT_SMOOTH_FAST
#define ADC_NUM_CH 4 
#define ADC_CURRENT_A 0
#define ADC_CURRENT_B 1
#define ADC_AMP_TEMP 2
#define ADC_MOTOR_TEMP 3
#endif

#if defined M3_MAX2_BLDC_T2R2 
#define ADC_NUM_SMOOTH	32 //Must be even
#define ADC_SHIFT_SMOOTH 5 //2^ADC_SHIFT_SMOOTH
#define ADC_NUM_SMOOTH_FAST	16 //Must be even
#define ADC_SHIFT_SMOOTH_FAST 4 //2^ADC_SHIFT_SMOOTH_FAST
#define ADC_NUM_CH 4 
#define ADC_CURRENT_A 0
#define ADC_CURRENT_B 1
#define ADC_AMP_TEMP 2
#define ADC_EXT 3			//SEAX2-1.2 is configured to take load-cell on the motor temp connector
//NOTE: Not using motor_temp on A2R2 due to excessive noise when sample 7 channels
//#define ADC_MOTOR_TEMP 6	//SEAX2-1.2 is configured to take case-temp sensor on SEAS SPI Encoder input

//SEAX-1.2 Configuration:
//RJ7 present, RJ6 DNS gives A5V to load cell
//RJ2 present, RJ3 DNS converts load cell 5V signal to 3V3
//SPI_CLK_SEAS used for chassis temp sensor (AN6)
#endif

#if  defined M3_MAX2_BDC_T2R2
#define ADC_NUM_SMOOTH	32 //Must be even
#define ADC_SHIFT_SMOOTH 5 //2^ADC_SHIFT_SMOOTH
#define ADC_NUM_SMOOTH_FAST	32 //Must be even
#define ADC_SHIFT_SMOOTH_FAST 5 //2^ADC_SHIFT_SMOOTH_FAST
#define ADC_NUM_CH 4 
#define ADC_CURRENT_A 0
#define ADC_CURRENT_B 1
#define ADC_AMP_TEMP 2
#define ADC_EXT 3			//SEAX2-1.2 is configured to take load-cell on the motor temp connector
//NOTE: Not using motor_temp on A2R2 due to excessive noise when sample 7 channels
//#define ADC_MOTOR_TEMP 6	//SEAX2-1.2 is configured to take case-temp sensor on SEAS SPI Encoder input

//SEAX-1.2 Configuration:
//RJ7 present, RJ6 DNS gives A5V to load cell
//RJ2 present, RJ3 DNS converts load cell 5V signal to 3V3
//SPI_CLK_SEAS used for chassis temp sensor (AN6)
#endif

#if defined M3_MAX2_BLDC_T2R3
#define ADC_NUM_SMOOTH	32 //Must be even
#define ADC_SHIFT_SMOOTH 5 //2^ADC_SHIFT_SMOOTH
#define ADC_NUM_SMOOTH_FAST	16 //Must be even
#define ADC_SHIFT_SMOOTH_FAST 4 //2^ADC_SHIFT_SMOOTH_FAST
#define ADC_NUM_CH 4 
#define ADC_CURRENT_A 0
#define ADC_CURRENT_B 1
#define ADC_AMP_TEMP 2
#define ADC_MOTOR_TEMP 3
#endif

#if defined M3_MAX2_BDC_T2R3 //EXT_TEMP is used by brake.
#define ADC_NUM_SMOOTH	32 //Must be even
#define ADC_SHIFT_SMOOTH 5 //2^ADC_SHIFT_SMOOTH
#define ADC_NUM_SMOOTH_FAST	16 //Must be even
#define ADC_SHIFT_SMOOTH_FAST 4 //2^ADC_SHIFT_SMOOTH_FAST
#define ADC_NUM_CH 3 
#define ADC_CURRENT_A 0
#define ADC_CURRENT_B 1
#define ADC_AMP_TEMP 2
#endif

#if defined M3_BMW_A2R1
#define ADC_NUM_SMOOTH	32 //Must be even
#define ADC_SHIFT_SMOOTH 5 //2^ADC_SHIFT_SMOOTH
#define ADC_NUM_SMOOTH_FAST	2 //Must be even
#define ADC_SHIFT_SMOOTH_FAST 1 //2^ADC_SHIFT_SMOOTH_FAST
#define ADC_NUM_CH 5 
#define ADC_CURRENT_A 0
#define ADC_CURRENT_B 1
#define ADC_MOTOR_TEMP 2
#define ADC_AMP_TEMP 3
#define ADC_EXT 4
#endif

#if defined M3_BMW_A2R2 || defined M3_BMW_A2R3
#define ADC_NUM_SMOOTH	32 //Must be even
#define ADC_SHIFT_SMOOTH 5 //2^ADC_SHIFT_SMOOTH
#define ADC_NUM_SMOOTH_FAST	2 //Must be even
#define ADC_SHIFT_SMOOTH_FAST 1 //2^ADC_SHIFT_SMOOTH_FAST
#define ADC_NUM_CH 2 
#define ADC_CURRENT_A 0
#define ADC_EXT_TEMP 1
#endif

#if defined M3_HEX4_S2R1
#define ADC_NUM_SMOOTH	32 //Must be even
#define ADC_SHIFT_SMOOTH 5 //2^ADC_SHIFT_SMOOTH
#define ADC_NUM_SMOOTH_FAST	2 //Must be even
#define ADC_SHIFT_SMOOTH_FAST 1 //2^ADC_SHIFT_SMOOTH_FAST
#define ADC_NUM_CH 3 
#define ADC_AMP_TEMP_A 0
#define ADC_MOTOR_TEMP 1
#define ADC_AMP_TEMP_B 2
#endif

#ifdef M3_WMA_0_1
#define ADC_NUM_SMOOTH	64 //Must be even
#define ADC_SHIFT_SMOOTH 6 //2^ADC_SHIFT_SMOOTH
#define ADC_NUM_SMOOTH_FAST	16 //Must be even
#define ADC_SHIFT_SMOOTH_FAST 4 //2^ADC_SHIFT_SMOOTH_FAST
#define ADC_NUM_CH 6 
#define ADC_SEAS 0
#define ADC_MOTOR_TEMP 1
#define ADC_EXT 2
#define ADC_AMP_TEMP 3
#define ADC_CURRENT_A 4
#define ADC_CURRENT_B 5
#endif

#if defined M3_PWR_0_2 || defined M3_PWR_0_3 || defined M3_PWR_0_4 || defined M3_PWR_0_5
//Can afford heavy smoothing
#define ADC_NUM_SMOOTH	32 //Must be even
#define ADC_SHIFT_SMOOTH 5 //2^ADC_SHIFT_SMOOTH
#define ADC_NUM_CH 3 
#define ADC_BUS_VOLTAGE 0
#define ADC_CURRENT_DIGITAL 1
#define ADC_EXT 2
#endif



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

#if defined M3_LOADX6_A2R1 
//#define ADC_USE_AVREF
#define ADC_NUM_SMOOTH	32 //Must be even
#define ADC_SHIFT_SMOOTH 5 //2^ADC_SHIFT_SMOOTH
#define ADC_NUM_CH 9
#define ADC_EXT_0 0
#define ADC_EXT_1 1
#define ADC_EXT_2 2
#define ADC_LOAD_0 3
#define ADC_LOAD_1 4
#define ADC_LOAD_2 5
#define ADC_LOAD_3 6
#define ADC_LOAD_4 7
#define ADC_LOAD_5 8
#endif


#ifdef M3_HMB_H1R1
#define ADC_NUM_SMOOTH	16 //Must be even
#define ADC_SHIFT_SMOOTH 4 //2^ADC_SHIFT_SMOOTH
#define ADC_NUM_CH 9 
#define ADC_SEAS_A 0
#define ADC_SEAS_B 1
#define ADC_CURRENT_A 2
#define ADC_CURRENT_B 3
#define ADC_MOTOR_TEMP_A 4
#define ADC_MOTOR_TEMP_B 5
#define ADC_EXT_A 6
#define ADC_EXT_B 7
#define ADC_AMP_TEMP 8
#endif


#if defined M3_HB2_H2R1_J0J1 
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

#if defined M3_HB2_H2R2_J0J1  || defined M3_HB2_H2R3_J0J1 
#define ADC_NUM_SMOOTH	8 //Must be even
#define ADC_SHIFT_SMOOTH 3 //2^ADC_SHIFT_SMOOTH
#define ADC_NUM_CH 6 
#define ADC_SEAS_B 0
#define ADC_CURRENT_B 1
#define ADC_SEAS_A 2
#define ADC_CURRENT_A 3
#define ADC_AMP_TEMP_B 4
#define ADC_AMP_TEMP_A 5
#endif

#if defined M3_HB2_H2R1_J2J3J4
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

#if defined M3_HB2_H2R2_J2J3J4 || defined M3_HB2_H2R3_J2J3J4
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
#endif

#ifdef M3_HEX2_S1R1
#define ADC_NUM_SMOOTH	16 //Must be even
#define ADC_SHIFT_SMOOTH 4 //2^ADC_SHIFT_SMOOTH
#define ADC_NUM_CH 9 
#define ADC_EXT_A 0
#define ADC_EXT_B 1
#define ADC_CURRENT_A 2
#define ADC_CURRENT_B 3
#define ADC_MOTOR_TEMP_A 4
#define ADC_MOTOR_TEMP_B 5
#define ADC_NOTUSED_A 6
#define ADC_NOTUSED_B 7
#define ADC_AMP_TEMP 8
#endif

#ifdef M3_GMB_G1R1
#define ADC_NUM_SMOOTH	16 //Must be even
#define ADC_SHIFT_SMOOTH 4 //2^ADC_SHIFT_SMOOTH
#define ADC_NUM_CH 9
#define ADC_SEAS_A 1
#define ADC_SEAS_B 0
#define ADC_AMP_TEMP 2
#define ADC_MOTOR_TEMP_A 4
#define ADC_MOTOR_TEMP_B 3
#define ADC_EXT_A 5
#define ADC_EXT_B 6
#define ADC_EXT_C 7
#define ADC_EXT_D 8
#endif

#if defined M3_LEDX2_S1R1 || defined M3_LEDX2XN_S2R1
#define ADC_NUM_SMOOTH	16 //Must be even
#define ADC_SHIFT_SMOOTH 4 //2^ADC_SHIFT_SMOOTH
#define ADC_NUM_CH 4
#define ADC_EXT_A 0
#define ADC_EXT_B 1
#define ADC_EXT_C 2
#define ADC_EXT_D 3
#endif

void setup_adc(void);
unsigned int get_avg_adc(int idx);
extern unsigned int adc_raw[];

#if defined M3_DAC_0_1 || defined M3_BMA_A1R1 || defined M3_WMA_0_1   || defined M3_ELMO_RNA_R0 || defined M3_ELMO_B1R1 || defined M3_ELMO_Z1R1
unsigned int get_avg_adc_torque();
#endif

#endif
#endif
