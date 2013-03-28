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


#ifndef _TIMER3_H
#define _TIMER3_H

#ifdef USE_TIMER3


#if defined M3_WMA_0_1 || defined M3_BMA_A1R1  || defined M3_HMB_H1R1 || defined M3_HB2_H2R1_J0J1 \
	|| defined M3_HB2_H2R1_J2J3J4  || defined M3_HB2_H2R2_J0J1 || defined M3_HB2_H2R2_J2J3J4  \
	|| defined M3_GMB_G1R1 || defined M3_HEX2_S1R1 || defined M3_HB2_H2R3_J0J1
#define T3_US_PER_IRQ 500  //2Khz
#endif

#if defined M3_HB2_H2R3_J2J3J4
#define T3_US_PER_IRQ 500  //2Khz
#endif

#if defined M3_MAX2 || defined M3_BMW_A2R1 || defined M3_BMW_A2R2 || defined M3_HEX4_S2R1 || defined M3_BMW_A2R3
#define T3_US_PER_IRQ 500  //2Khz
#endif

#if defined M3_DEV || defined M3_DAC_0_1 || defined M3_ELMO_RNA_R0 || defined M3_ELMO_B1R1 || defined M3_ELMO_Z1R1
#define T3_US_PER_IRQ 500 //2Khz
#endif

#if defined M3_PWR_0_2 || defined M3_PWR_0_3 || defined M3_PWR_0_4 || defined M3_PWR_0_5 //100hz
#define T3_US_PER_IRQ 10000
#define T3_US_ROLLOVER 100000000
#endif

#if defined M3_LEDX2_S1R1 || defined M3_LEDX2XN_S2R1 //100Hz
#define T3_US_PER_IRQ 10000
#endif

#if defined M3_FB_DEV_0_0
#define T3_US_PER_IRQ 1000  //1Khz
#endif

/////////////////////////
//16bit timer value, scaled by 5, so 13107 (13ms) max period : 76Hz
#define T3_TCKPS 1 //8:1 scalar, so 1 tick = 200ns, 5 ticks/1us
#define T3_PR3 T3_US_PER_IRQ*5 //Ticks to interrupt at. Set period to get interrupt every 10000us
/////////////////////////

#if defined M3_LEDMDRV_S2R1
//16bit timer value, scaled by 5, so 13107 (13ms) max period : 76Hz
//#define T3_TCKPS 2 //64:1 scalar, so 1 tick = 1600ns, 625 ticks/1ms
//#define T3_TICKS_PER_MS 625
//#define T3_PR3 T3_MS_PER_IRQ*T3_TICKS_PER_MS //Ticks to interrupt at.

#define T3_TCKPS 1 //8:1 scalar, so 1 tick = 200ns, 5 ticks/1us
#define T3_US_PER_IRQ 1000 //1000Hz
#define T3_PR3 T3_US_PER_IRQ*5 //Ticks to interrupt at. 
#define LEDMDRV_IRQ_PER_TRIGGER 8 //100Hz
#endif


void setup_timer3(void);
#ifdef USE_TACTILE_PPS
extern int pps_trigger;
#define PPS_IRQ_PER_TRIGGER (int)28571/(int)T3_US_PER_IRQ //35hz
//#define PPS_IRQ_PER_TRIGGER (int)18000/(int)T3_US_PER_IRQ //55hz
#endif

#endif
#endif
