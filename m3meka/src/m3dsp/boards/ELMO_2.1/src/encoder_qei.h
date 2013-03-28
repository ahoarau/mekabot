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


#ifndef _ENCODER_QEI_H
#define _ENCODER_QEI_H

#ifdef USE_ENCODER_QEI
//Setup for use with incremental quadarature encoder

void setup_qei(void);
int qei_error();
int  qei_position_HW();
int  qei_position_LW();
long qei_position();
int qei_calibrated_flag();
void qei_zero();
void step_qei();
extern long qei_pos;
extern long qei_roll;
#if defined M3_MAX2_BLDC_A2R3_QEI
#define QEI_PPR 1000 //Default counts per revolution 
#define QEI_MAXCNT (4*QEI_PPR-1) //Counts between index pulse (in 4x mode)
#endif

#if defined M3_ELMO_B1R1
#define QEI_PPR 2048 //2048 default counts per revolution (AMT102-V KIT, MAX RPM 7500, DIP = 1 1 1 1)
//#define QEI_PPR 500 //Temp
#define QEI_MAXCNT (4*QEI_PPR-1) //Counts between index pulse (in 4x mode)
//#define QEI_USE_INDEX 1 
#endif

#if defined M3_ELMO_Z1R1
#define QEI_PPR 2000 //Default counts per revolution 
#define QEI_MAXCNT (4*QEI_PPR-1) //Counts between index pulse (in 4x mode)
//#define QEI_32B_LIMIT = 32767-QEI_MAXCNT+1
//#define QEI_USE_INDEX 1 
#endif

#if defined M3_MAX2_BDC_ARMH || M3_MAX2_BDC_ARMH2
#define QEI_PPR 32 //Default counts per revolution 
#define QEI_MAXCNT (4*QEI_PPR-1) //Counts between index pulse (in 4x mode)
#define QEI_RAD_PER_PULSE 0.0490873852 //2.0*3.141592/(4.0*QEI_PPR)
#define QEI_DEG_PER_PULSE 2.8125 //360/(4.0*QEI_PPR)
#endif

#endif
#endif
