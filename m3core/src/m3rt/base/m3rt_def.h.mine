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

#ifndef M3RT_DEF_H
#define M3RT_DEF_H

//#include "math.h"

//Precision
//typedef float mReal;
typedef double mReal;

/* ------ Priorities for RT thread Step ordering ---------------------*/
#define MAX_PRIORITY 8
#define EC_PRIORITY 0
#define CALIB_PRIORITY 1
#define CONTROL_PRIORITY 2
#define JOINT_PRIORITY 3
#define CHAIN_PRIORITY 4
#define DYNAMATICS_PRIORITY 5
#define ROBOT_PRIORITY 6
#define ROBOT_CTRL_PRIORITY 7
#define ARM_HEAD_DYNAMATICS_PRIORITY 8

/*-------------------- RTAI AND ETHERCAT ----------------------------*/

#define LOG_FILE "/robot_log/m3rt_server.log"
// RTAI task characteristics
#define RT_TASK_FREQUENCY 1000		//Frequency of rt-task (HZ) (1000)
#define RT_KMOD_FREQUENCY 1000		//Frequency of rt kernel module (HZ) (3000)
#define RT_INHIBIT_TIME 20
#define RT_TIMER_TICKS_NS (1000000000 / RT_TASK_FREQUENCY)		//Period of rt-timer (ns) (=500us)	
#define RT_KMOD_TIMER_TICKS_NS (1000000000 / RT_KMOD_FREQUENCY)		//Period of kernel module rt-timer (ns) (=500us)				
#define RT_STACK_SIZE 10000
#define RT_STATUS_FREQUENCY 0 //RT_TASK_FREQUENCY/10
#define RT_TASK_PRIORITY 5

// RTAI names. Must be <=6 chars
#define SHMNAM_M3MKMD  "M3EC"	
#define SEMNAM_M3LEXT  "M3EX"
#define SEMNAM_M3LSHM  "M3SH"
#define SEMNAM_M3SYNC  "M3SN"

/*-------------------- ENVIRONMENT AND CONFIG ----------------------------*/
#define M3_ROBOT_ENV_VAR "M3_ROBOT"
#define M3_CONFIG_FILENAME    "m3_config.yml"
//#define M3_COMP_LIB_FILENAME    "m3_component_libs.yml"
#define M3_COMP_LIB_FILENAME  "m3_config.yml"
/*-------------------- CONVERSION MACROS ----------------------------*/
#define GRAV -9.8
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define RT_TASK_PERIOD_MS 1000.0/(mReal)RT_TASK_FREQUENCY
#define RT_TASK_PERIOD_US 1000000.0*RT_TASK_PERIOD_MS

#define F2C		(mReal)((((mReal)a)-32.0)*(5.0/9.0))
#define C2F(a)  	(mReal)((9.0/5.0)*((mReal)a)+32.0)
#define DEG2RAD(a)	(mReal)2.0*M_PI*((mReal)a)/360.0
#define RAD2DEG(a)	(mReal)360.0*((mReal)a)/(2.0*M_PI)
#define MS2CYC(a) 	(int)((a)/RT_TASK_PERIOD_MS)
#define US2CYC(a) 	(int)((a)/RT_TASK_PERIOD_US)
#define MNM2INLB(a)	(mReal((a))*.00885666)
#define INLB2MNM(a)	(mReal((a))/.00885666)
#undef MAX
#undef MIN
#define MAX(a, b)  (((a) > (b)) ? (a) : (b))
#define MIN(a, b)  (((a) < (b)) ? (a) : (b))
#define ABS(a)	   (((a) < 0) ? -(a) : (a))
#define SIGN(a)	   (((a) < 0) ? -1 : 1)
#define INC_MOD(a, b)  (((a+1) >= (b)) ? 0 : (a+1))
#define CLAMP(x, low, high)  (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))

#define ZERO(a) memset(&a,0,sizeof(a)) 



#endif

