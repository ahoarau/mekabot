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

#ifndef M3EC_PDO_V4_H
#define M3EC_PDO_V4_H

#ifdef __KERNEL__
#define int16_t short
#define int32_t int
#else
#ifndef EMBEDDED
#include <sys/types.h>
#include <stdint.h>
#endif
#endif

///////////////////////////////  From M3ACT_PDO_V4 /////////////////////////////////////////////////////

//V4 is for IQ version of actuator (current commanded)

#define ACTUATOR_EC_CONFIG_HAS_BRAKE		4 //Set this bit if there is a brake on the actuator e.g. torso J1

typedef struct 
{
	int16_t		config;					//Reserved
	int16_t		k_p;					//P gain, torque control
	int16_t		k_i;					//I gain, torque control
	int16_t		k_d;					//D gain, torque control
	int16_t		k_p_shift;				//Shift scalar, torque control
	int16_t		k_i_shift;				//Shift scalar, torque control
	int16_t		k_d_shift;				//Shift scalar, torque control
	int16_t		k_i_limit;				//Integral limit, torque control
	int16_t		t_desire;				//Desired joint torque, raw adc ticks
	int16_t		mode;					//Mode (Off=0, Pwm=1, PID=2)
	int16_t		qei_max;				//Max permissible qei
	int16_t		qei_min;				//Min permissible qei
	int16_t		pwm_max;				//Max permissible pwm
        int16_t         pwm_desired;
        int16_t         current_desired;
        int16_t         bldc_mode;                              // 1 for bldc, 0 for bdc
} M3ActPdoV4Cmd;

#define M3ACT_FLAG_POS_LIMITSWITCH 1
#define M3ACT_FLAG_NEG_LIMITSWITCH 2
#define M3ACT_FLAG_QEI_CALIBRATED 4
#define M3ACT_FLAG_AUX_SWITCH 8
#define M3ACT_FLAG_I_FAULT_CONT 16
#define M3ACT_FLAG_I_FAULT_MOM 32

typedef struct 
{
	int16_t		current_ma;		//Calibrated value
	int16_t		debug;			//Reserved [vertx err cnts]
	int16_t		adc_torque;			//Torque input 
	int16_t		torque_err_cnt;		//Err
	int16_t		adc_ext_temp;		//Motor temp
	int16_t		adc_amp_temp;		//Amplifier temp 
	int16_t		adc_current_a;		//Motor current leg A 
	int16_t		adc_current_b;		//Motor current leg B 
	int16_t		pwm_cmd;		//PWM command to motor
	int16_t		qei_on;			//Encoder ticks
	int16_t		qei_period;		//Encoder ticks
	int16_t		qei_rollover;		//Encoder rollover counts (directional) -1|0|1 ...
	int16_t		qei_err_cnt;		//Err
	int16_t		flags;			//Reserved
        int16_t         motor_pos;              // calculated from the hall effect sensors
}M3ActPdoV4Status;

///////////////////////////////  M3ACT_PDO_V4 Multi-Channel /////////////////////////////////////////////////////

typedef struct
{
	M3ActPdoV4Cmd command[1]; 
} M3ActX1PdoV4Cmd;

typedef struct
{
	M3ActPdoV4Cmd command[2];
} M3ActX2PdoV4Cmd;

typedef struct
{
	M3ActPdoV4Cmd command[3];
} M3ActX3PdoV4Cmd;

typedef struct
{
	M3ActPdoV4Cmd command[4];
} M3ActX4PdoV4Cmd;

typedef struct
{
	uint64_t    timestamp;		    //Time in us
	M3ActPdoV4Status 	status[1];
} M3ActX1PdoV4Status;

typedef struct
{
	uint64_t    timestamp;		    //Time in us
	M3ActPdoV4Status 	status[2];
} M3ActX2PdoV4Status;

typedef struct
{
	uint64_t    timestamp;		    //Time in us
	M3ActPdoV4Status 	status[3];
} M3ActX3PdoV4Status;



#endif
