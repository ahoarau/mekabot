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


#ifndef M3EC_PDO_V2_H
#define M3EC_PDO_V2_H

#ifdef __KERNEL__
#define int16_t short
#define int32_t int
#else
#ifndef EMBEDDED
#include <sys/types.h>
#include <stdint.h>
#endif
#endif

///////////////////////////////  M3ACT_PDO_V2 /////////////////////////////////////////////////////
#define M3PWR_CONFIG_BUZZER 1 	   //BUZZER

///////////////////////////////  PWR //////////////////////////////////////////////////////////

typedef struct 
{
	int16_t		config;					//Reserved
	int16_t		enable_motor;				//Software enable of motor bus voltage (RP12)
}M3PwrPdoV2Cmd;

typedef struct 
{
	uint64_t		timestamp;			//Time in us
	int16_t			motor_enabled;			//State of the motor bus relay (RP6)
	int16_t			adc_bus_voltage;		//Voltage input (AN0)
	int16_t			adc_current_digital;		//Digital logic power consumption (AN1)
	int16_t			adc_ext;			//Auxillary adc input (AN2)
	int16_t			flags;				//Reserved
}M3PwrPdoV2Status;

///////////////////////////////  ACT //////////////////////////////////////////////////////////

#define M3ACT_PDO_V2_CMD_EXT_SZ 16
typedef struct 
{
	int16_t		config;					//Reserved
	int16_t		t_desire;				//Desired joint torque, raw adc ticks
	int16_t		mode;					//Mode (Off=0, Pwm=1, PID=2)
	int16_t		ext_start_idx;
	unsigned char	ext[M3ACT_PDO_V2_CMD_EXT_SZ];
} M3ActPdoV2Cmd;

typedef struct 
{
	int16_t		k_p;					//P gain, torque control
	int16_t		k_i;					//I gain, torque control
	int16_t		k_d;					//D gain, torque control
	int16_t		k_p_shift;				//Shift scalar, torque control
	int16_t		k_i_shift;				//Shift scalar, torque control
	int16_t		k_d_shift;				//Shift scalar, torque control
	int16_t		k_i_limit;				//Integral limit, torque control
	int16_t		t_max;					//Max permissible torque
	int16_t		t_min;					//Min permissible torque
	int16_t		pwm_max;				//Max permissible pwm 
	int16_t		qei_max;				//Max permissible qei
	int16_t		qei_min;				//Min permissible qei
	int16_t		k_ff_zero;				//Deprecated (Zero point for ff term, torque control)
	int16_t		k_ff_shift;				//Deprecated (Shift scalar, torque control)
	int16_t		k_ff;					//Feedforward gain, torque control
	int16_t		pwm_db;					//Pwm deadband
} M3ActPdoV2CmdExt;


#define M3ACT_PDO_V2_STATUS_EXT_SZ 8
typedef struct 
{
	int16_t		adc_torque;			//Torque input 
	int16_t		pwm_cmd;			//Current PWM command to motor
	int16_t		qei_on;				//Encoder ticks
	int16_t		qei_period;			//Encoder ticks
	int16_t		qei_rollover;		//Encoder rollover counts (directional) -1|0|1 ...
	int16_t		flags;				//Reserved
	int16_t		ext_start_idx;
	unsigned char	ext[M3ACT_PDO_V2_STATUS_EXT_SZ];
}M3ActPdoV2Status;

typedef struct 
{
	int16_t		debug;				//Reserved
	int16_t		adc_motor_temp;		//Motor temp 
	int16_t		adc_ext_a;			//Auxillary adc input 
	int16_t		adc_ext_b;			//Auxillary adc input
	int16_t		adc_amp_temp;		//Amplifier temp 
	int16_t		adc_current_a;		//Motor current leg A 
	int16_t		adc_current_b;		//Motor current leg B 
}M3ActPdoV2StatusExt;

typedef struct
{
	M3ActPdoV2Cmd command[1]; 
} M3ActX1PdoV2Cmd;

typedef struct
{
	M3ActPdoV2Cmd command[2];
} M3ActX2PdoV2Cmd;

typedef struct
{
	M3ActPdoV2Cmd command[3];
} M3ActX3PdoV2Cmd;

typedef struct
{
	M3ActPdoV2Cmd command[4];
} M3ActX4PdoV2Cmd;

typedef struct
{
	uint64_t    timestamp;		    //Time in us
	M3ActPdoV2Status 	status[1];
} M3ActX1PdoV2Status;

typedef struct
{
	uint64_t    timestamp;		    //Time in us
	M3ActPdoV2Status 	status[2];
} M3ActX2PdoV2Status;

typedef struct
{
	uint64_t    timestamp;		    //Time in us
	M3ActPdoV2Status 	status[3];
} M3ActX3PdoV2Status;

typedef struct
{
	uint64_t    timestamp;		    //Time in us
	M3ActPdoV2Status 	status[4];
} M3ActX4PdoV2Status;

#endif
