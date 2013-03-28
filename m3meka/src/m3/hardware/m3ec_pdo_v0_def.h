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


#ifndef M3EC_PDO_V0_H
#define M3EC_PDO_V0_H

#ifdef __KERNEL__
#define int16_t short
#define int32_t int
#else
#ifndef EMBEDDED
#include <sys/types.h>
#include <stdint.h>
#endif
#endif

///////////////////////////////  M3SEA /////////////////////////////////////////////////////

#define M3SEA_CONFIG_TORQUE_SMOOTH 2
#define M3SEA_CONFIG_TORQUE_MASK_A 4
#define M3SEA_CONFIG_TORQUE_MASK_B 8
#define M3SEA_CONFIG_MA3_BOUNDS 16 //Limit PWM within QEI range
#define M3SEA_CONFIG_PWM_FWD_SIGN 32 //Does +PWM => +QEI change
//Size: 13*2=26 bytes [N=2]
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
	int16_t		t_max;					//Max permissible torque
	int16_t		t_min;					//Min permissible torque
	int16_t		mode;					//Mode (Off=0, Pwm=1, PID=2)
	int16_t		pwm_max;				//Max permissible pwm 
} M3SeaPdoV0Cmd;
//Size: 8*2+4*3+10*2=48 bytes  [N=2]
typedef struct 
{
	int32_t         timestamp;			//Time in us (uint64_t for DC clocks)
	int32_t			p_term;				//PID term
	int32_t			i_term;				//PID term
	int32_t			d_term;				//PID term
	int16_t			debug;				//Reserved
	int16_t			adc_torque;			//Torque input (AN0)
	int16_t			adc_motor_temp;		//Motor temp (AN1)
	int16_t			adc_ext;			//Auxillary adc input (AN2)
	int16_t			adc_amp_temp;		//Amplifier temp (AN3)
	int16_t			adc_current_a;		//Motor current leg A (AN4)
	int16_t			adc_current_b;		//Motor current leg B (AN5)
	int16_t			pwm_cmd;			//Current PWM command to motor
	int16_t			qei_on;				//Encoder ticks
	int16_t			qei_period;			//Encoder ticks
	int16_t			flags;				//Reserved
}M3SeaPdoV0Status;
 
///////////////////////////////  M3Gripper /////////////////////////////////////////////////////
//Size: 13*2=26 bytes  [N=2]
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
	int16_t		t_max;					//Max permissible torque
	int16_t		t_min;					//Min permissible torque
	int16_t		mode;					//Mode (Off=0, Pwm=1, PID=2)
	int16_t		pwm_max;				//Max permissible pwm 
} M3GmbPdoV0Cmd;

//Size: 26*2=52 bytes
typedef struct
{
	M3GmbPdoV0Cmd command[2];
} M3GmbX2PdoV0Cmd;


//Size: 2*10= 20 bytes
typedef struct 
{
	int16_t			debug;				//Reserved
	int16_t			adc_torque;			//Torque input 
	int16_t			adc_motor_temp;		//Motor temp 
	int16_t			adc_amp_temp;		//Amplifier temp 
	int16_t			adc_ext_a;			//Auxillary adc input 
	int16_t			adc_ext_b;			//Auxillary adc input 
	int16_t			pwm_cmd;			//Current PWM command to motor
	int16_t			qei_on;				//Encoder ticks
	int16_t			qei_period;			//Encoder ticks
	int16_t			flags;				//Reserved
}M3GmbPdoV0Status;

//22 Taxel PRT Pressure Profile Systems Sensor	
//Size: 2*22=44 bytes
typedef struct 
{
	uint16_t			taxel[22];			//PPS 22 taxel array
}M3TactilePPS22PdoV0Status;

	
//Size: 4*1+20*2+44*2 =132 bytes (5 30byte entries)  [N=5]
typedef struct
{
	int32_t		timestamp;			//Time in us
	M3GmbPdoV0Status 	status[2];
	M3TactilePPS22PdoV0Status tactile[2];
} M3GmbX2PdoV0Status;

///////////////////////////////  M3DIGITX2 /////////////////////////////////////////////////////
//Reduced form of M3SEA
//Size: 2*15=30 bytes  
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
	int16_t		t_max;					//Max permissible torque
	int16_t		t_min;					//Min permissible torque
	int16_t		mode;					//Mode (Off=0, Pwm=1, PID=2)
	int16_t		pwm_max;				//Max permissible pwm 
	int16_t		qei_max;				//Max permissible qei
	int16_t		qei_min;				//Min permissible qei
} M3DigitPdoV0Cmd;
//Size: 2*30=60 bytes  [N=2]
typedef struct
{
	M3DigitPdoV0Cmd command[2];
} M3DigitX2PdoV0Cmd;
//Size: 2*11=22 bytes
typedef struct 
{
	int16_t			debug;			//Reserved
	int16_t			adc_torque;		//Torque input 
	int16_t			adc_motor_temp;		//Motor temp 
	int16_t			adc_ext;		//Auxillary adc input 
	int16_t			adc_amp_temp;		//Amplifier temp 
	int16_t			adc_current;		//Motor current 
	int16_t			pwm_cmd;		//Current PWM command to motor
	int16_t			qei_on;			//Encoder ticks
	int16_t			qei_period;		//Encoder ticks
	int16_t			qei_rollover;	//Encoder rollover counts (directional) -1|0|1 ...
	int16_t			flags;			//Reserved
}M3DigitPdoV0Status;
//Size: 4*1+2*22=48 bytes  [N=2]
typedef struct
{
	int32_t			timestamp;			//Time in us
	M3DigitPdoV0Status 	status[2];
} M3DigitX2PdoV0Status;

///////////////////////////////  PWR //////////////////////////////////////////////////////////
//Size: 2*2=4 bytes  [N=2]
typedef struct 
{
	int16_t		config;					//Reserved
	int16_t		enable_motor;				//Software enable of motor bus voltage (RP12)
}M3PwrPdoV0Cmd;
//Size: 4*1+2*6=16 bytes  [N=2]
typedef struct 
{
	int32_t			timestamp;			//Time in us
	int16_t			mode_remote;			//Mode switch on remote (RP5)
	int16_t			motor_enabled;			//State of the motor bus relay (RP6)
	int16_t			adc_bus_voltage;		//Voltage input (AN0)
	int16_t			adc_current_digital;		//Digital logic power consumption (AN1)
	int16_t			adc_ext;			//Auxillary adc input (AN2)
	int16_t			flags;				//Reserved
}M3PwrPdoV0Status;

///////////////////////////////  LOADX6 //////////////////////////////////////////////////////
//Size: 2*1=2 bytes  [N=2]
typedef struct 
{
	int16_t		config;					//Reserved
}M3LoadX6PdoV0Cmd;
//Size: 2*4+2*11=30 bytes  [N=2]
typedef struct 
{
	int32_t			timestamp;			//Time in us
	int16_t			dig_ext_0;			//Auxillary digital input (RP5)
	int16_t			adc_ext_0;			//Auxillary adc input (AN0)
	int16_t			adc_ext_1;			//Auxillary adc input (AN1)
	int16_t			adc_ext_2;			//Auxillary adc input (AN2)
	int16_t			adc_load_0;			//Load cell input (AN3)
	int16_t			adc_load_1;			//Load cell input (AN4)
	int16_t			adc_load_2;			//Load cell input (AN5)
	int16_t			adc_load_3;			//Load cell input (AN6)
	int16_t			adc_load_4;			//Load cell input (AN7)
	int16_t			adc_load_5;			//Load cell input (AN8)
	int16_t			flags;				//Reserved
}M3LoadX6PdoV0Status;


/////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////  M3AH_PDO_V0 /////////////////////////////////////////////////////
#define M3AH_FINGER_CONFIG_FOO 1
#define M3AH_FINGER_CONFIG_BAR 2

typedef struct 
{
	int16_t		config;					//Reserved
	int16_t		mode;					//Mode (Off=0, Pwm=1)
	int16_t		pwm_cmd;				//Max permissible pwm 
} M3AhFingerPdoV0Cmd;

typedef struct
{
	M3AhFingerPdoV0Cmd command[1]; 
} M3AhFingerX1PdoV0Cmd;

#define M3AH_FINGER_FLAG_FOO 1
#define M3AH_FINGER_FLAG_BAR 2

typedef struct 
{
	int16_t		debug;			//Reserved
	int16_t		adc_current_a;		//Motor current leg A 
	int16_t		adc_current_b;		//Motor current leg B 
	int16_t		adc_ext_a;		//Auxillary adc input 
	int16_t		adc_ext_b;		//Auxillary adc input
	int16_t		force;			//AS5510 sensor
	int16_t		ir_emitter;		//Break-beam value
	int16_t		pwm_cmd;		//Current PWM command to motor
	int16_t		qei_enc_a;		//Encoder ticks
	int16_t		qei_enc_b;		//Encoder ticks
	int16_t		hall_state;		//Hall effect
	int16_t		flags;			//Reserved
	int16_t		data0;
	int16_t		data1;
	int16_t		data2;
	int16_t		data3;
	int16_t		data4;
	int16_t		data5;
	int16_t		data6;
	int16_t		data7;
	
}M3AhFingerPdoV0Status;


typedef struct
{
	uint64_t    timestamp;		    //Time in us
	M3AhFingerPdoV0Status 	status[1];
} M3AhFingerX1PdoV0Status;


#endif
