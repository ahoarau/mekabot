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


#ifndef M3EC_PDO_V1_H
#define M3EC_PDO_V1_H

#ifdef __KERNEL__
#define int16_t short
#define int32_t int
#else
#ifndef EMBEDDED
#include <sys/types.h>
#include <stdint.h>
#endif
#endif

///////////////////////////////  M3ACT_PDO_V1 /////////////////////////////////////////////////////
//NOTE: This is replicated in actuator_ec.proto 
#define M3ACT_CONFIG_TORQUE_SMOOTH 2
#define M3ACT_CONFIG_ENC_BOUNDS 16 	   //Limit PWM within QEI range
#define M3ACT_CONFIG_PWM_FWD_SIGN 32  //Does +PWM => +QEI change
#define M3ACT_CONFIG_BRAKE_OFF 64 	   //Turn brake off if present
#define M3ACT_CONFIG_VERTX_FILTER_OFF 128 //Disable VertX filter
#define M3ACT_CONFIG_LIMITSWITCH_STOP_POS 256 //Use limitswitch stop in +pwm directional
#define M3ACT_CONFIG_LIMITSWITCH_STOP_NEG 512 //Use limitswitch stop in +pwm directional
#define M3ACT_CONFIG_CALIB_QEI_LIMITSWITCH_POS 1024 //zero encoder on low-to-high transition of +limitswitch
#define M3ACT_CONFIG_CALIB_QEI_LIMITSWITCH_NEG 2048 //zero encoder on low-to-high transition of -limitswitch
#define M3ACT_CONFIG_TORQUE_FF 4096 //Incorporate torque feed-forward term into the DSP PID Controller
#define M3ACT_CONFIG_CALIB_QEI_MANUAL 8192 //Zero encoder on low to high transition
#define M3ACT_CONFIG_EC_WD 16384 //Toggle this bit for EtherCAT heartbeat/watchdog


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
	int16_t		k_ff_zero;				//Deprecated (Zero point for ff term, torque control)
	int16_t		k_ff_shift;				//Deprecated (Shift scalar, torque control)
	int16_t		k_ff;					//Feedforward gain, torque control
	int16_t		pwm_db;					//Pwm deadband
} M3ActPdoV1Cmd;
// 38 bytes

#define M3ACT_FLAG_POS_LIMITSWITCH 1
#define M3ACT_FLAG_NEG_LIMITSWITCH 2
#define M3ACT_FLAG_QEI_CALIBRATED 4
#define M3ACT_FLAG_AUX_SWITCH 8
#define M3ACT_FLAG_I_FAULT_CONT 16
#define M3ACT_FLAG_I_FAULT_MOM 32
typedef struct 
{
	int16_t		debug;				//Reserved
	int16_t		adc_torque;			//Torque input 
	int16_t		adc_motor_temp;		//Motor temp
	int16_t		adc_ext_a;			//Auxillary adc input 
	int16_t		adc_ext_b;			//Auxillary adc input
	int16_t		adc_amp_temp;		//Amplifier temp 
	int16_t		adc_current_a;		//Motor current leg A 
	int16_t		adc_current_b;		//Motor current leg B 
	int16_t		pwm_cmd;			//Current PWM command to motor
	int16_t		qei_on;				//Encoder ticks
	int16_t		qei_period;			//Encoder ticks
	int16_t		qei_rollover;		//Encoder rollover counts (directional) -1|0|1 ...
	int16_t		flags;				//Reserved
}M3ActPdoV1Status;
// 26 bytes

///////////////////////////////  M3ACT_PDO_V1 Multi-Channel /////////////////////////////////////////////////////

typedef struct
{
	M3ActPdoV1Cmd command[1]; 
} M3ActX1PdoV1Cmd;

typedef struct
{
	M3ActPdoV1Cmd command[2];
} M3ActX2PdoV1Cmd;

typedef struct
{
	M3ActPdoV1Cmd command[3];
} M3ActX3PdoV1Cmd;

typedef struct
{
	M3ActPdoV1Cmd command[4];
} M3ActX4PdoV1Cmd;



typedef struct
{
	uint64_t    timestamp;		    //Time in us
	M3ActPdoV1Status 	status[1];
} M3ActX1PdoV1Status;
// 34 bytes

typedef struct
{
	uint64_t    timestamp;		    //Time in us
	M3ActPdoV1Status 	status[2];
} M3ActX2PdoV1Status;
// 60 bytes


typedef struct
{
	uint64_t    timestamp;		    //Time in us
	M3ActPdoV1Status 	status[3];
} M3ActX3PdoV1Status;

typedef struct
{
	uint64_t    timestamp;		    //Time in us
	M3ActPdoV1Status 	status[4];
} M3ActX4PdoV1Status;

///////////////////////////////  Tactile /////////////////////////////////////////////////////

//22 Taxel PRT Pressure Profile Systems Sensor	
typedef struct 
{
	uint16_t			taxel[22];			//PPS 22 taxel array
}M3TactilePPS22V1Status;


typedef struct
{
	uint64_t    timestamp;		    //Time in us
	M3ActPdoV1Status 	status[2];
	M3TactilePPS22V1Status tactile[2];
} M3TactX2PdoV1Status;

typedef M3ActX1PdoV1Cmd M3TactX2PdoV1Cmd;

///////////////////////////////  PWR //////////////////////////////////////////////////////////

typedef struct 
{
	int16_t		config;					//Reserved
	int16_t		enable_motor;				//Software enable of motor bus voltage (RP12)
}M3PwrPdoV1Cmd;

typedef struct 
{
	uint64_t		timestamp;			//Time in us
	int16_t			mode_remote;			//Mode switch on remote (RP5)
	int16_t			motor_enabled;			//State of the motor bus relay (RP6)
	int16_t			adc_bus_voltage;		//Voltage input (AN0)
	int16_t			adc_current_digital;		//Digital logic power consumption (AN1)
	int16_t			adc_ext;			//Auxillary adc input (AN2)
	int16_t			flags;				//Reserved
}M3PwrPdoV1Status;

///////////////////////////////  LedX2 //////////////////////////////////////////////////////////

typedef struct 
{
	int16_t		r;
	int16_t		g;		
	int16_t		b;
}M3LedX2RGBV1;
typedef struct 
{
	M3LedX2RGBV1 board_a;
	M3LedX2RGBV1 board_b;
}M3LedX2BranchV1;
typedef struct 
{
	int16_t		config;					//Reserved
	M3LedX2BranchV1 branch_a;
	M3LedX2BranchV1 branch_b;
	int16_t  enable_a;
	int16_t	 enable_b;
}M3LedX2PdoV1Cmd;
typedef struct 
{
	uint64_t		timestamp;			//Time in us
	int16_t			flags;				//Reserved
	int16_t			debug;
	int16_t			adc_ext_a;
	int16_t			adc_ext_b;
	int16_t			adc_ext_c;
	int16_t			adc_ext_d;
}M3LedX2PdoV1Status;
///////////////////////////////  LedX2 //////////////////////////////////////////////////////////

typedef struct 
{
	int16_t		r;
	int16_t		g;		
	int16_t		b;
	int16_t		id;
}M3LedX2XNRGBV1;

typedef struct 
{
	int16_t	 config;		//Reserved
	M3LedX2XNRGBV1 branch[2];
	int16_t  enable[2];
}M3LedX2XNPdoV1Cmd;

typedef struct 
{
	uint64_t		timestamp;			//Time in us
	int16_t			flags;				//Reserved
	int16_t			debug;
	int16_t			adc_ext_a;
	int16_t			adc_ext_b;
	int16_t			adc_ext_c;
	int16_t			adc_ext_d;
}M3LedX2XNPdoV1Status;

///////////////////////////////  LOADX6 //////////////////////////////////////////////////////

typedef struct 
{
	int16_t		config;					//Reserved
}M3LoadX6PdoV1Cmd;

typedef struct 
{
	uint64_t		timestamp;			//Time in us
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
}M3LoadX6PdoV1Status;


///////////////////////////////  BOOTLOADER //////////////////////////////////////////////////

#define LED_MTX_BUF_SIZE 12

typedef struct 
{
	unsigned char		r;
	unsigned char		g;		
	unsigned char		b;	
	unsigned char		idx;
}M3LedMatrixRGBV1;

typedef struct 
{	
	unsigned char  enable;
	M3LedMatrixRGBV1 array[LED_MTX_BUF_SIZE];
	
}M3LedMatrixPdoV1Cmd;

typedef struct 
{
	int32_t timestamp;						//Number of data bytes	
	int32_t debug;
	int32_t flags;
}M3LedMatrixPdoV1Status;


#endif
