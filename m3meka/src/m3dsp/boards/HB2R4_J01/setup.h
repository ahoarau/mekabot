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


#ifndef _SETUP_H
#define _SETUP_H

//Configure the desired board with these compiler options

//////////////////// Supported Boards ///////////////////////////

//#define M3_DAC_0_1		////DAC based external amp controller. Version 0.1: 4 DOF Proto Board with SPI DAC.

//#define M3_ELMO_RNA_R0	//DAC based external Elmo controller, RoNA version.
//#define M3_ELMO_B1R1		//DAC based external Elmo controller, B1.R1 base version.
//#define M3_ELMO_Z1R1		//DAC based external Elmo controller, Z1.R1 z-lift version.

//#define M3_WMA_0_1		//WMA DC motor amplifier for A1.R1 arms, S1.R1 head, and T1.R1 torso (SEAX2 0.1)

//#define M3_BMA_A1R1		//BMA DC/BLDC amplifier for SEAX2 0.1 board in A2R1 arm


//#define M3_MAX2				//All MAX2 BDC/BLDC amplifiers
//#define M3_MAX2_MA3		    //MAX2 support for MA3 encoder feedback on joint angle
//#define M3_MAX2_BDC_A2R1		//Brushed DC motor for A2.R1 arm (J0,J1) and SEAX2 1.1 board
//#define M3_MAX2_BLDC_A2R1		//Brushless DC motor for A2.R1 arm (J2,J3) and SEAX2 1.1 board
//#define M3_MAX2_BDC_A2R2		//Brushed DC motor for A2.R2 arm (J0,J1) and SEAX2 1.2 board
//#define M3_MAX2_BDC_A2R3		//Brushed DC motor for A2.R3 arm (J0,J1) and SEAX2 1.2 board
//#define M3_MAX2_BDC_ARMH		//Brushed DC motor for ArmH test controller and SEAX2 1.2 board
//#define M3_MAX2_BDC_ARMH2		//Brushed DC motor for ArmH test controller and SEAX2 1.2 board (no push buttons)
//#define M3_MAX2_BLDC_A2R2		//Brushless DC motor for A2.R2 arm (J2,J3) and SEAX2 1.2 board
//#define M3_MAX2_BLDC_A2R3		//Brushless DC motor for A2.R3 arm (J2,J3) and SEAX2 1.2 board
//#define M3_MAX2_BDC_T2R1		//Brushed DC motor for T2.R1 torso (J1) and SEAX2 1.1 board
//#define M3_MAX2_BLDC_T2R1		//Brushless DC motor for T2.R1 torso (J0) and SEAX2 1.1 board
//#define M3_MAX2_BDC_T2R2		//Brushed DC motor for T2.R2 torso (J1) and SEAX2 1.2 board
//#define M3_MAX2_BDC_T2R3		//Brushed DC motor for T2.R3 torso (J1) and SEAX2 1.2 board
//#define M3_MAX2_BLDC_T2R2		//Brushless DC motor for T2.R2 torso (J0) and SEAX2 1.2 board
//#define M3_MAX2_BLDC_T2R3		//Brushless DC motor for T2.R3 torso (J0) and SEAX2 1.2 board
//#define M3_MAX2_BDC_S1R1      //Brushed DC motor for S1.R1 head (upgrade) and SEAX2 1.2 board. MA3 encoder feedback
//#define M3_MAX2_AS5510		//AS5510 Hall encoder test

//#define M3_BMW_A2R1				//BLDC amplifier for A2.R1 arm J4J5J6
//#define M3_BMW_A2R2				//BLDC amplifier for A2.R2 arm J4J5J6
//#define M3_BMW_A2R3				//BLDC amplifier for A2.R3 arm J4J5J6

//#define M3_HEX4_S2R1				//BMW style amplifier for S2.R1 head

//#define M3_PWR_0_2		//Version 0.2: Active board for A1 arm 
//#define M3_PWR_0_3		//Version 0.3: Active board for A2.R1 arm
//#define M3_PWR_0_4		//Version 0.4: Active board for A2.R2 arm
//#define M3_PWR_0_5		//Version 0.5: Active board for A2.R3 arm

//#define M3_LOADX6_A2R1				//6 axis load cell conditioner, A2R1 arm.
//#define M3_LOADX6_A2R2				//6 axis load cell conditioner, A2R2 arm.
//#define M3_LOADX6_A2R3				//6 axis load cell conditioner, A2R3 arm.

//#define M3_HMB_H1R1				//Hand 4ch control board, all versions.
//#define M3_HMB_H1R1_0_1		//Version 0.1: Active board for H1 arms
 

//#define M3_HB2_H2R1_J0J1		    //Board for H2.R1 hand, joints 0-1
//#define M3_HB2_H2R1_J2J3J4		//Board for H2.R1 hand, joints 2-4
//#define M3_HB2_H2R2_J0J1		    //Board for H2.R2 hand, joints 0-1
//#define M3_HB2_H2R2_J2J3J4		//Board for H2.R2 hand, joints 2-4
//#define M3_HB2_H2R3_J0J1		    //Board for H2.R3 hand, joints 0-1
//#define M3_HB2_H2R3_J2J3J4		//Board for H2.R3 hand, joints 2-4

//#define M3_DEV				//M3 EC Development board, all versions.
//#define M3_DEV_0_0		//Version 0.0: SPI test board

//#define M3_GMB_G1R1				//Gripper 2ch control board, all versions.

//#define M3_FB_DEV_0_0				//Arm-HFinger development Board


//#define M3_HEX2_S1R1				//4ch control board for S1.R1 head

//#define M3_LEDX2_S1R1				//2ch interface board for the MegaBrite RGB LED controllers, S1.R1 head version
//#define M3_LEDX2XN_S2R1			//2ch interface board for the 12ch LED Ring boards, S2.R1 head version

//#define M3_BLD				//Bootloader for M3EC board.
///////////////////////////////////////////////////////////////////

//Configure the components to use with these compiler options
//#define USE_UART
//#define USE_PWM
//#define USE_ENCODER_MA3
//#define USE_ENCODER_VERTX
//#define USE_ADC
//#define USE_ADC_SPI
//#define USE_CONTROL
//#define USE_ETHERCAT
//#define USE_TIMER3
//#define USE_DIO
//#define USE_DAC
//#define USE_SYNC0					//Use Sync0 EtherCAT IRQ
//#define USE_TIMESTAMP_DC			//Use DC CLOCK Timestamp from LATCH
//#define USE_TACTILE_PPS			//Tactile sensor
//#define USE_BLDC					//BLDC commutation
//#define USE_LED_RGB				//RGB Led array
//#define USE_BRAKE					//Motor brake
//#define USE_BLDC_INPUT_CAPTURE	//Use Hall Input Capture for BLDC
//#define USE_BLDC_CHANGE_NOTIFICATION //Use Hall Change Notification for BLDC
//#define USE_AS5510					//AS5510 Hall effect sensor


#include "p33Fxxxx.h"
#include "../../../m3/hardware/m3ec_def.h"
#include "../../../m3/hardware/m3ec_pdo_v1_def.h"
#include "../../../m3/hardware/m3ec_pdo_v2_def.h"
#include "../../../m3/hardware/m3ec_pdo_v0_def.h" //needed for M3_FB_DEV_0_0 ??

#include "dio.h"
#include "ethercat.h"
#include "timer3.h"
#include "encoder_ma3.h"
#include "encoder_vertx.h"
#include "encoder_qei.h"
#include "pwm.h"
#include "control.h"
#include "dac.h"
#include "adc.h"
#include "adc_spi.h"
#include "uart1.h"
#include "tactile_pps.h"
#include "bldc.h"
#include "led_rgb.h"
#include "brake.h"
#include "led_mdrv.h"
#include "encoder_as5510.h"
//#include "m3serial.h"

//#define EC_USE_WATCHDOG

//Status flags

#define FCY   40000000 //Clock frequency
#define MS_DELAY_CONST 8000 // good for 40 MHz and OL s
#define US100_DELAY_CONST 800
#define US_DELAY_CONST 8

#define MAX(a, b)  (((a) > (b)) ? (a) : (b))
#define MIN(a, b)  (((a) < (b)) ? (a) : (b))
#define ABS(a)	   (((a) < 0) ? -(a) : (a))
#define SIGN(a)	   (((a) < 0) ? -1 : 1)
#define INC_MOD(a, b)  (((a+1) >= (b)) ? 0 : (a+1))
#define XINC_MOD(a, b, c)  (((a+c) >= (b)) ? a+c-b : (a+c))
#define CLAMP(x, low, high)  (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))


void setup_oscillator(void);
void setup_ports(void);
void setup_peripheral_pin_select(void);
void setup_interrupt_priorities(void);


#ifndef M3_BLD
#define DELAY_25NS asm("nop");
#define DELAY_100NS asm("nop");asm("nop");asm("nop");asm("nop");
#define DELAY_225NS DELAY_100NS DELAY_100NS DELAY_25NS
#define DELAY_75NS asm("nop");asm("nop");asm("nop");
#define DELAY_500NS DELAY_100NS DELAY_100NS DELAY_100NS DELAY_100NS DELAY_100NS
#define DELAY_700NS DELAY_500NS DELAY_100NS DELAY_100NS
#define DELAY_1000NS DELAY_500NS DELAY_500NS 
#define DELAY_1200NS DELAY_1000NS DELAY_100NS DELAY_100NS
#define DELAY_2300NS DELAY_1000NS DELAY_1000NS DELAY_100NS DELAY_100NS DELAY_100NS
#define DELAY_5000NS DELAY_1000NS DELAY_1000NS DELAY_1000NS DELAY_1000NS DELAY_1000NS\
#define DELAY_15000NS DELAY_5000NS DELAY_5000NS DELAY_5000NS

void ms_delay(int n);
void us100_delay(int n);
void us_delay(int n);
void ns_delay(int n);

#define INTERRUPT_PROTECT_ENABLE {              \
    char saved_ipl;                          \
    SET_AND_SAVE_CPU_IPL(saved_ipl,7);       \

#define INTERRUPT_PROTECT_DISABLE		\
    RESTORE_CPU_IPL(saved_ipl); } (void) 0;

#else
#define INTERRUPT_PROTECT_ENABLE
#define INTERRUPT_PROTECT_DISABLE
#endif

#endif
