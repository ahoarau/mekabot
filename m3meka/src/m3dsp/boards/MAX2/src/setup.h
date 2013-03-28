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
///////////////////////////////////////////////////////////////////

//Configure the components to use with these compiler options
//#define USE_PWM
//#define USE_ENCODER_VERTX
//#define USE_ADC
//#define USE_CONTROL
//#define USE_CURRENT
//#define USE_ETHERCAT
//#define USE_TIMER1
//#define USE_DIO
//#define USE_SYNC0					//Use Sync0 EtherCAT IRQ
//#define USE_TIMESTAMP_DC			//Use DC CLOCK Timestamp from LATCH
//#define USE_MAX2_0_2				//Choose the right MAX2 version
//#define USE_MAX2_0_3

#include "p33Fxxxx.h"
#include "inttypes.h"
#include <string.h> /* memset */
//#include "../../m3rt/base/m3ec_def.h"
#include "..\..\..\..\m3\hardware\m3ec_pdo_v1_def.h"
#include "..\..\..\..\m3\hardware\m3ec_pdo_v3_def.h"

#include "dio.h"
#include "ethercat.h"
#include "timer1.h"
#include "encoder_vertx.h"
#include "pwm.h"
#include "control.h"
#include "adc.h"
#include "current.h"
#include "timer3.h"
#include "bldc.h"
#include "brake.h"
#include "adc_spi.h"

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
