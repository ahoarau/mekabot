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

#ifdef USE_BLDC

#include "p33Fxxxx.h"
#include "setup.h"
#include "bldc.h"
#include "ethercat.h"
#include "timer3.h"
#include "dio.h"
#include "pwm.h"

volatile unsigned int bldc_fwd;
unsigned int bldc_hall_val;



/*************************************************************
	Low side driver table is as below.  In the StateLoTableClk
	and the StateLoTableAntiClk tables, the Low side driver is
	PWM while the high side driver is either on or off. See
	Microchip AN957
*************************************************************/

/*
Sensor Code order: 5 4 6 2 3 1 
X X POVD3H POVD3L   POVD2H POVD2L POVD1H POVD1L   X X POUT3H POUT3L   POUT2H POUT2L POUT1H POUT1L 
X X   Q5     Q6       Q3     Q4      Q1    Q2     X X   Q5     Q6       Q3    Q4      Q1    Q2
POVDXX=1: PWM
POVDXX=0: POUT

//4Q PWM
//Table based on Microchip AN857
Forward:
Code: 1 : Q1=On, Q6=PWM	 =	0011 0000 0000 0010 = 0x3002
Code: 2 : Q3=On, Q2=PWM	 =	0000 0011 0000 1000 = 0x0308
Code: 3 : Q3=On, Q6=PWM  =	0011 0000 0000 1000 = 0x3008
Code: 4 : Q5=On, Q4=PWM  =	0000 1100 0010 0000 = 0x0C20 
Code: 5 : Q1=On, Q4=PWM  =	0000 1100 0000 0010 = 0x0C02 
Code: 6 : Q5=On, Q2=PWM  =	0000 0011 0010 0000 = 0x0320 

//2Q PWM
//Table based on Microchip AN857
Forward:
Code: 1 : Q1=On, Q6=PWM	 =	0001 0000 0000 0010 = 0x1002
Code: 2 : Q3=On, Q2=PWM	 =	0000 0001 0000 1000 = 0x0108
Code: 3 : Q3=On, Q6=PWM  =	0001 0000 0000 1000 = 0x1008
Code: 4 : Q5=On, Q4=PWM  =	0000 0100 0010 0000 = 0x0420 
Code: 5 : Q1=On, Q4=PWM  =	0000 0100 0000 0010 = 0x0402 
Code: 6 : Q5=On, Q2=PWM  =	0000 0001 0010 0000 = 0x0120 
*/

//Pwm low leg

#ifdef PWM_4Q
unsigned int StateTableFwd[] = {0x0000, 0x3002, 0x0308, 0x3008,
									0x0C20, 0x0C02, 0x0320, 0x0000};
unsigned int StateTableRev[] = {0x0000, 0x0320, 0x0C02, 0x0C20,
								0x3008, 0x0308, 0x3002, 0x0000};
#endif
#ifdef PWM_2Q

unsigned int StateTableFwd[] = {0x0000, 0x1002, 0x0108, 0x1008,
									0x0420, 0x0402, 0x120, 0x0000};
unsigned int StateTableRev[] = {0x0000, 0x0120, 0x0402, 0x0420,
								0x1008, 0x0108, 0x1002, 0x0000};
#endif

/*
//ARM-H Commutation table:
unsigned int StateTableFwd[] = {0x0000, 0x2001, 0x0204, 0x2004,
									0x0810, 0x0801, 0x0210, 0x0000};
unsigned int StateTableRev[] = {0x0000, 0x0210, 0x0801, 0x0810,
								0x2004, 0x0204, 0x2001, 0x0000};
*/

void set_bldc_dir(unsigned int fwd)
{
	bldc_fwd=fwd;
	bldc_hall_val = BLDC_HALL_STATE; 
	ec_debug[0]=bldc_hall_val;
	if (bldc_fwd)
		P1OVDCON = StateTableFwd[bldc_hall_val];
	else
		P1OVDCON = StateTableRev[bldc_hall_val];
}

void __attribute__((__interrupt__, no_auto_psv)) _CNInterrupt(void)
{
	_CNIF=0;
	bldc_hall_val = BLDC_HALL_STATE;
	///if (bldc_hall_val==0 || bldc_hall_val==7)
		//ec_debug[0]=ec_debug[0]+1;//=bldc_hall_val;
	if (bldc_fwd)
		OVDCON = StateTableFwd[bldc_hall_val];
	else
		OVDCON = StateTableRev[bldc_hall_val];
}

void setup_bldc(void) {
	bldc_fwd=0;
	bldc_hall_val = BLDC_HALL_STATE;
	//ec_debug[0]=bldc_hall_val;
	OVDCON = StateTableRev[bldc_hall_val];

	CNEN1bits.CN1IE =1;		//Enable change-notification interrupt CN1: Hall1
	CNEN2bits.CN21IE =1;	//Enable change-notification interrupt CN21: Hall2
	CNEN2bits.CN22IE =1;	//Enable change-notification interrupt CN22: Hall3

	CNPU1bits.CN1PUE=1; //Enable weak pull-up on CN1
	CNPU2bits.CN21PUE=1; //Enable weak pull-up on CN21
	CNPU2bits.CN22PUE=1; //Enable weak pull-up on CN22

	_CNIF = 0;		//Clear change-notification Interrupt Status Flag
	_CNIE=1;//Enable change notification interrupt
}
#endif
