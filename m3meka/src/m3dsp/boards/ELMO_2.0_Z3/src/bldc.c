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

static volatile unsigned int bldc_fwd;
//unsigned int bldc_hall_val;
enum {
    OFF,
    BRAKE,
    COMMUTATION
} bldc_mode;

enum {
    BDC_OFF,
    BDC_ON
} bdc_mode;

void commutate();

static int16_t motor_pos;
const int16_t commutation_pos_table[8][8] = {{0,0,0,0,0,0,0,0},
											 {0,0,0,1,0,-1,0,0},
											 {0,0,0,-1,0,0,1,0},
											 {0,-1,1,0,0,0,0,0},
											 {0,0,0,0,0,1,-1,0},
											 {0,1,0,0,-1,0,0,0},
											 {0,0,-1,0,1,0,0,0},
											 {0,0,0,0,0,0,0,0}};

int16_t get_motor_pos()
{
	return motor_pos;
}


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
//unsigned int StateTableFwd[] = {0x0000, 0x3002, 0x0308, 0x3008,
//									0x0C20, 0x0C02, 0x0320, 0x0000};
//unsigned int StateTableRev[] = {0x0000, 0x0320, 0x0C02, 0x0C20,
//								0x3008, 0x0308, 0x3002, 0x0000};

// low side table
unsigned int StateTableFwd[] = {0x0000, 0x3001, 0x0304, 0x3004,
                                             0x0C10, 0x0C01, 0x0310, 0x0000};

unsigned int StateTableRev[] = {0x0000, 0x0310, 0x0C01, 0x0C10,
                                        0x3004, 0x0304, 0x3001, 0x0000};

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
    if (bldc_fwd != fwd) {
        bldc_fwd=fwd;

        if (bldc_mode == COMMUTATION)
            commutate();
    }

}

void __attribute__((__interrupt__, no_auto_psv)) _CNInterrupt(void)
{
	static int hall_last = 0;
	int hall_state = get_hall_state();
	if (bldc_mode == COMMUTATION)
        commutate();

	motor_pos += commutation_pos_table[hall_last][hall_state];


	hall_last = hall_state;
    _CNIF=0;
}

void setup_bldc(void)
{
    set_bldc_open();
	
	
	motor_pos = 0;
	CNPU1bits.CN1PUE=1; //Enable weak pull-up on CN1
    CNPU2bits.CN21PUE=1; //Enable weak pull-up on CN21
    CNPU2bits.CN22PUE=1; //Enable weak pull-up on CN22

    CNEN1bits.CN1IE =1;		//Enable change-notification interrupt CN1: Hall1
    CNEN2bits.CN21IE =1;	//Enable change-notification interrupt CN21: Hall2
    CNEN2bits.CN22IE =1;	//Enable change-notification interrupt CN22: Hall3

	_CNIF = 0;		//Clear change-notification Interrupt Status Flag
    _CNIE=1;            //Enable change notification interrupt
}

int get_hall_state()
{
    if(bdc_mode)
        return (2);
    else
        return (BLDC_HALL_STATE);
}

void set_bldc_open()
{
    bldc_mode = OFF;
//    P1OVDCON = 0x3F00;
    P1OVDCON = 0x0000;
 //   _CNIE=0;
}

void set_bldc_brake()
{
    bldc_mode = BRAKE;
    // set all low legs to on, kill interrupts
//    P1OVDCON = 0x3F15;
    P1OVDCON = 0x0015;
//    _CNIE=0;
}

void set_bldc_commutation()
{
    if (bldc_mode == COMMUTATION)
        return;

    bldc_mode = COMMUTATION;

//    CNPU1bits.CN1PUE=1; //Enable weak pull-up on CN1
//    CNPU2bits.CN21PUE=1; //Enable weak pull-up on CN21
//   CNPU2bits.CN22PUE=1; //Enable weak pull-up on CN22

//    CNEN1bits.CN1IE =1;		//Enable change-notification interrupt CN1: Hall1
//    CNEN2bits.CN21IE =1;	//Enable change-notification interrupt CN21: Hall2
//    CNEN2bits.CN22IE =1;	//Enable change-notification interrupt CN22: Hall3

    set_bldc_dir(0);
    commutate();



//    _CNIF = 0;		//Clear change-notification Interrupt Status Flag
//    _CNIE=1;            //Enable change notification interrupt
}

void commutate()
{
    int hall_val = get_hall_state();


    if (bldc_fwd)
        P1OVDCON = StateTableFwd[hall_val];
    else
        P1OVDCON = StateTableRev[hall_val];
}

int get_bldc_dir()
{
    return bldc_fwd;
}

void set_bldc_mode(int bldc_bdc_mode)
{
    // 1 for bldc, 0 for bdc
    bdc_mode = !bldc_bdc_mode;
}


#endif

