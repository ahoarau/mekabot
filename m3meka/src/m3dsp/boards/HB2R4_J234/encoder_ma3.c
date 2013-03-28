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

#ifdef USE_ENCODER_MA3

#include "p33Fxxxx.h"
#include "setup.h"
#include "encoder_ma3.h"
#include "ethercat.h"
#include "timer3.h"
#include "dio.h"


int t_on[NUM_MA3_CH];
int t_period[NUM_MA3_CH];
int t_rise[NUM_MA3_CH];
int t_rise_last[NUM_MA3_CH];
int t_fall[NUM_MA3_CH];
int first_rise[NUM_MA3_CH];
int rising[NUM_MA3_CH];
int t_on_last[NUM_MA3_CH];
int t_period_last[NUM_MA3_CH];
int lbound_on[NUM_MA3_CH];
int ubound_on[NUM_MA3_CH];
int rollover_cnt[NUM_MA3_CH];
int delta[NUM_MA3_CH];
int startup_cnt[NUM_MA3_CH];

int  ma3_on(int chid) 
{
	return t_on_last[chid];
}

int  ma3_period(int chid)
{
	return t_period_last[chid];
}

//Return true if past soft limit
int ma3_at_lower_bound(int chid)
{
	int lbound;
	if (lbound_on[chid])
		lbound=CLAMP(ec_cmd.command[chid].qei_min+MA3_BOUND_HYSTERISIS,0,MA3_MAX_PULSE_TICKS);
	else
		lbound=CLAMP(ec_cmd.command[chid].qei_min,0,MA3_MAX_PULSE_TICKS);
	lbound_on[chid]=(ec_cmd.command[chid].config&M3ACT_CONFIG_ENC_BOUNDS) && t_on_last[chid]<lbound;
	return lbound_on[chid];
}

//Return true if past soft limit
int ma3_at_upper_bound(int chid)
{
	int ubound;
	if (ubound_on[chid])
		ubound=CLAMP(ec_cmd.command[chid].qei_max-MA3_BOUND_HYSTERISIS,0,MA3_MAX_PULSE_TICKS);
	else
		ubound=CLAMP(ec_cmd.command[chid].qei_max,0,MA3_MAX_PULSE_TICKS);
	ubound_on[chid]= (ec_cmd.command[chid].config&M3ACT_CONFIG_ENC_BOUNDS) && t_on_last[chid]>ubound;
	return ubound_on[chid];
}

//Better to do this in Linux with FPU
int ma3_pos(int chid)
{
	long res = t_on_last[chid]*MA3_PERIOD_US;
	res=res/t_period_last[chid];
	res=(res/t_period_last[chid])-1;
	return (int)res;
}


int  ma3_rollover(int chid)
{
	return rollover_cnt[chid];
}


void __attribute__((__interrupt__, no_auto_psv)) _IC1Interrupt(void)
{	
	if (first_rise[0])
	{
		while (IC1CONbits.ICBNE)
		{
			t_rise_last[0]=t_rise[0];
			t_rise[0]=IC1BUF;
		}
		first_rise[0]=0;
		IC1CONbits.ICM=0;	//Disable IC1
		IC1CONbits.ICM=1;	//Capture all edges
		rising[0]=0;
	}
	else
	{
		while (IC1CONbits.ICBNE)
		{
			if (rising[0])
			{
				t_rise_last[0]=t_rise[0];
				t_rise[0]=IC1BUF;
				rising[0]=0;
					
				if (t_rise[0]>t_rise_last[0])
					t_period[0]=t_rise[0]-t_rise_last[0];
				else
					t_period[0]=(PR2-t_rise_last[0])+t_rise[0];

				if (t_rise_last[0]<t_fall[0])
					t_on[0]=t_fall[0]-t_rise_last[0];
				else
					t_on[0]=(PR2-t_rise_last[0])+t_fall[0];
			}
			else
			{
				t_fall[0]=IC1BUF;
				rising[0]=1;
			}
		}
	}

	//Allow for rollover in each direction
	if (startup_cnt[0]>0)
		startup_cnt[0]--;
	else
	{
		delta[0]=t_on[0]-t_on_last[0];
		if (ABS(delta[0])>MA3_ROLLOVER_THRESH) //large jump, assume rollover
		{
			if (delta[0]<0) rollover_cnt[0]++; else rollover_cnt[0]--;
		}
	}

	t_on_last[0]=t_on[0];
	t_period_last[0]=t_period[0];
	_IC1IF = 0;		//Clear the flag
}

#if defined M3_HMB_H1R1 || defined M3_GMB_G1R1 || defined M3_HEX2_S1R1 || defined M3_HB2_H2R1_J0J1 || defined M3_HB2_H2R1_J2J3J4 || \
	defined M3_HB2_H2R2_J0J1 || defined M3_HB2_H2R2_J2J3J4 || defined M3_HB2_H2R3_J0J1 || defined M3_HB2_H2R3_J2J3J4
void __attribute__((__interrupt__, no_auto_psv)) _IC2Interrupt(void)
{	
	if (first_rise[1])
	{
		while (IC2CONbits.ICBNE)
		{
			t_rise_last[1]=t_rise[1];
			t_rise[1]=IC2BUF;
		}
		first_rise[1]=0;
		IC2CONbits.ICM=0;	//Disable IC2
		IC2CONbits.ICM=1;	//Capture all edges
		rising[1]=0;
	}
	else
	{
		while (IC2CONbits.ICBNE)
		{
			if (rising[1])
			{
				t_rise_last[1]=t_rise[1];
				t_rise[1]=IC2BUF;
				rising[1]=0;
					
				if (t_rise[1]>t_rise_last[1])
					t_period[1]=t_rise[1]-t_rise_last[1];
				else
					t_period[1]=(PR2-t_rise_last[1])+t_rise[1];

				if (t_rise_last[1]<t_fall[1])
					t_on[1]=t_fall[1]-t_rise_last[1];
				else
					t_on[1]=(PR2-t_rise_last[1])+t_fall[1];
			}
			else
			{
				t_fall[1]=IC2BUF;
				rising[1]=1;
			}
		}
	}

	//Allow for rollover in each direction
	if (startup_cnt[1]>0)
		startup_cnt[1]--;
	else
	{
		delta[1]=t_on[1]-t_on_last[1];
		if (ABS(delta[1])>MA3_ROLLOVER_THRESH) //large jump, assume rollover
		{
			if (delta[1]<0) rollover_cnt[1]++; else rollover_cnt[1]--;
		}
	}

	t_on_last[1]=t_on[1];
	t_period_last[1]=t_period[1];
	_IC2IF = 0;		//Clear the flag
}
#endif


#if defined M3_HB2_H2R1_J2J3J4 || defined M3_HB2_H2R2_J2J3J4 || defined M3_HB2_H2R3_J2J3J4
void __attribute__((__interrupt__, no_auto_psv)) _IC7Interrupt(void)
{	
	if (first_rise[2])
	{
		while (IC7CONbits.ICBNE)
		{
			t_rise_last[2]=t_rise[2];
			t_rise[2]=IC7BUF;
		}
		first_rise[2]=0;
		IC7CONbits.ICM=0;	//Disable IC2
		IC7CONbits.ICM=1;	//Capture all edges
		rising[2]=0;
	}
	else
	{
		while (IC7CONbits.ICBNE)
		{
			if (rising[2])
			{
				t_rise_last[2]=t_rise[2];
				t_rise[2]=IC7BUF;
				rising[2]=0;
					
				if (t_rise[2]>t_rise_last[2])
					t_period[2]=t_rise[2]-t_rise_last[2];
				else
					t_period[2]=(PR2-t_rise_last[2])+t_rise[2];

				if (t_rise_last[2]<t_fall[2])
					t_on[2]=t_fall[2]-t_rise_last[2];
				else
					t_on[2]=(PR2-t_rise_last[2])+t_fall[2];
			}
			else
			{
				t_fall[2]=IC7BUF;
				rising[2]=1;
			}
		}
	}

	//Allow for rollover in each direction
	if (startup_cnt[2]>0)
		startup_cnt[2]--;
	else
	{
		delta[2]=t_on[2]-t_on_last[2];
		if (ABS(delta[2])>MA3_ROLLOVER_THRESH) //large jump, assume rollover
		{
			if (delta[2]<0) rollover_cnt[2]++; else rollover_cnt[2]--;
		}
	}

	t_on_last[2]=t_on[2];
	t_period_last[2]=t_period[2];
	_IC7IF = 0;		//Clear the flag
}
#endif

void setup_ma3(void) {
	int i;
	for (i=0;i<NUM_MA3_CH;i++)
	{
		first_rise[i]=1;
		rising[i]=0;
		t_rise[i]=0;
		t_rise_last[i]=0;
		t_fall[i]=0;
		t_on[i]=0;
		t_period[i]=0;
		t_on_last[i]=0;
		t_period_last[i]=0;
		lbound_on[i]=0;
		ubound_on[i]=0;
		delta[i]=0;
		rollover_cnt[i]=0;
		startup_cnt[i]=10;
	}



	//Setup Timer2
	T2CON = 0;
	TMR2 = 0x0000;
	T2CONbits.TCKPS=1;							// 1:8 Prescalar, so 1 tick = 200ns , 5 ticks/us
	PR2 = (unsigned int)T2_PERIOD_TICKS;		//Set period to roll-over every T2_PERIOD_TICKS
	T2CONbits.TON = 1;							//Start Timer 2


	//Setup input capture 1
	IC1CONbits.ICM=0;	//Disable IC1
	IC1CONbits.ICTMR=1; //Use Timer2 time-base
	IC1CONbits.ICI=0;	//Interrupt every capture event
	IC1CONbits.ICM=3;	//Capture rising edge
	IFS0bits.IC1IF = 0; //Clear IC1 Interrupt Status Flag
	IEC0bits.IC1IE = 1; //Enable IC1 interrupt

#if defined M3_HMB_H1R1 || defined M3_GMB_G1R1 || defined M3_HEX2_S1R1 || defined M3_HB2_H2R1_J0J1 || \
	defined M3_HB2_H2R1_J2J3J4 || defined M3_HB2_H2R2_J0J1 || defined M3_HB2_H2R2_J2J3J4 \
	|| defined M3_HB2_H2R3_J0J1 || defined M3_HB2_H2R3_J2J3J4
	IC2CONbits.ICM=0;	//Disable IC2
	IC2CONbits.ICTMR=1; //Use Timer2 time-base
	IC2CONbits.ICI=0;	//Interrupt every capture event
	IC2CONbits.ICM=3;	//Capture rising edge
	IFS0bits.IC2IF = 0; //Clear IC2 Interrupt Status Flag
	IEC0bits.IC2IE = 1; //Enable IC2 interrupt
#endif
#if defined M3_HB2_H2R1_J2J3J4 || defined M3_HB2_H2R2_J2J3J4 || defined M3_HB2_H2R3_J2J3J4
	IC7CONbits.ICM=0;	//Disable IC7
	IC7CONbits.ICTMR=1; //Use Timer2 time-base
	IC7CONbits.ICI=0;	//Interrupt every capture event
	IC7CONbits.ICM=3;	//Capture rising edge
	IFS1bits.IC7IF = 0; //Clear IC7 Interrupt Status Flag
	IEC1bits.IC7IE = 1; //Enable IC7 interrupt*
#endif
}
#endif
