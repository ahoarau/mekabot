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

#ifdef USE_CONTROL


#include "setup.h"
#include "control.h"
#if defined M3_PWR_0_3 || defined M3_PWR_0_4 || defined M3_PWR_0_5
#include "timer3.h"
int pwr_status_cnt;
#endif

void step_ctrl_pid(int chid,int des);

/////////////////////////////////////////////////////////////

//3ch controller maxing out memory. If we disable D term can squeeze under the line
//, which is OK since not using for hand
//However, this used to work OK for H2R2. Something changed in total amount of memory 
#ifndef HB2_0_2_H2R3_J2J3J4
int  torque_delta[NUM_CTRL_CH][NUM_TORQUE_DOT_SAMPLES];
#endif
int fsa_state[NUM_CTRL_CH];
int ramp_idx[NUM_CTRL_CH];
long t_error_sum[NUM_CTRL_CH];
long torque_dot[NUM_CTRL_CH];


int  torque_prev[NUM_CTRL_CH];
int  td_idx[NUM_CTRL_CH];
ec_cmd_t gains;

long p_term[NUM_CTRL_CH];
long i_term[NUM_CTRL_CH];
long d_term[NUM_CTRL_CH];
long ff_term[NUM_CTRL_CH];

int t_error;
long result;


int ramp_pid_gains_up(int chid,int rate)
{
	int done=0;

	M3ActPdoV1Cmd * g = &(gains.command[chid]);
	M3ActPdoV1Cmd * d = &(ec_cmd.command[chid]);

	ramp_idx[chid]++;
	g->k_p_shift=d->k_p_shift;
	g->k_i_shift=d->k_i_shift;
	g->k_d_shift=d->k_d_shift;
	g->k_i_limit=d->k_i_limit;
	g->k_ff_shift=d->k_ff_shift;
	g->k_ff_zero=d->k_ff_zero;
	
	if (ramp_idx[chid]>=rate)
	{
		ramp_idx[chid]=0;
		done=1;
		if (g->k_p!=d->k_p)
		{
			g->k_p-=SIGN(g->k_p-d->k_p);
			g->k_p=CLAMP(g->k_p,-GAIN_LIMIT,GAIN_LIMIT);
			done=0;
		}
		if (g->k_i!=d->k_i)
		{
			g->k_i-=SIGN(g->k_i-d->k_i);
			g->k_i=CLAMP(g->k_i,-GAIN_LIMIT,GAIN_LIMIT);
			done=0;
		}
		if (g->k_d!=d->k_d)
		{
			g->k_d-=SIGN(g->k_d-d->k_d);
			g->k_d=CLAMP(g->k_d,-GAIN_LIMIT,GAIN_LIMIT);
			done=0;
		}
		if (g->k_ff!=d->k_ff)
		{
			g->k_ff-=SIGN(g->k_ff-d->k_ff);
			g->k_ff=CLAMP(g->k_ff,-GAIN_LIMIT,GAIN_LIMIT);
			done=0;
		}
	}
	return done;
}

int ramp_pid_gains_down(int chid,int rate)
{
	int done=0;
	M3ActPdoV1Cmd * g = &(gains.command[chid]);
	M3ActPdoV1Cmd * d = &(ec_cmd.command[chid]);
	ramp_idx[chid]++;

	g->k_p_shift=d->k_p_shift;
	g->k_i_shift=d->k_i_shift;
	g->k_d_shift=d->k_d_shift;
	g->k_i_limit=d->k_d_shift;
	g->k_ff_shift=d->k_ff_shift;
	g->k_ff_zero=d->k_ff_zero;
	
	if (ramp_idx[chid]>=rate)
	{
		ramp_idx[chid]=0;
		done=1;
		if (g->k_p!=0)
		{
			g->k_p-=SIGN(g->k_p);
			done=0;
		}
		if (g->k_i!=0)
		{
			g->k_i-=SIGN(g->k_i);
			done=0;
		}
		if (g->k_d!=0)
		{
			g->k_d-=SIGN(g->k_d);
			done=0;
		}
		if (g->k_ff!=0)
		{
			g->k_ff-=SIGN(g->k_ff);
			done=0;
		}
		
	}
	return done;
}


void setup_pid(int chid)
{
	M3ActPdoV1Cmd * g = &(gains.command[chid]);
	g->k_p=0;
	g->k_i=0;
	g->k_d=0;
	g->k_ff=0;
	g->k_i_limit=0;
	g->k_p_shift=0;
	g->k_i_shift=0;
	g->k_d_shift=0;
	g->k_ff_shift=0;
	torque_dot[chid]=0;
	t_error_sum[chid]=0;
#ifndef HB2_0_2_H2R3_J2J3J4
	memset(torque_delta[chid],0,NUM_TORQUE_DOT_SAMPLES*sizeof(int));
#endif
	torque_prev[chid]=0;
	td_idx[chid]=0;
	ramp_idx[chid]=0;
}

void setup_control() 
{
	int chid;
	for (chid=0;chid<NUM_CTRL_CH;chid++)
	{
		fsa_state[chid]=CTRL_OFF;
		setup_pid(chid);
	}
}



void step_amp_out(int chid, int des)
{
#if defined HB2_H2R1_J0J1 \
	|| defined HB2_H2R1_J2J3J4 || defined HB2_H2R2_J0J1 || defined HB2_H2R2_J2J3J4 \
	|| defined HB2_0_2_H2R3_J0J1 || defined HB2_0_2_H2R3_J2J3J4 
	set_pwm(chid,des);
#endif
}



#if defined HB2_H2R1_J0J1 || defined HB2_H2R1_J2J3J4 || defined HB2_H2R2_J0J1 || \
	defined HB2_H2R2_J2J3J4 \
	|| defined HB2_0_2_H2R3_J0J1 || defined HB2_0_2_H2R3_J2J3J4
void step_control()
{
int chid;


for ( chid=0;chid<NUM_CTRL_CH;chid++)
{

#ifdef EC_USE_WATCHDOG
	if (ec_watchdog_expired)
		fsa_state[chid]=CTRL_ABORT;
#endif

	int mode = ec_cmd.command[chid].mode;
	int des = ec_cmd.command[chid].t_desire;

	switch (fsa_state[chid])
	{
		case CTRL_OFF:

			step_amp_out(chid,0);
			if (mode==MODE_PWM)
				fsa_state[chid]=CTRL_PWM;
			if (mode==MODE_PID)
				fsa_state[chid]=CTRL_OFF_TO_PID;
				setup_pid(chid);
			break;

		case CTRL_PWM:

			step_amp_out(chid,des);
			if (mode!=MODE_PWM)
				fsa_state[chid]=CTRL_OFF;
			break;

		case CTRL_OFF_TO_PID:

			if (mode!=MODE_PID)
			{
				fsa_state[chid]=CTRL_OFF;
				break;
			}
			if (ramp_pid_gains_up(chid,RAMP_UPDATE_RATE))
					fsa_state[chid]=CTRL_PID;
			step_ctrl_pid(chid,des);
			break;

		case CTRL_PID:

			if (mode!=MODE_PID )
			{
				fsa_state[chid]=CTRL_PID_TO_OFF;
				break;
			}
			ramp_pid_gains_up(chid,RAMP_UPDATE_RATE);
			step_ctrl_pid(chid,des);
			break;

		case CTRL_PID_TO_OFF:

			if (ramp_pid_gains_down(chid,RAMP_UPDATE_RATE))
					fsa_state[chid]=CTRL_OFF;
			step_amp_out(chid,0);
			break;;
#ifdef EC_USE_WATCHDOG
			if (!ec_wd_expired)
				fsa_state[chid]=CTRL_OFF;
#endif
			break;
	default:

		step_amp_out(chid,0);
		break;
	};
}
}



void step_ctrl_pid(int chid,int des)
{

	M3ActPdoV1Cmd * g = &(gains.command[chid]);
	M3ActPdoV1Cmd * d = &(ec_cmd.command[chid]);
	int ddes = CLAMP(des,d->t_min,d->t_max);
	int s=0;

#if defined HB2_H2R1_J2J3J4 || defined HB2_H2R2_J2J3J4 || defined HB2_0_2_H2R3_J2J3J4
#if defined USE_ADC
	if (chid==0)
	{
		if (d->config&M3ACT_CONFIG_TORQUE_SMOOTH)
			s= get_avg_adc(ADC_SEAS_A);
		else
			s=adc_raw[ADC_SEAS_A];
	}
	if (chid==1)
	{
		if (d->config&M3ACT_CONFIG_TORQUE_SMOOTH)
			s= get_avg_adc(ADC_SEAS_B);
		else
			s=adc_raw[ADC_SEAS_B];
	}
	if (chid==2)
	{
		if (d->config&M3ACT_CONFIG_TORQUE_SMOOTH)
			s= get_avg_adc(ADC_SEAS_C);
		else
			s=adc_raw[ADC_SEAS_C];
	}
#endif
#endif

#if defined HB2_H2R1_J0J1 || defined HB2_H2R2_J0J1  || defined HB2_0_2_H2R3_J0J1
	if (chid==0)
	{
		if (d->config&M3ACT_CONFIG_TORQUE_SMOOTH)
			s= get_avg_adc(ADC_SEAS_A);
		else
			s=adc_raw[ADC_SEAS_A];
	}
	if (chid==1)
	{
		if (d->config&M3ACT_CONFIG_TORQUE_SMOOTH)
			s= get_avg_adc(ADC_SEAS_B);
		else
			s=adc_raw[ADC_SEAS_B];
	}
#endif

	t_error = (ddes-s);
	
	//Proportional term
	p_term[chid] = ((long)((long)g->k_p * (t_error>>g->k_p_shift)));//(t_error&(~mask))))>>gains.k_p_shift;   
	
	//Integral term
/*#ifdef M3_MAX2_BDC_A2R3
	t_error_sum[chid] += (g->k_i *t_error)>>g->k_i_shift;
	t_error_sum[chid]=CLAMP(t_error_sum[chid], -g->k_i_limit, g->k_i_limit);
	i_term[chid] = t_error_sum[chid];//(t_error_sum[chid]>>g->k_i_shift);
#else*/
	t_error_sum[chid] += t_error;
	t_error_sum[chid]=CLAMP(t_error_sum[chid], -g->k_i_limit, g->k_i_limit);
	i_term[chid] = g->k_i * (t_error_sum[chid]>>g->k_i_shift);
//#endif
	//Derivative term
	//Note: should actually normalize for time, but discretization is an issue...
	
#ifndef HB2_0_2_H2R3_J2J3J4
	int tdi=td_idx[chid];
	torque_dot[chid]= torque_dot[chid] - torque_delta[chid][tdi];
	torque_delta[chid][tdi] = s-torque_prev[chid];
	torque_prev[chid] = s;
	torque_dot[chid]=torque_dot[chid]+torque_delta[chid][tdi];
	td_idx[chid]=INC_MOD(tdi,NUM_TORQUE_DOT_SAMPLES);
	d_term[chid] = ((long)(g->k_d * torque_dot[chid]))>>g->k_d_shift;  
	ec_debug[chid]=(int)d_term[chid];
#else
	d_term[chid] = 0;
#endif
	//ec_debug[chid]=d_term[chid];
	//Feedforward term
	ff_term[chid]=0;
//Newer boards get FeedForward term from host
	if (d->config&M3ACT_CONFIG_TORQUE_FF)
	  ff_term[chid] = g->k_ff;
//Older boards use this form of FeedForward
#if !(defined  HB2_0_2_H2R3_J0J1 || defined HB2_0_2_H2R3_J2J3J4)
	else
	  ff_term[chid] = g->k_ff*((ddes-g->k_ff_zero)>>g->k_ff_shift);
#endif
	result=p_term[chid]+i_term[chid]+d_term[chid]+ff_term[chid];
	//ec_debug[chid]=ff_term[chid];
	
	result=CLAMP(result,-PWM_MAX_DUTY,PWM_MAX_DUTY);

	step_amp_out(chid,(int)result);
}
#endif
#endif //USE_CONTROL
