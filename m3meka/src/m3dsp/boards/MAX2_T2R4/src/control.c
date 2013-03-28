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
#include "brake.h"
#include "adc_spi.h"

int t_error;
volatile long result;
ec_cmd_t gains;

int  torque_delta[NUM_CTRL_CH][NUM_TORQUE_DOT_SAMPLES];
int fsa_state[NUM_CTRL_CH];
int ramp_idx[NUM_CTRL_CH];
long t_error_sum[NUM_CTRL_CH];
long torque_dot[NUM_CTRL_CH];
int  torque_prev[NUM_CTRL_CH];
int  td_idx[NUM_CTRL_CH];

long p_term[NUM_CTRL_CH];
long i_term[NUM_CTRL_CH];
long d_term[NUM_CTRL_CH];
long ff_term[NUM_CTRL_CH];

extern unsigned int watchdog_expired;

//Prototypes:
void step_torque_pid(int chid,int des);
void step_current_pid(int chid,int des);

//Functions:

int ramp_pid_gains_up(int chid,int rate)
{
	int done=0;

	#if defined MAX2_BDC_0_2_A2R3 || defined MAX2_BLDC_0_2_A2R3 || defined MAX2_BDC_0_2_T2R3 || defined MAX2_BLDC_0_2_T2R3 
	M3ActPdoV1Cmd * g = &(gains.command[chid]);
	M3ActPdoV1Cmd * d = &(ec_cmd.command[chid]);
	#else
	M3ActPdoV3Cmd * g = &(gains.command[chid]);
	M3ActPdoV3Cmd * d = &(ec_cmd.command[chid]);
	#endif

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
	
	#if defined MAX2_BDC_0_2_A2R3 || defined MAX2_BLDC_0_2_A2R3 || defined MAX2_BDC_0_2_T2R3 || defined MAX2_BLDC_0_2_T2R3
	M3ActPdoV1Cmd * g = &(gains.command[chid]);
	M3ActPdoV1Cmd * d = &(ec_cmd.command[chid]);
	#else
	M3ActPdoV3Cmd * g = &(gains.command[chid]);
	M3ActPdoV3Cmd * d = &(ec_cmd.command[chid]);
	#endif
	
	
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
	#if defined MAX2_BDC_0_2_A2R3 || defined MAX2_BLDC_0_2_A2R3 || defined MAX2_BDC_0_2_T2R3 || defined MAX2_BLDC_0_2_T2R3
	M3ActPdoV1Cmd * g = &(gains.command[chid]);
	#else
	M3ActPdoV3Cmd * g = &(gains.command[chid]);
	#endif
	
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
	memset(torque_delta[chid],0,NUM_TORQUE_DOT_SAMPLES*sizeof(int));
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
	set_pwm(chid,des);
}


void step_control()
{
	int chid;

	for ( chid=0;chid<NUM_CTRL_CH;chid++)
	{
		#ifdef USE_WATCHDOG
		if (watchdog_expired)
			fsa_state[chid]=CTRL_OFF;
		#endif
	
		int mode = (ec_cmd.command[chid].mode & 0x00FF);	//This mask is needed when M3 is using the old watchdog, can be removed later
		int des = ec_cmd.command[chid].t_desire;
		
		#ifdef USE_BRAKE
		step_brake(ec_cmd.command[chid].config&M3ACT_CONFIG_BRAKE_OFF);
		#endif
	
		switch (fsa_state[chid])
		{
			case CTRL_OFF:
				step_amp_out(chid,0);
				if (mode==MODE_PWM)
					fsa_state[chid]=CTRL_PWM;
				if (mode==MODE_PID)
				{
					fsa_state[chid]=CTRL_OFF_TO_PID;
					setup_pid(chid);
				}
				if (mode==MODE_CURRENT)
				{
					fsa_state[chid]=CTRL_CURRENT;
					setup_pid(chid);
				}
				break;
				
			case CTRL_PWM:
				step_amp_out(chid,des);
				if (mode!=MODE_PWM)
				{
					fsa_state[chid]=CTRL_OFF;
				}
				break;
	
			case CTRL_OFF_TO_PID:
				if (mode!=MODE_PID)
				{
					fsa_state[chid]=CTRL_OFF;
					break;
				}
				if (ramp_pid_gains_up(chid,RAMP_UPDATE_RATE))
						fsa_state[chid]=CTRL_PID;
				step_torque_pid(chid,des);
				break;
	
			case CTRL_PID:
				if (mode!=MODE_PID )
				{
					fsa_state[chid]=CTRL_PID_TO_OFF;
					break;
				}
				ramp_pid_gains_up(chid,RAMP_UPDATE_RATE);
				step_torque_pid(chid,des);
				break;
		
			case CTRL_PID_TO_OFF:
				if (ramp_pid_gains_down(chid,RAMP_UPDATE_RATE))
				{
					fsa_state[chid]=CTRL_OFF;
				}
				step_amp_out(chid,0);
				break;								//BUG Double break...
				
				#ifdef EC_USE_WATCHDOG			
				if (!ec_wd_expired)
					fsa_state[chid]=CTRL_OFF;
				#endif
				break;								//BUG Double break...
				
			case CTRL_CURRENT:
				if (mode!=MODE_CURRENT)
				{
					//ToDo: Should we provide exit-states like for the PID?
					fsa_state[chid] = CTRL_OFF;
					break;
				}
				/* Disabled for now, we send 0 PWM
				ramp_pid_gains_up(chid,RAMP_UPDATE_RATE);
				step_current_pid(chid,des);
				*/
				step_amp_out(chid,0);	//Remove when you enable current control
				break;
				
			default:
				step_amp_out(chid,0);
				break;
		};
	}
}

void step_torque_pid(int chid,int des)
{
	#if defined MAX2_BDC_0_2_A2R3 || defined MAX2_BLDC_0_2_A2R3 || defined MAX2_BDC_0_2_T2R3 || defined MAX2_BLDC_0_2_T2R3
	M3ActPdoV1Cmd * g = &(gains.command[chid]);
	M3ActPdoV1Cmd * d = &(ec_cmd.command[chid]);
	#else
	M3ActPdoV3Cmd * g = &(gains.command[chid]);
	M3ActPdoV3Cmd * d = &(ec_cmd.command[chid]);
	#endif
	
	int ddes = CLAMP(des,d->t_min,d->t_max);
	int s=0;

	#if defined USE_ENCODER_VERTX && defined VERTX_CH_SEAS
	if (d->config&M3ACT_CONFIG_TORQUE_SMOOTH)
		s=get_avg_vertx(VERTX_CH_SEAS);
	else
		s=vertx_pos(VERTX_CH_SEAS);
	#endif
	
	#if defined MAX2_BDC_0_3_T2R2 || defined MAX2_BLDC_0_3_T2R2
	if (d->config&M3ACT_CONFIG_TORQUE_SMOOTH)
		s=get_avg_adc_torque();
	else
		s=adc_raw[ADC_EXT];
	#endif
	
	#if defined MAX2_BDC_0_2_T2R3 || defined MAX2_BLDC_0_2_T2R3
	if (d->config&M3ACT_CONFIG_TORQUE_SMOOTH)
		s=get_avg_adc_spi(0);
	else
		s=get_adc_spi(0);
	#endif


	t_error = (ddes-s);
	
	//Proportional term
	p_term[chid] = ((long)((long)g->k_p * (t_error>>g->k_p_shift)));//(t_error&(~mask))))>>gains.k_p_shift;   
	
	//Integral term
	t_error_sum[chid] += t_error;
	t_error_sum[chid]=CLAMP(t_error_sum[chid], -g->k_i_limit, g->k_i_limit);
	i_term[chid] = g->k_i * (t_error_sum[chid]>>g->k_i_shift);

	//Derivative term
	//Note: should actually normalize for time, but discretization is an issue...
	int tdi=td_idx[chid];
	torque_dot[chid]= torque_dot[chid] - torque_delta[chid][tdi];
	torque_delta[chid][tdi] = s-torque_prev[chid];
	torque_prev[chid] = s;
	torque_dot[chid]=torque_dot[chid]+torque_delta[chid][tdi];
	td_idx[chid]=INC_MOD(tdi,NUM_TORQUE_DOT_SAMPLES);
	d_term[chid] = ((long)(g->k_d * torque_dot[chid]))>>g->k_d_shift;  
	//ec_debug[chid]=(int)d_term[chid];

	//ec_debug[chid]=d_term[chid];
	//Feedforward term
	ff_term[chid]=0;
	//Newer boards get FeedForward term from host
	if (d->config&M3ACT_CONFIG_TORQUE_FF)
		ff_term[chid] = g->k_ff;
	#if !(defined MAX2_BDC_0_2_T2R3 || defined MAX2_BLDC_0_2_T2R3 || defined MAX2_BDC_0_2_A2R3 \
	|| defined MAX2_BLDC_0_2_A2R3)
	else
		ff_term[chid] = g->k_ff*((ddes-g->k_ff_zero)>>g->k_ff_shift);
	#endif
	
	result=p_term[chid]+i_term[chid]+d_term[chid]+ff_term[chid];
	result=CLAMP(result,-PWM_MAX_DUTY,PWM_MAX_DUTY);
	step_amp_out(chid,(int)result);
}

//Current control
//Note: same gains and same code as the torque PID
//Note: this function is unipolar (PWM saturation from 0 to +MAX)
//		the sign is added at the end
void step_current_pid(int chid, int des)
{
	#ifdef USE_CURRENT
/* Not used for the moment
	
	#if defined MAX2_BDC_0_2_A2R3 || defined MAX2_BDC_0_2_T2R3 || defined MAX2_BLDC_0_2_T2R3
	M3ActPdoV1Cmd * g = &(gains.command[chid]);
	M3ActPdoV1Cmd * d = &(ec_cmd.command[chid]);
	#else
	M3ActPdoV3Cmd * g = &(gains.command[chid]);
	M3ActPdoV3Cmd * d = &(ec_cmd.command[chid]);
	#endif
	
	int ddes = ABS(CLAMP(des,d->t_min,d->t_max));
	int s=0;
	int sign = SIGN(des);

	s = get_current_ma();
	t_error = (ddes-s);
	
	//Proportional term
	p_term[chid] = ((long)((long)g->k_p * (t_error>>g->k_p_shift)));//(t_error&(~mask))))>>gains.k_p_shift;   
	
	//Integral term
	t_error_sum[chid] += t_error;
	t_error_sum[chid]=CLAMP(t_error_sum[chid], -g->k_i_limit, g->k_i_limit);
	i_term[chid] = g->k_i * (t_error_sum[chid]>>g->k_i_shift);

	//Derivative term
	//Note: should actually normalize for time, but discretization is an issue...
	int tdi=td_idx[chid];
	torque_dot[chid]= torque_dot[chid] - torque_delta[chid][tdi];
	torque_delta[chid][tdi] = s-torque_prev[chid];
	torque_prev[chid] = s;
	torque_dot[chid]=torque_dot[chid]+torque_delta[chid][tdi];
	td_idx[chid]=INC_MOD(tdi,NUM_TORQUE_DOT_SAMPLES);
	d_term[chid] = ((long)(g->k_d * torque_dot[chid]))>>g->k_d_shift;  
	//ec_debug[chid]=(int)d_term[chid];

	//ec_debug[chid]=d_term[chid];
	//Feedforward term
	ff_term[chid]=0;
	//Newer boards get FeedForward term from host
	if (d->config&M3ACT_CONFIG_TORQUE_FF)
	  ff_term[chid] = g->k_ff;

	result=p_term[chid]+i_term[chid]+d_term[chid]+ff_term[chid];
	result=CLAMP(result,0,PWM_MAX_DUTY);	//Modified to 0 - Unipolar
	step_amp_out(chid,(int)result*sign);	
	
*/
	#endif
}

#endif //USE_CONTROL
