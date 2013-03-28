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
/////////////////////////////////////////////////////////////////////////
//3ch controller maxing out memory. If we disable D term can squeeze under the line
//, which is OK since not using for hand
//However, this used to work OK for H2R2. Something changed in total amount of memory 

int  torque_delta[NUM_CTRL_CH][NUM_TORQUE_DOT_SAMPLES];

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
#if defined M3_DAC_0_1 || defined M3_ELMO_B1R1
//Honor limitswitch
 if (ec_cmd.command[chid].config & M3ACT_CONFIG_LIMITSWITCH_STOP_POS && des>0 && limit_switch_pos_flag() ||
     ec_cmd.command[chid].config & M3ACT_CONFIG_LIMITSWITCH_STOP_NEG && des<0 && limit_switch_neg_flag())
    set_dac(0);
 else
    set_dac(des);
#endif

#if  defined M3_ELMO_RNA_R0 
    set_dac(des);
#endif

#if defined M3_ELMO_Z1R1
//Force brake on whenever hit aux switch
//Force to stall if hit aux switch or moving down and hit limit switch
//Thus to un-stall, aux switch must be un-pressed or command must move in other direction

//Override
if (aux_switch_flag() && !limit_switch_neg_flag())
{
	set_dac(0);
	force_brake_on();
	ec_debug[0]=1;
}
else
{
	reset_force_brake_on();
	if (ec_cmd.command[chid].config & M3ACT_CONFIG_LIMITSWITCH_STOP_NEG && des<0 && limit_switch_neg_flag())
	{
		set_dac(0);
	}
	else
	{
		set_dac(des);
		ec_debug[0]=-1;
	}
}

#endif

}




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

#ifdef USE_BRAKE
	step_brake(ec_cmd.command[chid].config&M3ACT_CONFIG_BRAKE_OFF);
#endif


	switch (fsa_state[chid])
	{
		case CTRL_OFF:
#if defined M3_DAC_0_1 || defined M3_ELMO_RNA_R0 || defined M3_ELMO_B1R1 || defined M3_ELMO_Z1R1
			ClrEnableAmp;
#endif
			step_amp_out(chid,0);
			if (mode==MODE_PWM)
				fsa_state[chid]=CTRL_PWM;
			if (mode==MODE_PID)
				fsa_state[chid]=CTRL_OFF_TO_PID;
				setup_pid(chid);
			break;

		case CTRL_PWM:
#if defined M3_DAC_0_1 || defined M3_ELMO_RNA_R0 || defined M3_ELMO_B1R1 || defined M3_ELMO_Z1R1
			SetEnableAmp;
#endif
			step_amp_out(chid,des);
			if (mode!=MODE_PWM)
				fsa_state[chid]=CTRL_OFF;
			break;

		case CTRL_OFF_TO_PID:
#if defined M3_DAC_0_1 || defined M3_ELMO_RNA_R0 || defined M3_ELMO_B1R1 || defined M3_ELMO_Z1R1
			SetEnableAmp;
#endif
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
#if defined M3_DAC_0_1 || defined M3_ELMO_RNA_R0 || defined M3_ELMO_B1R1 || defined M3_ELMO_Z1R1
			SetEnableAmp;
#endif
			if (mode!=MODE_PID )
			{
				fsa_state[chid]=CTRL_PID_TO_OFF;
				break;
			}
			ramp_pid_gains_up(chid,RAMP_UPDATE_RATE);
			step_ctrl_pid(chid,des);
			break;

		case CTRL_PID_TO_OFF:
#if defined M3_DAC_0_1 || defined M3_ELMO_RNA_R0 || defined M3_ELMO_B1R1 || defined M3_ELMO_Z1R1
			ClrEnableAmp;
#endif
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
#if defined M3_DAC_0_1 || defined M3_ELMO_RNA_R0 || defined M3_ELMO_B1R1 || defined M3_ELMO_Z1R1
			ClrEnableAmp;
#endif
		step_amp_out(chid,0);
		break;
	};
}
}



void step_ctrl_pid(int chid,int des)
{


# if defined M3_ELMO_B1R1 || defined M3_ELMO_Z1R1 
    step_amp_out(chid,des); //des should be dac ticks already, ff_current_ctrl
    return;
#endif
}


#endif //USE_CONTROL
