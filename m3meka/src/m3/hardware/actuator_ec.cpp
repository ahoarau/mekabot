/* 
M3 -- Meka Robotics Robot Components
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

#include "actuator_ec.h"
#include "m3rt/base/component_factory.h"

namespace m3{
	
using namespace m3rt;
using namespace std;

#define PWR_SLEW_TIME 4.0 //Seconds to ramp setpoint in after power up
#define PWM_SLEW_TIME 4.0 //Seconds to ramp setpoint in after switch to pwm mode

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
M3BaseStatus * M3ActuatorEc::GetBaseStatus()
{
	return status.mutable_base();
}

void M3ActuatorEc::ResetCommandPdo(unsigned char * pdo)
{
	//V0
	if (IsPdoVersion(SEA_PDO_V0))
	{
		M3SeaPdoV0Cmd * p = (M3SeaPdoV0Cmd *) pdo;
		memset(p,0,sizeof(M3SeaPdoV0Cmd));	
		return;
	}
	//V1
	if (IsPdoVersion(ACTX1_PDO_V1) || IsPdoVersion(ACTX1_PDO_V3)) //V3 cmd same as V1
	{
		M3ActX1PdoV1Cmd * p = (M3ActX1PdoV1Cmd *) pdo;
		memset(p,0,sizeof(M3ActX1PdoV1Cmd));	
		return;
	}
	if (IsPdoVersion(ACTX2_PDO_V1)|| IsPdoVersion(TACTX2_PDO_V1))
	{
		M3ActX2PdoV1Cmd * ec = (M3ActX2PdoV1Cmd *) pdo;
		M3ActPdoV1Cmd * p=&(ec->command[chid]);
		memset(p,0,sizeof(M3ActPdoV1Cmd));	
		return;
	}
	if (IsPdoVersion(ACTX3_PDO_V1))
	{
		M3ActX3PdoV1Cmd * ec = (M3ActX3PdoV1Cmd *) pdo;
		M3ActPdoV1Cmd * p=&(ec->command[chid]);
		memset(p,0,sizeof(M3ActPdoV1Cmd));	
		return;
	}
	if (IsPdoVersion(ACTX4_PDO_V1))
	{
		M3ActX4PdoV1Cmd * ec = (M3ActX4PdoV1Cmd *) pdo;
		M3ActPdoV1Cmd * p=&(ec->command[chid]);
		memset(p,0,sizeof(M3ActPdoV1Cmd));	
		return;
	}
	
	
	//V2
	if (IsPdoVersion(ACTX1_PDO_V2))
	{
		M3ActX1PdoV2Cmd * p = (M3ActX1PdoV2Cmd *) pdo;
		memset(p,0,sizeof(M3ActX1PdoV2Cmd));	
		return;
	}
	if (IsPdoVersion(ACTX2_PDO_V2))
	{
		M3ActX2PdoV2Cmd * ec = (M3ActX2PdoV2Cmd *) pdo;
		M3ActPdoV2Cmd * p=&(ec->command[chid]);
		memset(p,0,sizeof(M3ActPdoV2Cmd));	
		return;
	}
	if (IsPdoVersion(ACTX3_PDO_V2))
	{
		M3ActX3PdoV2Cmd * ec = (M3ActX3PdoV2Cmd *) pdo;
		M3ActPdoV2Cmd * p=&(ec->command[chid]);
		memset(p,0,sizeof(M3ActPdoV2Cmd));	
		return;
	}
	if (IsPdoVersion(ACTX4_PDO_V2))
	{
		M3ActX4PdoV2Cmd * ec = (M3ActX4PdoV2Cmd *) pdo;
		M3ActPdoV2Cmd * p=&(ec->command[chid]);
		memset(p,0,sizeof(M3ActPdoV2Cmd));	
		return;
	}
	
	//V4
	if (IsPdoVersion(ACTX1_PDO_V4)) 
	{
		M3ActX1PdoV4Cmd * p = (M3ActX1PdoV4Cmd *) pdo;
		memset(p,0,sizeof(M3ActX1PdoV4Cmd));	
		return;
	}
	if (IsPdoVersion(ACTX2_PDO_V4))
	{
		M3ActX2PdoV4Cmd * ec = (M3ActX2PdoV4Cmd *) pdo;
		M3ActPdoV4Cmd * p=&(ec->command[chid]);
		memset(p,0,sizeof(M3ActPdoV4Cmd));	
		return;
	}
	if (IsPdoVersion(ACTX3_PDO_V4))
	{
		M3ActX3PdoV4Cmd * ec = (M3ActX3PdoV4Cmd *) pdo;
		M3ActPdoV4Cmd * p=&(ec->command[chid]);
		memset(p,0,sizeof(M3ActPdoV4Cmd));	
		return;
	}
	
}

void M3ActuatorEc::SetStatusFromPdoV0(unsigned char * data)
{
    M3SeaPdoV0Status * ax;
    
    if (IsPdoVersion(SEA_PDO_V0))
    {
	ax = (M3SeaPdoV0Status *) data;
	status.set_timestamp(GetBaseStatus()->timestamp());
	status.set_qei_period(ax->qei_period);
	status.set_qei_on(ax->qei_on);
	status.set_qei_rollover(0);//not supported
	status.set_debug(ax->debug);
	status.set_adc_torque(ax->adc_torque);
	status.set_adc_ext_temp(ax->adc_motor_temp); //Post RBL version renamed PDO motor_temp to ext_temp
	status.set_adc_ext_a(ax->adc_ext);
	status.set_adc_ext_b(0);//not supported
	status.set_adc_amp_temp(ax->adc_amp_temp);
	status.set_adc_current_a(ax->adc_current_a);
	status.set_adc_current_b(ax->adc_current_b);
	status.set_pwm_cmd(ax->pwm_cmd);	
	status.set_flags(ax->flags);
	status.set_current_ma(0);
    }
}

void M3ActuatorEc::SetStatusFromPdoV1(unsigned char * data)
{
    M3ActPdoV1Status * ax;
    if (IsPdoVersion(ACTX1_PDO_V1))
    {
	    M3ActX1PdoV1Status * ec = (M3ActX1PdoV1Status *) data;
	    status.set_timestamp(ec->timestamp);
	    ax=&(ec->status[chid]);
    }
    
    if (IsPdoVersion(ACTX2_PDO_V1))
    {
	    M3ActX2PdoV1Status * ec = (M3ActX2PdoV1Status *) data;
	    status.set_timestamp(ec->timestamp);
	    ax=&(ec->status[chid]);
    }
    if (IsPdoVersion(TACTX2_PDO_V1))
    {
	    M3TactX2PdoV1Status * ec = (M3TactX2PdoV1Status *) data;
	    status.set_timestamp(ec->timestamp);
	    ax=&(ec->status[chid]);
    }
    if (IsPdoVersion(ACTX3_PDO_V1))
    {
	    M3ActX3PdoV1Status * ec = (M3ActX3PdoV1Status *) data;
	    status.set_timestamp(ec->timestamp);
	    ax=&(ec->status[chid]);
    }
    if (IsPdoVersion(ACTX4_PDO_V1))
    {
	    M3ActX4PdoV1Status * ec = (M3ActX4PdoV1Status *) data;
	    status.set_timestamp(ec->timestamp);
	    ax=&(ec->status[chid]);
    }
    status.set_qei_period(ax->qei_period);
    status.set_qei_on(ax->qei_on);
    status.set_qei_rollover(ax->qei_rollover);
    status.set_debug(ax->debug);
    status.set_adc_torque(ax->adc_torque);
    status.set_adc_ext_temp(ax->adc_motor_temp); //Post RBL version renamed PDO motor_temp to ext_temp
    status.set_adc_ext_a(ax->adc_ext_a);
    status.set_adc_ext_b(ax->adc_ext_b);
    status.set_adc_amp_temp(ax->adc_amp_temp);
    status.set_adc_current_a(ax->adc_current_a);
    status.set_adc_current_b(ax->adc_current_b);
    status.set_pwm_cmd(ax->pwm_cmd);	
    status.set_flags(ax->flags);
}

	
void M3ActuatorEc::SetStatusFromPdoV2(unsigned char * data)
{
      M3ActPdoV2Status * ax;
      if (IsPdoVersion(ACTX1_PDO_V2))
      {
	      M3ActX1PdoV2Status * ec = (M3ActX1PdoV2Status *) data;
	      status.set_timestamp(ec->timestamp);
	      ax=&(ec->status[chid]);
      }
      if (IsPdoVersion(ACTX2_PDO_V2))
      {
	      M3ActX2PdoV2Status * ec = (M3ActX2PdoV2Status *) data;
	      status.set_timestamp(ec->timestamp);
	      ax=&(ec->status[chid]);
      }
      if (IsPdoVersion(ACTX3_PDO_V2))
      {
	      M3ActX3PdoV2Status * ec = (M3ActX3PdoV2Status *) data;
	      status.set_timestamp(ec->timestamp);
	      ax=&(ec->status[chid]);
      }
      if (IsPdoVersion(ACTX4_PDO_V2))
      {
	      M3ActX4PdoV2Status * ec = (M3ActX4PdoV2Status *) data;
	      status.set_timestamp(ec->timestamp);
	      ax=&(ec->status[chid]);
      }
      status.set_qei_period(ax->qei_period);
      status.set_qei_on(ax->qei_on);
      status.set_qei_rollover(ax->qei_rollover);
      status.set_adc_torque(ax->adc_torque);
      status.set_pwm_cmd(ax->pwm_cmd);	
      status.set_flags(ax->flags);
      
      //Pass in EXT data
      int ext_sz = sizeof(M3ActPdoV2StatusExt);
      int num_copy=MIN(M3ACT_PDO_V2_STATUS_EXT_SZ,ext_sz); //max num to copy 
      int start = ax->ext_start_idx; 
      int end = (start+num_copy-1)%ext_sz; 
      if (end<start) //handle roll-over
      {
	int nc1=ext_sz-start; 
	int nc2=num_copy-nc1; 
	memcpy(((unsigned char *)&exs)+start,(unsigned char *) ax->ext,nc1);
	memcpy((unsigned char *)&exs,(unsigned char *) ax->ext+nc1,nc2);
      }
      else
	memcpy((unsigned char *)&exs+start,(unsigned char *) ax->ext,num_copy);
     
      status.set_debug(exs.debug);
      status.set_adc_ext_temp(exs.adc_motor_temp);
      status.set_adc_ext_a(exs.adc_ext_a);
      status.set_adc_ext_b(exs.adc_ext_b);
      status.set_adc_amp_temp(exs.adc_amp_temp);
      status.set_adc_current_a(exs.adc_current_a);
      status.set_adc_current_b(exs.adc_current_b);
}

void M3ActuatorEc::SetStatusFromPdoV3(unsigned char * data)
{
    M3ActPdoV3Status * ax;
    M3ActX1PdoV3Status * ec = (M3ActX1PdoV3Status *) data;
    status.set_timestamp(ec->timestamp);
    ax=&(ec->status[chid]);
    status.set_qei_period(ax->qei_period);
    status.set_qei_on(ax->qei_on);
    status.set_qei_rollover(ax->qei_rollover);
    status.set_debug(ax->debug);
    status.set_adc_torque(ax->adc_torque);
    status.set_adc_ext_temp(ax->adc_motor_temp); //Post RBL version renamed PDO motor_temp to ext_temp
    status.set_adc_ext_a(0);
    status.set_adc_ext_b(0);
    status.set_adc_amp_temp(ax->adc_amp_temp);
    status.set_adc_current_a(ax->adc_current_a);
    status.set_adc_current_b(ax->adc_current_b);
    status.set_pwm_cmd(ax->pwm_cmd);	
    status.set_flags(ax->flags);
    status.set_current_ma(ax->current_ma);
}

void M3ActuatorEc::SetStatusFromPdoV4(unsigned char * data)
{
    M3ActPdoV4Status * ax;
    if (IsPdoVersion(ACTX1_PDO_V4))
    {
	    M3ActX1PdoV4Status * ec = (M3ActX1PdoV4Status *) data;
	    status.set_timestamp(ec->timestamp);
	    ax=&(ec->status[chid]);
    }
    if (IsPdoVersion(ACTX2_PDO_V4))
    {
	    M3ActX2PdoV4Status * ec = (M3ActX2PdoV4Status *) data;
	    status.set_timestamp(ec->timestamp);
	    ax=&(ec->status[chid]);
    }
    if (IsPdoVersion(ACTX3_PDO_V4))
    {
	    M3ActX3PdoV4Status * ec = (M3ActX3PdoV4Status *) data;
	    status.set_timestamp(ec->timestamp);
	    ax=&(ec->status[chid]);
    }
    
    status.set_current_ma(ax->current_ma);
    status.set_debug(ax->debug);
    
    //tq_old = status.adc_torque();
    int tq_diff = ABS(ax->adc_torque - status.adc_torque());
    
    if (tq_diff > 40)
    {
      tq_err_cnt++;
    }
    else
    {
      status.set_adc_torque(ax->adc_torque);
      tq_err_cnt = 0;
    }
    
    if (tq_err_cnt > 2)
    {
      tq_err_cnt = 0;
      status.set_adc_torque(ax->adc_torque);
    }
    
    if (ABS(ax->qei_on - status.qei_on()) > 50)
    {
      qei_err_cnt++;
    }
    else
    {
      status.set_qei_on(ax->qei_on);
      qei_err_cnt = 0;
    }
    
    if (qei_err_cnt > 2)
    {
      qei_err_cnt = 0;
      status.set_qei_on(ax->qei_on);
    }
    
    //status.set_adc_torque(ax->adc_torque);
    //status.set_qei_on(ax->qei_on);  
      
    status.set_torque_err_cnt(ax->torque_err_cnt);
    if (override_ext_temp)
      status.set_adc_ext_temp(((M3ActuatorEcStatus*)override_ext_temp_act_ec->GetStatus())->adc_ext_temp()); 
    else
      status.set_adc_ext_temp(ax->adc_ext_temp); 
    status.set_adc_amp_temp(ax->adc_amp_temp);
    status.set_adc_ext_a(0);
    status.set_adc_ext_b(0);
    status.set_adc_current_a(ax->adc_current_a);
    status.set_adc_current_b(ax->adc_current_b);
    status.set_pwm_cmd(ax->pwm_cmd);	
    
    status.set_qei_period(ax->qei_period);
    status.set_qei_rollover(ax->qei_rollover);
    status.set_qei_err_cnt(ax->qei_err_cnt);
    status.set_flags(ax->flags);
	status.set_motor_pos(ax->motor_pos);
	
    
}



void M3ActuatorEc::SetStatusFromPdo(unsigned char * data)
{
  if (IsPdoVersion(SEA_PDO_V0))
	SetStatusFromPdoV0(data);
  
    if (IsPdoVersion(ACTX1_PDO_V1) || 
	IsPdoVersion(ACTX2_PDO_V1) || 
	IsPdoVersion(ACTX3_PDO_V1) || 
	IsPdoVersion(ACTX4_PDO_V1) || 
	IsPdoVersion(TACTX2_PDO_V1))
	SetStatusFromPdoV1(data);
    
    if (IsPdoVersion(ACTX1_PDO_V2) || 
	IsPdoVersion(ACTX2_PDO_V2) || 
	IsPdoVersion(ACTX3_PDO_V2) || 
	IsPdoVersion(ACTX4_PDO_V2))
	SetStatusFromPdoV2(data);
    
    if (IsPdoVersion(ACTX1_PDO_V3))
	SetStatusFromPdoV3(data);
    
    if (IsPdoVersion(ACTX1_PDO_V1) || 
	IsPdoVersion(ACTX2_PDO_V1) || 
	IsPdoVersion(ACTX3_PDO_V1))
	SetStatusFromPdoV1(data);
	
	if (IsPdoVersion(ACTX1_PDO_V4))
		SetStatusFromPdoV4(data);
	
}


void  M3ActuatorEc::StepStatus()
{
  M3ComponentEc::StepStatus();
  
  //if (IsPdoVersion(ACTX1_PDO_V3))
  //{
    
    if (IsCurrentFaultCont())
    {      
      if (!error_printed)
      {
	SetStateError();
	M3_ERR("Amp Continuous OverCurrent Fault (Last instaneous: %d (mA)) for  %s\n",status.current_ma(),GetName().c_str());      
	error_printed = true;
      }
      
    }
    if (IsCurrentFaultMom())
    {      
      if (!error_printed)
      {
	SetStateError();
	M3_ERR("Amp Momentary OverCurrent Fault (Last instaneous: %d (mA)) for  %s\n",status.current_ma(),GetName().c_str());      
	error_printed = true;
      }
    }
    status.set_motor_power_slewed_on(motor_power_slewed_on);
  //}
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void M3ActuatorEc::SetPdoV0FromPdoV1Command(unsigned char * data)
{
  M3SeaPdoV0Cmd* ec = (M3SeaPdoV0Cmd *) data;
  ec->config=acc.config;
  ec->k_p=acc.k_p;
  ec->k_i=acc.k_i;
  ec->k_d=acc.k_d;
  ec->k_p_shift=acc.k_p_shift;
  ec->k_i_shift=acc.k_i_shift;
  ec->k_d_shift=acc.k_d_shift;
  ec->k_i_limit=acc.k_i_limit;
  ec->t_desire=acc.t_desire;
  ec->t_max=acc.t_max;
  ec->t_min=acc.t_min;
  ec->mode=acc.mode;
  ec->pwm_max=acc.pwm_max;
}

void M3ActuatorEc::SetPdoV4FromPdoV1Command(unsigned char * data)
{
    M3ActPdoV4Cmd * ax;
    if (IsPdoVersion(ACTX1_PDO_V4))
    {
	    M3ActX1PdoV4Cmd* ec = (M3ActX1PdoV4Cmd *) data;
	    ax=&(ec->command[chid]);
    }
    if (IsPdoVersion(ACTX2_PDO_V4))
    {
	    M3ActX2PdoV4Cmd* ec = (M3ActX2PdoV4Cmd *) data;
	    ax=&(ec->command[chid]);
    }
    if (IsPdoVersion(ACTX3_PDO_V4))
    {
	    M3ActX3PdoV4Cmd* ec = (M3ActX3PdoV4Cmd *) data;
	    ax=&(ec->command[chid]);
    }
    if (has_brake)
	    acc.config |= ACTUATOR_EC_CONFIG_HAS_BRAKE;
    ax->config=acc.config;
    ax->k_p=acc.k_p;
    ax->k_i=acc.k_i;
    ax->k_d=acc.k_d;
    ax->k_p_shift=acc.k_p_shift;
    ax->k_i_shift=acc.k_i_shift;
    ax->k_d_shift=acc.k_d_shift;
    ax->k_i_limit=acc.k_i_limit;
    ax->t_desire=acc.t_desire;
    ax->mode=acc.mode;
    ax->qei_max=acc.qei_max;
    ax->qei_min=acc.qei_min;
    ax->pwm_max=acc.pwm_max;
    
    ax->current_desired = CLAMP(command.current_desired(),-32767,32767);
	ax->pwm_desired = CLAMP(command.pwm_desired(),-32767,32767);
	ax->bldc_mode = param.bldc_mode();
    /*if (tmp_cnt++ == 100)
    {
      M3_DEBUG("pwm: %d\n", pwm_max_ext);
      tmp_cnt = 0;
    }*/
    //ax->pwm_desired = acc.pwm_desired;
    //ax->current_desired = acc.current_desired;
}


void M3ActuatorEc::SetPdoV2FromPdoV1Command(unsigned char * data)
{
    //Copy V1 into V2 structures
    axc.config=acc.config;
    axc.t_desire=acc.t_desire;
    axc.mode=acc.mode;
    exc.k_p=acc.k_p;
    exc.k_i=acc.k_i;
    exc.k_d=acc.k_d;
    exc.k_p_shift=acc.k_p_shift;
    exc.k_i_shift=acc.k_i_shift;
    exc.k_d_shift=acc.k_d_shift;
    exc.k_i_limit=acc.k_i_limit;
    exc.t_max=acc.t_max;
    exc.t_min=acc.t_min;
    exc.pwm_max=acc.pwm_max;
    exc.qei_max=acc.qei_max;
    exc.qei_min=acc.qei_min;
    exc.k_ff_zero=acc.k_ff_zero;
    exc.k_ff_shift=acc.k_ff_shift;
    exc.k_ff=acc.k_ff;
    exc.pwm_db=acc.pwm_db;
    //if (tmp_cnt++%100==0)
    //  M3_INFO("Config: %d\n",(int)axc.config);
    //Copy portion of EXT struct to ext buffer
    int ext_sz = sizeof(M3ActPdoV2CmdExt);
    int num_copy=MIN(M3ACT_PDO_V2_CMD_EXT_SZ,ext_sz);
    int start = axc.ext_start_idx;
    int end=(start+num_copy-1)%ext_sz;
    if (end<start) //rollover
    {
      int nc1=ext_sz-start; 
      int nc2=num_copy-nc1; 
      memcpy((unsigned char *)&(axc.ext),((unsigned char *) &exc)+start,nc1);
      memcpy(((unsigned char *)&(axc.ext))+nc1,(unsigned char *)(&exc),nc2);
    }
    else
      memcpy((unsigned char *)&(axc.ext),((unsigned char *) &exc)+start,num_copy);
      
   
   //Now copy axc to the output PDO
    M3ActPdoV2Cmd * ax;
    if (IsPdoVersion(ACTX1_PDO_V2))
    {
	    M3ActX1PdoV2Cmd* ec = (M3ActX1PdoV2Cmd *) data;
	    ax=&(ec->command[chid]);
    }
    if (IsPdoVersion(ACTX2_PDO_V2))
    {
	    M3ActX2PdoV2Cmd* ec = (M3ActX2PdoV2Cmd *) data;
	    ax=&(ec->command[chid]);
    }
    if (IsPdoVersion(ACTX3_PDO_V2))
    {
	    M3ActX3PdoV2Cmd* ec = (M3ActX3PdoV2Cmd *) data;
	    ax=&(ec->command[chid]);
    }
    if (IsPdoVersion(ACTX4_PDO_V2))
    {
	    M3ActX4PdoV2Cmd* ec = (M3ActX4PdoV2Cmd *) data;
	    ax=&(ec->command[chid]);
    }
    memcpy((unsigned char *)ax,(unsigned char *)&axc,sizeof(M3ActPdoV2Cmd));
    //Finally advance the copy pointer
    axc.ext_start_idx=(axc.ext_start_idx+num_copy)%ext_sz; 
}


void M3ActuatorEc::SetPdoFromCommand(unsigned char * data)
{
      mReal current_in = (mReal)command.current_desired();
      mReal pwm_in = (mReal)command.pwm_desired();
      mReal mode_in = (mReal)command.mode();
      if (tmp_cnt == 200)
	{
  //M3_DEBUG("I_des_in: %f \n",current_in);
	}
//       
      
      
	M3ActPdoV1Cmd * ax;
	if (IsPdoVersion(ACTX1_PDO_V2) || 
	    IsPdoVersion(ACTX2_PDO_V2) || 
	    IsPdoVersion(ACTX3_PDO_V2) || 
	    IsPdoVersion(ACTX4_PDO_V2) ||
	    IsPdoVersion(ACTX1_PDO_V4) || 
	    IsPdoVersion(ACTX2_PDO_V4) || 
	    IsPdoVersion(ACTX3_PDO_V4) || 
	    IsPdoVersion(SEA_PDO_V0))
	  ax=&acc; //Fill in this temporary V1 PDO,then transfer to V0/V2 PDO
	  
	if (IsPdoVersion(ACTX1_PDO_V1) || IsPdoVersion(ACTX1_PDO_V3))
	{
		M3ActX1PdoV1Cmd* ec = (M3ActX1PdoV1Cmd *) data;
		ax=&(ec->command[chid]);
	}
	if (IsPdoVersion(ACTX2_PDO_V1))
	{
		M3ActX2PdoV1Cmd* ec = (M3ActX2PdoV1Cmd *) data;
		ax=&(ec->command[chid]);
	}
	if (IsPdoVersion(TACTX2_PDO_V1))
	{
		M3TactX2PdoV1Cmd* ec = (M3TactX2PdoV1Cmd *) data;
		ax=&(ec->command[chid]);
	}
	if (IsPdoVersion(ACTX3_PDO_V1))
	{
		M3ActX3PdoV1Cmd* ec = (M3ActX3PdoV1Cmd *) data;
		ax=&(ec->command[chid]);
	}
	if (IsPdoVersion(ACTX4_PDO_V1))
	{
		M3ActX4PdoV1Cmd* ec = (M3ActX4PdoV1Cmd *) data;
		ax=&(ec->command[chid]);
	}

	mReal pwr_scale=0; //Smooth ramp in of current on power-on
	mReal pwm_scale=0; //Smooth ramp in of pwm when switch to from off
	if (ignore_pwm_slew)//Send down pwm command as soon as pwr is on
	{
	    if (pwr==NULL || pwr->IsMotorPowerOn())
		pwr_scale=1.0;
	    else
		pwr_scale=0.0;
	    motor_power_slewed_on = true;
	    pwm_scale=1.0;
	}
	else
	{
		if (pwr==NULL || pwr->IsMotorPowerOn())
			pwr_scale=pwr_slew.Step(1.0,1.0/PWR_SLEW_TIME);
		else
		{
			pwr_slew.Reset(0);
			pwr_scale=0;
		}
		if (command.mode()==ACTUATOR_EC_MODE_PWM)
			pwm_scale=pwm_slew.Step(1.0,1.0/PWM_SLEW_TIME);
		else
		{
			pwm_slew.Reset(0);
			pwm_scale=0;
		}
		if (pwr_scale >= 0.98)
		  motor_power_slewed_on = true;
		else
		  motor_power_slewed_on = false;
	}
	
	
	 
	ax->config=param.config();
	if (command.brake_off() && !command.mode()==ACTUATOR_EC_MODE_OFF && pwr->IsMotorPowerOn())
		ax->config=ax->config | ACTUATOR_EC_CONFIG_BRAKE_OFF; //Pass in as config bit for legacy reasons.
	else
		ax->config=ax->config & ~ACTUATOR_EC_CONFIG_BRAKE_OFF; 
	
	if (IsVersion(IQ))
	{
	    ax->k_p=CLAMP((int)((mReal)param.k_p()),-32767,32767);
	    ax->k_i=CLAMP((int)((mReal)param.k_i()),-32767,32767);
	    ax->k_d=CLAMP((int)((mReal)param.k_d()),-32767,32767);
	    ax->pwm_max=(int)(CLAMP(MIN(pwm_max_ext,param.pwm_max()),0,32767));
	} else {
	    ax->k_p=CLAMP((int)((mReal)param.k_p()*pwr_scale),-32767,32767);
	    ax->k_i=CLAMP((int)((mReal)param.k_i()*pwr_scale),-32767,32767);
	    ax->k_d=CLAMP((int)((mReal)param.k_d()*pwr_scale),-32767,32767);
	    ax->pwm_max=(int)(CLAMP(MIN(pwm_max_ext,param.pwm_max()),0,32767)*pwr_scale);
	    //ax->pwm_max=(int)(CLAMP(MIN(pwm_max_ext,param.pwm_max()),0,32767));
	}
	
	ax->k_p_shift=CLAMP(param.k_p_shift(),-32767,32767);
	ax->k_i_shift=CLAMP(param.k_i_shift(),-32767,32767);
	ax->k_d_shift=CLAMP(param.k_d_shift(),-32767,32767);
	ax->k_i_limit=CLAMP(param.k_i_limit(),-32767,32767);
	ax->pwm_db=CLAMP(param.pwm_db(),-32767,32767);			
	
	ax->t_desire=CLAMP(command.t_desire(),-32767,32767);

	
	if (command.mode()==ACTUATOR_EC_MODE_PWM) //Overwrite
		ax->t_desire=CLAMP((int)((mReal)command.t_desire()*pwr_scale*pwm_scale),-1*ax->pwm_max,ax->pwm_max);

	mReal current_desired = 0.0;
	//if (command.mode()==ACTUATOR_EC_MODE_CURRENT) //Overwrite	  
	//	current_desired = ((mReal)command.current_desired())*pwr_scale;
	mReal current_post_scale = current_desired;
	
	
	if (IsVersion(DEFAULT)||IsVersion(ISS))
	{
	  ax->k_ff_zero=CLAMP(param.k_ff_zero(),-32767,32767);
	  ax->k_ff_shift=CLAMP(param.k_ff_shift(),-32767,32767);	
	  ax->k_ff=CLAMP(param.k_ff(),-32767,32767);	
	}
	if (IsVersion(ESP))
	{
	  ax->k_ff_zero=0;
	  ax->k_ff_shift=0;
	  ax->k_ff=0;
	  if (command.mode()==ACTUATOR_EC_MODE_TORQUE && UseTorqueFF()) //Overwrite
	  {
		  ax->k_ff=(int)CLAMP(pwm_ff,-1*ax->pwm_max,ax->pwm_max);
	  }
	}
	if (IsVersion(IQ)) //Deprecated
	{
	  ax->k_ff_zero=0;
	  ax->k_ff_shift=0;	
	  ax->k_ff=0;	
	}
	
	ax->t_max=CLAMP(param.t_max(),-32767,32767);
	ax->t_min=CLAMP(param.t_min(),-32767,32767);
	
	ax->qei_max=CLAMP(param.qei_max(),-32767,32767);
	ax->qei_min=CLAMP(param.qei_min(),-32767,32767);
	
	// TODO Lee commented this
	//if (pwr_scale==0)
	//	ax->mode=(int)ACTUATOR_EC_MODE_OFF;
	//else
		ax->mode=(int)command.mode();
	
	//toggle bit to give DSP a heartbeat signal
	  if (toggle == 0)
	    toggle = M3ACT_CONFIG_EC_WD;
	  else if (toggle != 0)
	  {
	    ax->config = ax->config | toggle;
	    toggle = 0;
	  }
	
	
	if (IsPdoVersion(ACTX1_PDO_V2) || 
	    IsPdoVersion(ACTX2_PDO_V2) || 
	    IsPdoVersion(ACTX3_PDO_V2) || 
	    IsPdoVersion(ACTX4_PDO_V2))
		SetPdoV2FromPdoV1Command(data);
		
	if (IsPdoVersion(SEA_PDO_V0))
		SetPdoV0FromPdoV1Command(data);
	
	if (IsPdoVersion(ACTX1_PDO_V4) || 
	    IsPdoVersion(ACTX2_PDO_V4) || 
	    IsPdoVersion(ACTX3_PDO_V4))
		SetPdoV4FromPdoV1Command(data);
static int cnt;

	/*if (tmp_cnt++ == 200)
	{
	  cnt++;
	  M3_DEBUG("cnt: %d\n", cnt);
	  //M3_DEBUG("Name: %s\n", GetName().c_str());
	  M3_DEBUG("mode_in: %f\n", mode_in);	  
	  M3_DEBUG("mode_out: %d \n", (int)((M3ActX1PdoV4Cmd *)data)->command[0].mode);	  
	  M3_DEBUG("Pwr_slew: %f\n", pwr_scale);
	  M3_DEBUG("I_des_in: %f \n",current_in);
	  M3_DEBUG("I_des_out: %d\n", ((M3ActX1PdoV4Cmd *)data)->command[0].current_desired);
	  M3_DEBUG("pwm_slew: %f\n", pwm_scale);
	  M3_DEBUG("pwm_des_in: %f \n",pwm_in);
	  M3_DEBUG("pwm_des_out: %d\n", ((M3ActX1PdoV4Cmd *)data)->command[0].pwm_desired);
 	  //  M3_DEBUG("mode_in: %d mode_out: %d pwm_scale %f Pwr slew %f  pwm_in %f pwm_out %d\n",
 		//  (int)mode_in, (int)((M3ActX1PdoV4Cmd *)data)->command[0].mode, pwm_scale,pwr_scale,pwm_in, ((M3ActX1PdoV4Cmd *)data)->command[0].pwm_desired);
	  tmp_cnt = 0;
	}*/
	
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool M3ActuatorEc::LinkDependentComponents()
{
	pwr=(M3Pwr*) factory->GetComponent(pwr_name);
	if (pwr==NULL)
	{
		M3_INFO("M3Pwr component %s not found for component %s. Proceeding without it...\n",pwr_name.c_str(),GetName().c_str());
	}
	
	if (override_ext_temp)
	{
	  override_ext_temp_act_ec = (M3ActuatorEc*) factory->GetComponent(override_ext_temp_act_ec_name);
	  if (override_ext_temp_act_ec==NULL)
	  {
		  M3_INFO("M3ActuatorEc component %s not found for component %s although configured for ext_temp. Proceeding without it...\n", override_ext_temp_act_ec_name.c_str(),GetName().c_str());
		  override_ext_temp = false;
	  }
	}
	
	return true;
}

bool M3ActuatorEc::ReadConfig(const char * filename)
{
	YAML::Node doc;
	if (!M3ComponentEc::ReadConfig(filename))
		return false;
	GetYamlDoc(filename, doc);	
	
	doc["pwr_component"] >> pwr_name;
	doc["chid"] >> chid;
	try
	{
		doc["ignore_pwm_slew"] >>ignore_pwm_slew;
	} catch(YAML::TypedKeyNotFound<string> e) 
	{
		ignore_pwm_slew=0;
	}
	
	const YAML::Node& ymlparam = doc["param"];
	int val;
	if (IsVersion(DEFAULT)||IsVersion(ISS)||IsVersion(ESP))
	{
		ymlparam["k_p"] >> val;
		param.set_k_p(val);
		ymlparam["k_i"] >> val;
		param.set_k_i(val);
		ymlparam["k_d"] >> val;
		param.set_k_d(val);
		ymlparam["k_p_shift"] >> val;
		param.set_k_p_shift(val);
		ymlparam["k_i_shift"] >> val;
		param.set_k_i_shift(val);
		ymlparam["k_d_shift"] >> val;
		param.set_k_d_shift(val);
		ymlparam["k_i_limit"] >> val;
		param.set_k_i_limit(val);
		ymlparam["t_max"] >> val;
		param.set_t_max(val);
		ymlparam["t_min"] >> val;
		param.set_t_min(val);
		ymlparam["qei_max"] >> val;
		param.set_qei_max(val);
		ymlparam["qei_min"] >> val;
		param.set_qei_min(val);
		ymlparam["config"] >> val;
		param.set_config(val);
		ymlparam["pwm_max"] >> val;
		param.set_pwm_max(val);
		pwm_max_ext=val;

		ymlparam["pwm_db"] >> val;
		param.set_pwm_db(val);
		
		try
		{
			ymlparam["bldc_mode"] >> val;						
		} catch(YAML::TypedKeyNotFound<string> e) 
		{
			val=0;
		}
		param.set_bldc_mode(val);
		
		if (IsVersion(DEFAULT)||IsVersion(ISS))
		{
		  ymlparam["k_ff_zero"] >> val;
		  param.set_k_ff_zero(val);
		  ymlparam["k_ff_shift"] >> val;
		  param.set_k_ff_shift(val);
		  ymlparam["k_ff"] >> val;
		  param.set_k_ff(val);	
		}
		
		try {
		  doc["config"]["has_brake"] >> has_brake;
		} catch (YAML::KeyNotFound &e) {
		  has_brake = false;
		  //M3_DEBUG("exception: %s\n", e.what() );
		}
		
		override_ext_temp = true;
		try {
		  doc["config"]["over_ride_ext_temp"] >> override_ext_temp_act_ec_name;		  
		} catch (YAML::KeyNotFound &e) {
		  override_ext_temp_act_ec_name = "";
		  override_ext_temp = false;		  		  
		}
		//M3_DEBUG("%s has_brake: %d\n", filename, has_brake);
	}
	return true;
}
}
   
