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

#include "actuator.h"
#include <m3rt/base/m3rt_def.h>
#include <m3rt/base/component_factory.h>
#include <inttypes.h>

namespace m3{
	
using namespace m3rt;
using namespace std;

///////////////////////////////////////////////////////
#define BOUNDS_AVERAGE_TIME 2.0 //Seconds to average temp values for out-of-bounds errors
#define SENSOR_FAULT_LIMIT 0.1 // 100 ms limit for sensor error before error state

void M3Actuator::Startup()
{
	int downsample=20;
	amp_temp_avg.Resize((int)(BOUNDS_AVERAGE_TIME*1000000),downsample);
	
	if (ecc!=NULL)
		SetStateSafeOp();
	else
		SetStateError();
	
	if (!encoder_calib_req)
	  old_is_calibrated = true;
}

void M3Actuator::Shutdown()
{

  if (IsVersion(DEFAULT) || IsVersion(ISS) || IsVersion(IQ))
  {
    motor.ThermalShutdown(config_filename, ex_sense.GetTempC());
  }
  
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool M3Actuator::ReadConfig(const char * filename)
{	
	int val;
	string str;
	mReal mval;
	YAML::Node doc;
	config_filename = string(filename);

	if (!M3Component::ReadConfig(filename))
		return false;
	GetYamlDoc(filename, doc);
	
	if (IsVersion(ISS) || IsVersion(IQ))
	{
		ex_sense.ReadConfig(doc["calib"]["ext_temp"]);
	}
	if (IsVersion(DEFAULT)) //Moved to motor model post version 0
	{	
		doc["param"]["max_motor_temp"] >> max_motor_temp;
		doc["param"]["max_current"] >> max_current;
		ex_sense.ReadConfig(doc["calib"]["motor_temp"]); //motor_temp renamed ext_temp
	}
	if (IsVersion(DEFAULT) || IsVersion(ISS) || IsVersion(IQ))
	{
		doc["ec_component"] >> ecc_name;
		doc["ignore_bounds"] >> val;
		ignore_bounds = (bool) val;
		try
		{
			doc["safe_pwm_limit"] >> val;
			safe_pwm_limit = (bool) val;
		} catch(YAML::TypedKeyNotFound<string> e) 
		{
			safe_pwm_limit = (bool) 0;
		} 
		try
		{
			doc["use_i_torque_ctrl"] >> val;
			use_i_torque_ctrl = (bool) val;
		} catch(YAML::TypedKeyNotFound<string> e) 
		{
			use_i_torque_ctrl = (bool) 0;
		} 
		doc["param"]["max_amp_temp"] >> mval;
		param.set_max_amp_temp(mval);
		doc["param"]["max_tq"] >> mval;
		param.set_max_tq(mval);
		doc["param"]["min_tq"] >> mval;
		param.set_min_tq(mval);
		try               
		{
			doc["param"]["max_i"] >> mval;
			param.set_max_i(mval);
		} catch(YAML::TypedKeyNotFound<string> e) 
		{
			param.set_max_i(0.0);
		} 
		motor.ReadConfig(doc["calib"]["motor"], config_filename);	
		q_sense.ReadConfig( doc["calib"]["theta"]);
		tq_sense.ReadConfig(doc["calib"]["torque"]);
		at_sense.ReadConfig(doc["calib"]["amp_temp"]);
		i_sense.ReadConfig( doc["calib"]["current"]);
		angle_df.ReadConfig( doc["calib"]["angle_df"]);
		torquedot_df.ReadConfig(doc["calib"]["torquedot_df"]);
		try 
		{
			doc["param"]["max_overload_time"] >> mval;
			param.set_max_overload_time(mval);
		} catch(YAML::TypedKeyNotFound<string> e) 
		{
			param.set_max_overload_time(1.0);
		} 
		try 
		{		  
			doc["encoder_calib_req"] >> val;
			encoder_calib_req = bool(val);
		} catch(YAML::TypedKeyNotFound<string> e) 
		{
			encoder_calib_req=0;
		} 
		try 
		{		  
			doc["disable_vertx_check"] >> disable_vertx_check;
		} catch(YAML::TypedKeyNotFound<string> e) 
		{
			disable_vertx_check=false;
		} 
		try 
		{		  
			doc["vertx_timeout_limit_ms"] >> vertx_timeout_limit_ms;
		} catch(YAML::TypedKeyNotFound<string> e) 
		{
			vertx_timeout_limit_ms=int(SENSOR_FAULT_LIMIT*1000.0);
		} 
	}
	if (IsVersion(ISS) || IsVersion(IQ))
	{
		doc["param"]["max_amp_current"] >> mval;
		param.set_max_amp_current(mval);
		
	}
	if (IsVersion(IQ))
	{
		doc["param"]["pid_current"]["k_p"] >> mval;
		ParamPIDTorque()->set_k_p(mval);
		doc["param"]["pid_current"]["k_i"] >> mval;
		ParamPIDTorque()->set_k_i(mval);
		doc["param"]["pid_current"]["k_d"] >> mval;
		ParamPIDTorque()->set_k_d(mval);
		doc["param"]["pid_current"]["k_i_limit"] >> mval;
		ParamPIDTorque()->set_k_i_limit(mval);
		doc["param"]["pid_current"]["k_i_range"] >> mval;
		ParamPIDTorque()->set_k_i_range(mval);

		try {
		doc["calib"]["torque"]["torque_shift"] >> torque_shift;
		} catch (YAML::KeyNotFound &e) {
		torque_shift = 1.0;
		}
		
		try {
			doc["calib"]["amp_control_input"] >> str;
			if (str.compare("pwm") == 0)			{amp_control_input	= ACTUATOR_INPUT_PWM;}
			else if (str.compare("current") == 0)	{amp_control_input	= ACTUATOR_INPUT_CURRENT;}
		} catch (YAML::KeyNotFound &e) {
			amp_control_input = ACTUATOR_INPUT_CURRENT;
		}
		
	}
	return true;
}

bool M3Actuator::LinkDependentComponents()
{
	ecc=(M3ActuatorEc*) factory->GetComponent(ecc_name);
	if (ecc==NULL)
	{
		M3_INFO("M3ActuatorEc component %s not found for component %s\n",ecc_name.c_str(),GetName().c_str());
		return false;
	}
	return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void M3Actuator::StepStatus()
{
	if (IsStateError())
		return;
	
	
	 
	if (IsVersion(DEFAULT) || IsVersion(ISS) || IsVersion(IQ))
	{
		M3ActuatorEcStatus * ecs = (M3ActuatorEcStatus * )(ecc->GetStatus());

		
		//Calibrate Raw Data
		
		//M3_INFO("%lu\n",ecs->timestamp()-old_ts);
		//old_ts = ecs->timestamp();
		//Angle
		/*unsigned int low=(unsigned short)ecs->qei_on();
		unsigned int high=(((unsigned short)ecs->qei_rollover())<<16);
		int ticks = high|low;		
		printf("%ld, %lu, %d %hd, %hd\n",(long)(GetTimestamp()-old_ts_rtai), (unsigned long)(ecs->timestamp()-old_ts), ticks-old_ticks, (short)(ecs->timestamp()), (short)ticks);
		//printf("%ld, %lu, %d\n",(long)(GetTimestamp()), (unsigned long)(ecs->timestamp()), ticks);
		old_ticks = ticks;
		old_ts = ecs->timestamp();
		old_ts_rtai = GetTimestamp();*/
		//printf("%lu\n", (unsigned long)(ecs->timestamp()));
		//printf("%ld, %lu, %d\n",(long)(GetTimestamp()), (unsigned long)(ecs->timestamp()), ticks);
		
		q_sense.Step(ecs->qei_on(),ecs->qei_period(),ecs->qei_rollover());
		// Clear state on filters to prevent blow up on encoder zero
		if (!old_is_calibrated && IsEncoderCalibrated() && encoder_calib_req)
		{
		  angle_df.Reset();		 
		}
		old_is_calibrated = IsEncoderCalibrated();
		angle_df.Step(q_sense.GetThetaDeg(),q_sense.GetThetaDotDeg());
		status.set_theta(angle_df.GetTheta());
		/*if (q_sense.IsTypeQEI())
		{
		    status.set_thetadot(q_sense.GetThetaDotDeg());
		} else {
		    mReal td=angle_df.GetThetaDot();
		    //if (ABS(td)<2.0)
		    //  td=0.0;
		    status.set_thetadot(td);
		}*/
		mReal td=angle_df.GetThetaDot();
		    //if (ABS(td)<2.0)
		    //  td=0.0;
		status.set_thetadot(td);
		status.set_thetadotdot(angle_df.GetThetaDotDot());
		
		//Torque
		tq_sense.Step(ecs->adc_torque()/torque_shift);
		status.set_torque(tq_sense.GetTorque_mNm());
		status.set_torquedot(torquedot_df.Step(status.torque()));
		
		//Current and Temp
		
		ex_sense.Step(ecs->adc_ext_temp()); //was motor_temp
		at_sense.Step(ecs->adc_amp_temp());
		i_sense.Step(ecs->adc_current_a(),ecs->adc_current_b(),ecs->current_ma());
		if (!ecc->IsMotorPowerOn()) //Force zero-reading when off to ignore small bias errors.
		  i_sense.SetZero();
		motor.Step(i_sense.GetCurrent_mA(), GetPwmCmd(), GetThetaDotDeg()*60.0/360.0, ex_sense.GetTempC());//Compute motor 
		
		//printf("current: %f\n",ecs->current_ma());
		status.set_current(i_sense.GetCurrent_A());
		status.set_amp_temp(amp_temp_avg.Step(at_sense.GetTempC()));
		status.set_ext_temp(ex_sense.GetTempC());
		status.set_motor_temp(motor.GetWindingTemp());
		status.set_ambient_temp(motor.GetAmbientTemp());
		status.set_case_temp(motor.GetCaseTemp());
		
		status.set_power(motor.GetPowerElec());
		//if (tmp_cnt%100==0)
		 // M3_INFO("%s: %f %f\n",GetName().c_str(),ex_sense.GetTempC(),motor.GetWindingTemp());
		if (safe_pwm_limit && !ignore_bounds)
		{
		  //if (tmp_cnt++%100==0)
		    //M3_INFO("PwmMax: %f\n",motor.GetMaxPwmDuty());
		    ecc->SetPwmMax(motor.GetSafePwmDuty());
		}
			
		
		//Update status data
		status.set_flags(ecs->flags());
		status.set_pwm_cmd(ecs->pwm_cmd());
	
		// Detect Loss of Vertx sensors
		q_sense.StepError(ecs->qei_rollover());
		tq_sense.StepError(ecs->qei_period());	
		
		if (last_theta_err != q_sense.GetError())
		  sensor_fault_theta_cnt++;
		else
		  sensor_fault_theta_cnt = 0;
		
		if (last_torque_err != tq_sense.GetError())
		  sensor_fault_torque_cnt++;
		else
		  sensor_fault_torque_cnt = 0;
		
		if (!disable_vertx_check)
		{
			if (((mReal)sensor_fault_theta_cnt) > ((mReal(vertx_timeout_limit_ms)/1000.0) * (mReal)RT_TASK_FREQUENCY)) // throw an error if we loose jnt/tq sensor for 100 ms
			{
			  M3_DEBUG("th_cnt: %d, th_err: %d, th_err_last: %d\n", sensor_fault_theta_cnt, q_sense.GetError(), last_theta_err);
			  SetStateError();
			  M3_ERR("------ SENSOR FAULT EVENT FOR JOINT ANGLE OF %s ---------------\n",GetName().c_str());
			}
		
			if (((mReal)sensor_fault_torque_cnt) > ((mReal(vertx_timeout_limit_ms)/1000.0) * (mReal)RT_TASK_FREQUENCY)) // throw an error if we loose jnt/tq sensor for 100 ms
			{
			  M3_DEBUG("tq_cnt: %d, tq_err: %d, tq_err_last: %d\n", sensor_fault_torque_cnt, tq_sense.GetError(), last_torque_err);
			  SetStateError();
			  M3_ERR("------ SENSOR FAULT EVENT FOR TORQUE OF %s ---------------\n",GetName().c_str());
			}
		}
		last_theta_err = q_sense.GetError();
		last_torque_err = tq_sense.GetError();
	}
	StepOverloadDetect();
	
	
	
}

void M3Actuator::StepOverloadDetect()
{
	int overload_cnt_orig=overload_cnt;
	
	if (!ignore_bounds && GetAmpTemp()>param.max_amp_temp())
		overload_cnt++;
	
	if (IsVersion(DEFAULT))
	{
		if (!ignore_bounds && GetCurrent()>max_current)
			overload_cnt++;
		//This is now redundant, leave in for now. Allows to set a more conservative max winding temp.
		if (!ignore_bounds && (GetMotorTemp()>max_motor_temp))
			overload_cnt++;
		if (!ignore_bounds&& motor.GetWindingTemp()>motor.GetMaxWindingTemp())
			overload_cnt++;
	}
	
	if (IsVersion(ISS) || IsVersion(IQ))
	{
		if (!ignore_bounds&& motor.GetWindingTemp()>(motor.GetMaxWindingTemp()))
			overload_cnt++;
		if (!ignore_bounds && motor.GetCurrentContinuous()>param.max_amp_current())
			overload_cnt++;
	}
	if (overload_cnt==overload_cnt_orig) //no overload, reset cntr
	{
		overload_cnt==0;
		return;
	}
	
	if (overload_cnt >= (int)((mReal)param.max_overload_time()*(mReal)RT_TASK_FREQUENCY))
	{
		SetStateError();
		M3_ERR("------ OVER-THRESHOLD EVENT FOR %s ---------------\n",GetName().c_str());
		M3_ERR("Amp temp %f (C)| Threshold of %f (C)\n",GetAmpTemp(),param.max_amp_temp());
		if (IsVersion(DEFAULT))
		{
			M3_ERR("Motor current %f | Threshold of %f \n",GetCurrent(),max_current);
			M3_ERR("Motor temp %f (C) | Threshold of %f (C) \n",GetMotorTemp(),max_motor_temp);
			M3_ERR("Motor winding temp %f (C) | Threshold of %f \n",motor.GetWindingTemp(),motor.GetMaxWindingTemp());
		}
		if (IsVersion(ISS) || IsVersion(IQ))
		{
			M3_ERR("Motor winding temp %f (C) | Threshold of %f (C)\n",motor.GetWindingTemp(),motor.GetMaxWindingTemp());
			M3_ERR("Amplifier current %f |Threshold of %f (mA)\n",motor.GetCurrentContinuous(),param.max_amp_current());
		}
		M3_ERR("------------------------------------------------------\n");
	}
}
void M3Actuator::StepCommand()
{
	pnt_cnt++;
	status.set_torque_error(0);
	if (!ecc || IsStateSafeOp())
		return;

	M3ActuatorEcCommand * ec_command = (M3ActuatorEcCommand *)ecc->GetCommand();
	M3ActuatorEcParam * ec_param = (M3ActuatorEcParam *)ecc->GetParam();
	
	/*if (tmp_cnt++  == 100)
	{
	  M3_DEBUG("tq_mNm: %f\n", command.tq_desired());
	 // M3_DEBUG("tq_ec: %d\n", tq_sense.mNmToTicks(tq_mNm,&i_sense));				  
	  tmp_cnt = 0;
	}*/
	
	
	if(IsStateError())
	{
		ec_command->set_mode(ACTUATOR_EC_MODE_OFF);
		return;
	}
	
	
	if (IsVersion(IQ)) { // new style is simple pass through, with limit checking
		status.set_mode_cmd(command.ctrl_mode());
		ec_command->set_brake_off(command.brake_off());
		switch (command.ctrl_mode())
		{
			case ACTUATOR_MODE_OFF:
				ec_command->set_mode(ACTUATOR_EC_MODE_OFF);
				break;
			case ACTUATOR_MODE_PWM:
				ec_command->set_mode(ACTUATOR_EC_MODE_PWM);
				ec_command->set_t_desire(pwm_dither.Step(command.pwm_desired()));
				break;
			case ACTUATOR_MODE_TORQUE:
				ec_command->set_mode(ACTUATOR_EC_MODE_OFF);
				break;
			case ACTUATOR_MODE_CURRENT:
			{
				ec_command->set_mode(ACTUATOR_EC_MODE_CURRENT);
				mReal i_mA=CLAMP(command.i_desired(),-1*param.max_i(),param.max_i())*1000.0; 
				ec_command->set_current_desired(i_mA);
				status.set_i_cmd(i_mA/1000.0);
				break;
			}
			case ACTUATOR_MODE_BRAKE:
				ec_command->set_mode(ACTUATOR_EC_MODE_BRAKE);
				break;
			default:
				ec_command->set_mode(ACTUATOR_EC_MODE_OFF);
				break;
		}
	
	} else { // Legacy, no supported on BMW,MAX2 versions 2.0
		ec_command->set_brake_off(command.brake_off());

		switch(command.ctrl_mode())
		{
			case ACTUATOR_MODE_OFF:
				ec_command->set_mode(ACTUATOR_EC_MODE_OFF);
				break;
			case ACTUATOR_MODE_PWM:
				ec_command->set_mode(ACTUATOR_EC_MODE_PWM);
				ec_command->set_t_desire(pwm_dither.Step(command.pwm_desired()));
				break;
			case ACTUATOR_MODE_TORQUE:		
			{
				mReal tq_mNm=command.tq_desired();
				tq_mNm=CLAMP(tq_mNm,param.min_tq(),param.max_tq());
				
				
				
				if (use_i_torque_ctrl && IsVersion(IQ)) //Do torque control here, command current
				{
				mReal i_mA = pid_torque.Step(GetTorque(),
							GetTorqueDot(),
							tq_mNm,
							ParamPIDTorque()->k_p(), 
							ParamPIDTorque()->k_i(),
							ParamPIDTorque()->k_d(),
							ParamPIDTorque()->k_i_limit(),
							ParamPIDTorque()->k_i_range());
					ec_command->set_mode(ACTUATOR_EC_MODE_CURRENT);
					i_mA=CLAMP(i_mA,-1*param.max_i(),param.max_i()); 
					ec_command->set_t_desire(i_sense.mAtoTicks(i_mA));
				}
				else if (tq_sense.IsFFCurrentCtrl()) //Fake a torque by commanding a voltage
				{
				ec_command->set_mode(ACTUATOR_EC_MODE_PWM);
				ec_command->set_t_desire(tq_sense.mNmToTicks(tq_mNm,&i_sense));
				/*if (tmp_cnt++  == 100)
				{
				  M3_DEBUG("tq_mNm: %f\n", tq_mNm);
				  M3_DEBUG("tq_ec: %f\n", tq_sense.mNmToTicks(tq_mNm,&i_sense));				  
				  tmp_cnt = 0;
				}*/
				} else 
				{
				ec_command->set_mode(ACTUATOR_EC_MODE_TORQUE); //Do torque control on DSP
				int raw_t_max= tq_sense.mNmToTicks(param.max_tq());
				int raw_t_min= tq_sense.mNmToTicks(param.min_tq());
				if (raw_t_min>raw_t_max) //Can be reversed depending on calibration
				{
					int tmp=raw_t_min;
					raw_t_min=raw_t_max;
					raw_t_max=tmp;
				}
				//Overwrite the raw  params with the calibrated values
				ec_param->set_t_max(raw_t_max);
				ec_param->set_t_min(raw_t_min);			  
				status.set_torque_error(tq_mNm-status.torque());
				ec_command->set_t_desire(tq_sense.mNmToTicks(tq_mNm));
				if (ecc->UseTorqueFF())
					ecc->SetTorqueFF(motor.mNmToPwm(tq_mNm));
				}
				break;
			}
			case ACTUATOR_MODE_CURRENT:
				if ( IsVersion(IQ)) 
				{
				ec_command->set_mode(ACTUATOR_EC_MODE_CURRENT);
				mReal i_mA=CLAMP(command.i_desired(),-1*param.max_i(),param.max_i()); 
				ec_command->set_t_desire(i_sense.mAtoTicks(i_mA));
				}
				else //Previous versions don't support current mode
				{
				ec_command->set_mode(ACTUATOR_EC_MODE_OFF);
				ec_command->set_t_desire(0);
				}
				break;
			default:
				ec_command->set_mode(ACTUATOR_EC_MODE_OFF);
				break;
		};
	}
	
// 	if (tmp_cnt++ == 200)
// 	{
// 	  //M3_INFO("Name: %s\n", GetName().c_str());
// 	  M3_INFO("Act: I_des_in %f I_des_out %f\n",
// 		  ec_command->current_desired(), command.i_desired());
// 	  tmp_cnt = 0;
// 	}
	
}

} // namespace
