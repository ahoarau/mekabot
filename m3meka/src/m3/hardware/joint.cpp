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

#include "m3/hardware/joint.h"
#include "m3rt/base/m3rt_def.h"
#include "m3rt/base/component_factory.h"
#include <cmath>

namespace m3{
	
using namespace m3rt;
using namespace std;



//#define MODE_SWITCH_SLEW_TIME 1.0 //Seconds
//Slew on joint desired when entering a mode
#define MODE_Q_ON_SLEW_TIME 1.0 
#define MODE_TQ_ON_SLEW_TIME 1.0 
#define MODE_PWM_ON_SLEW_TIME 1.0
#define MODE_BRAKE_OFF_SLEW_TIME 2.0

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////



bool M3Joint::ReadConfig(const char * filename)
{
	YAML::Node doc;
	if (!M3Component::ReadConfig(filename))
		return false;

	GetYamlDoc(filename, doc);	
	doc["actuator_component"] >> act_name;	
	trans = new M3Transmission();	
	trans->ReadConfig(doc["transmission"]);	
	mReal val;
	doc["param"]["kq_p"] >> val;
	param.set_kq_p(val);
	doc["param"]["kq_i"] >> val;
	param.set_kq_i(val);
	doc["param"]["kq_i_range"] >> val;
	param.set_kq_i_range(val);
	doc["param"]["kq_d"] >> val;
	param.set_kq_d(val);
	doc["param"]["kq_i_limit"] >> val;
	param.set_kq_i_limit(val);
	doc["param"]["kq_g"] >> val;
	param.set_kq_g(val);
	doc["param"]["kt_p"] >> val;
	param.set_kt_p(val);
	doc["param"]["kt_i"] >> val;
	param.set_kt_i(val);
	doc["param"]["kt_i_range"] >> val;
	param.set_kt_i_range(val);
	doc["param"]["kt_d"] >> val;
	param.set_kt_d(val);
	doc["param"]["kt_i_limit"] >> val;
	param.set_kt_i_limit(val);
	doc["param"]["max_q"] >> val;
	param.set_max_q(val);
	doc["param"]["min_q"] >> val;
	param.set_min_q(val);
	doc["param"]["max_q_pad"] >> val;
	param.set_max_q_pad(val);
	doc["param"]["min_q_pad"] >> val;
	param.set_min_q_pad(val);
	doc["param"]["max_q_slew_rate"] >> val;
	param.set_max_q_slew_rate(val);
	
	string t;
	try 
	{
		doc["brake"] >> t;
		if (t.compare("none")==0){brake_type=BRAKE_NONE;}
		if (t.compare("auto")==0){brake_type=BRAKE_AUTO;}
		if (t.compare("manual")==0){brake_type=BRAKE_MANUAL;}
	} catch(YAML::TypedKeyNotFound<string> e) 
	{
		brake_type=BRAKE_NONE;
	} 
	
	try 
	{
		doc["control_component"] >> ctrl_simple_name;		
	} catch(YAML::TypedKeyNotFound<string> e) 
	{
		ctrl_simple_name="";
	} 
	return true;
}

bool M3Joint::LinkDependentComponents()
{
	act=(M3Actuator*) factory->GetComponent(act_name);
	if (act==NULL)
	{
		M3_INFO("M3Actuator component %s not found for component %s\n",act_name.c_str(),GetName().c_str());
		return false;
	}
	if (!trans->LinkDependentComponents(factory))
		return false;
	
	if (IsVersion(IQ))
	{
	  ctrl_simple=(M3CtrlSimple*) factory->GetComponent(ctrl_simple_name);
	  if (ctrl_simple==NULL)
	  {
		  M3_INFO("M3CtrlSimple component %s not found for component %s\n",ctrl_simple_name.c_str(),GetName().c_str());		
	  }
	}
	return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void M3Joint::Startup()
{
	if (act!=NULL)
		SetStateSafeOp();
	else
		SetStateError();
}

void M3Joint::Shutdown()
{
	if (trans!=NULL)
		delete trans;
	trans=NULL;
}

void M3Joint::StepStatus()
{
	pnt_cnt++;
	if (IsStateError())
		return;
	status.set_flags(act->GetFlags());
	status.set_amp_temp(act->GetAmpTemp());
	status.set_motor_temp(act->GetMotorTemp());
	status.set_ambient_temp(act->GetAmbientTemp());
	status.set_case_temp(act->GetCaseTemp());
	status.set_power(act->GetPower());
	status.set_current(act->GetCurrent());
	status.set_pwm_cmd(act->GetPwmCmd());
	status.set_theta(trans->GetThetaJointDeg());
	status.set_thetadot(trans->GetThetaDotJointDeg());
	status.set_thetadotdot(trans->GetThetaDotDotJointDeg());
	status.set_torque(trans->GetTorqueJoint()); ;  // TODO: Make GetTorque nNm again
	status.set_torquedot(trans->GetTorqueDotJoint());
}


//Convert from joint-space to actuator-space and do slew/min-jerk smoothing
void M3Joint::CalcThetaDesiredSmooth()
{
	mReal q_des_jt,q_on, q_des;
	//Ramp in from theta at switchover point
	q_on=q_on_slew.Step(1.0,1.0/MODE_Q_ON_SLEW_TIME);
	q_des_jt=command.q_desired()*q_on+(1.0-q_on)*q_switch;
	
	//Clamp in range
	q_des_jt=CLAMP(q_des_jt,param.min_q()+param.min_q_pad(), param.max_q()-param.max_q_pad());
		
	if (command.ctrl_mode()==JOINT_MODE_THETA || command.ctrl_mode()==JOINT_MODE_THETA_GC)
		q_des_jt=q_slew.Step(q_des_jt,CLAMP(command.q_slew_rate(),0.,param.max_q_slew_rate()));

	//Min-Jerk
	if (command.ctrl_mode()==JOINT_MODE_THETA_MJ || command.ctrl_mode()==JOINT_MODE_THETA_GC_MJ  )
		q_des_jt=jerk_joint.Step(GetTimestamp(),q_des_jt,(mReal)command.qdot_desired());	
	
	//Min-Jerk may make desired out of bounds so re-clamp
	q_des_jt=CLAMP(q_des_jt,param.min_q()+param.min_q_pad(),param.max_q()-param.max_q_pad());
	
	//Set result
	trans->SetThetaDesJointDeg(q_des_jt);
}

//Set the brake command and possibly modify the controller desired to be safe
//Pass in controller desired and the current sensor value
void M3Joint::StepBrake(mReal & ctrl_des, mReal curr)
{
	
	if (brake_type == BRAKE_NONE)
		return;
	if (brake_type==BRAKE_MANUAL)
	{
		act->SetBrakeOff( command.brake_off());
		return;
	}

	if (brake_type==BRAKE_AUTO) //Ignore soft-enable, assume wants to be on. Do it safely.
	{
		mReal bs = brake_off_slew.Step(2.0,1.0/MODE_BRAKE_OFF_SLEW_TIME);
		mReal q_des=trans->GetThetaDesJointDeg();
		mReal q_cur=trans->GetThetaJointDeg();
		mReal brake_des=0;
		
		//Handle auto turn-on of brake MODE_BRAKE_OFF_SLEW_TIME/2 seconds after power-up
		if (!act->IsMotorPowerOn() || (!pwr_on_last &&  act->IsMotorPowerOn()))
			brake_off_slew.Reset(0.0);
		pwr_on_last= act->IsMotorPowerOn();
		if (bs>1.0 &&  act->IsMotorPowerOn()) //bs goes from 0 to 2.0. Turn on brake halfway through.
			act->SetBrakeOff(1);
		else
			act->SetBrakeOff(0);
		
		/*if (tmp_cnt++%100==0)
		{
			M3ActuatorCommand * command=(M3ActuatorCommand *)act->GetCommand();
			M3_INFO("------------\n");
			M3_INFO("Brake slew: %f\n",bs);
			M3_INFO("Brake set off: %d\n",command->brake_off());
			M3_INFO("Brake des: %f\n",ctrl_des);
			M3_INFO("Brake curr: %f\n",curr);
		}*/
		
		//Now transform desired to servo current value until brake turned on
		switch(command.ctrl_mode())
		{
			case JOINT_MODE_OFF:
				act->SetBrakeOff(0);
				return;
			//case JOINT_MODE_PWM:
			//	act->SetBrakeOff(1);		
			//	return;
			case JOINT_MODE_THETA_GC:
			case JOINT_MODE_THETA_GC_MJ:
			case JOINT_MODE_THETA:
			case JOINT_MODE_THETA_MJ:
			case JOINT_MODE_TORQUE:
			{
				if (bs<1.0)
					ctrl_des=curr;
				else
					ctrl_des=ctrl_des*(bs-1.0)+(2.0-bs)*curr;
				break;
			}
			case JOINT_MODE_TORQUE_GC:
			{
				if (bs<1.0)
					ctrl_des=0;
				else
					ctrl_des=ctrl_des*(bs-1.0);
				break;	
			}
		};
	
		
			
	}
}

void M3Joint::StepCommand()
{
	if (!act || IsStateSafeOp())
		return;
	
	if (IsVersion(IQ) && !ctrl_simple)
		return;
	
	
	if(IsStateError())
	{
		if (IsVersion(IQ))
		  ctrl_simple->SetDesiredControlMode(CTRL_MODE_OFF);
		else
		  act->SetDesiredControlMode(ACTUATOR_MODE_OFF);
		  
		return;
	}
	
	/*tmp_cnt++;
	if (tmp_cnt == 100)
	{
	    M3_DEBUG("th_in: %f\n", command.q_desired());
	    

	}*/
	
	
	
	if (command.ctrl_mode() == JOINT_MODE_THETA && command.smoothing_mode() == SMOOTHING_MODE_MIN_JERK)
	{
	  command.set_ctrl_mode(JOINT_MODE_THETA_MJ);
	  	
	}
	
	if (command.ctrl_mode() == JOINT_MODE_THETA_GC && command.smoothing_mode() == SMOOTHING_MODE_MIN_JERK)
	{
	  command.set_ctrl_mode(JOINT_MODE_THETA_GC_MJ);	  
	}
	
	//Avoid pops
	if (command.ctrl_mode()!=mode_last)
	{
		q_on_slew.Reset(0.0);
		tq_on_slew.Reset(0.0);
		pwm_on_slew.Reset(0.0);
		q_slew.Reset(GetThetaDeg());
		jerk_joint.Startup(GetTimestamp(), GetThetaDeg());
		if (IsVersion(IQ)) 
		  tq_switch=GetTorque()*1000;  // TODO: Make GetTorque nNm again
		else
		  tq_switch=GetTorque(); 
		q_switch=GetThetaDeg();
		pwm_switch=GetPwmCmd();
		if (IsVersion(IQ)) { 
		  ctrl_simple->ResetIntegrators();
		}
	}
	if (IsVersion(IQ)) { 
	  if (!act->IsMotorPowerSlewedOn())
	      ctrl_simple->ResetIntegrators();	 
	}
	
	
	if (IsVersion(IQ)) { 
	  
		ctrl_simple->SetDesiredControlMode(CTRL_MODE_OFF);
		
		switch(command.ctrl_mode())
		{
			
			case JOINT_MODE_THETA:
			case JOINT_MODE_THETA_MJ:
			{
				if (!IsEncoderCalibrated())
				{
				// M3_INFO("Not calib %s\n",GetName().c_str());
				  break;
				}
				CalcThetaDesiredSmooth();
				mReal des=trans->GetThetaDesActuatorDeg();
				StepBrake(des,trans->GetThetaActuatorDeg());
				//Do PID in actuator space so result is direct PWM
				//ctrl_simple->SetDesiredControlMode(CTRL_MODE_THETA);
				ctrl_simple->SetDesiredControlMode(CTRL_MODE_THETA);
				ctrl_simple->SetDesiredTheta(DEG2RAD(des));
								
				break;
			}
			case JOINT_MODE_THETA_GC:
			case JOINT_MODE_THETA_GC_MJ:
			{
				if (!IsEncoderCalibrated())
				  break;
				mReal stiffness,gravity,tq_des;
				CalcThetaDesiredSmooth();
				mReal des=trans->GetThetaDesJointDeg();
				StepBrake(des,trans->GetThetaJointDeg());
				//Do PID in joint space
				tq_des =pid_theta_gc.Step(trans->GetThetaJointDeg(),
						trans->GetThetaDotJointDeg(),
						des,
						param.kq_p(),
						param.kq_i(),
						param.kq_d(),
						param.kq_i_limit(),
						param.kq_i_range());
				/*if (pnt_cnt%200==0) {		
					M3_DEBUG("actuator: %s\n", GetName().c_str());
					M3_DEBUG("tq_des: %f\n",tq_des);
					M3_DEBUG("des: %f\n\n",des);
				}*/
				stiffness=CLAMP(command.q_stiffness(),0.0,1.0);
				gravity=GetTorqueGravity()*param.kq_g();
				//Ramp in from torque at switch-over point
				mReal tq_on, tq_out;
				tq_on=tq_on_slew.Step(1.0,1.0/MODE_TQ_ON_SLEW_TIME); 
				//tq_on = 1.0;
				tq_out=tq_on*(stiffness*tq_des-gravity)+(1.0-tq_on)*tq_switch;
				//tq_out = tq_switch;
				//tq_out = stiffness*tq_des-gravity;
				//Send out
				trans->SetTorqueDesJoint(tq_out/1000.0);	//TODO: convert back to mNm	
				
				ctrl_simple->SetDesiredControlMode(CTRL_MODE_TORQUE);
				ctrl_simple->SetDesiredTorque(trans->GetTorqueDesActuator());
				
				break;
			}
			case JOINT_MODE_TORQUE_GC:			
			{
			      if (!IsEncoderCalibrated())
				  break;
				mReal tq_on, tq_out,gravity;
				mReal tq_des=command.tq_desired();
				StepBrake(tq_des,trans->GetTorqueJoint());
				gravity=GetTorqueGravity()*param.kq_g();
				//Ramp in from torque at switch-over point
				tq_on=tq_on_slew.Step(1.0,1.0/MODE_TQ_ON_SLEW_TIME);
				tq_out=tq_on*(tq_des-gravity)+(1.0-tq_on)*tq_switch;
				//Send out
				trans->SetTorqueDesJoint(tq_out/1000.0);				
				
				ctrl_simple->SetDesiredControlMode(CTRL_MODE_TORQUE);
				ctrl_simple->SetDesiredTorque(trans->GetTorqueDesActuator());
				
				break;
			}
			case JOINT_MODE_TORQUE:
			{		  
				mReal tq_on,tq_out;
				//Ramp in from torque at switch-over point
				mReal tq_des=command.tq_desired();
				StepBrake(tq_des,trans->GetTorqueJoint());
				tq_on=tq_on_slew.Step(1.0,1.0/MODE_TQ_ON_SLEW_TIME);
				tq_out=tq_on*tq_des+(1.0-tq_on)*tq_switch;
				//Send out
				trans->SetTorqueDesJoint(tq_out/1000.0);
				
				ctrl_simple->SetDesiredControlMode(CTRL_MODE_TORQUE);
				ctrl_simple->SetDesiredTorque(trans->GetTorqueDesActuator());
								
				break;
			}
			case JOINT_MODE_OFF:
			case JOINT_MODE_PWM: // no longer used 
			default:
				ctrl_simple->SetDesiredControlMode(CTRL_MODE_OFF);
				//act->SetDesiredControlMode(ACTUATOR_MODE_OFF);
				mReal des; //dummy
				StepBrake(des,0);
				break;
		};
	} else { // Legacy, no supported on BMW,MAX2 versions 2.0
	
//		act->SetDesiredControlMode(ACTUATOR_MODE_OFF);//default
		
		switch(command.ctrl_mode())
		{
			case JOINT_MODE_PWM:
			{
				//Ramp in from pwm at switch-over point
				mReal pwm_des=command.pwm_desired();
				StepBrake(pwm_des,0);			
				//StepBrake(pwm_des,command.pwm_cmd());			
				
				mReal pwm_on, pwm_out;
				pwm_on=pwm_on_slew.Step(1.0,1.0/MODE_PWM_ON_SLEW_TIME);
				pwm_out=pwm_on*pwm_des+(1.0-pwm_on)*pwm_switch;
				//Send out
				act->SetDesiredControlMode(ACTUATOR_MODE_PWM);
				if (disable_pwm_ramp)
				  act->SetDesiredPwm(command.pwm_desired());
				else
				  act->SetDesiredPwm((int)pwm_out);;
				break;
			}
			case JOINT_MODE_THETA:
			case JOINT_MODE_THETA_MJ:
			{
				
				
					
				if (!IsEncoderCalibrated())
				{
//					M3_INFO("Not calib %s\n",GetName().c_str());
					break;
				}
				
				CalcThetaDesiredSmooth();
				mReal des=trans->GetThetaDesActuatorDeg();
				StepBrake(des,trans->GetThetaActuatorDeg());
				//Do PID in actuator space so result is direct PWM
				mReal pwm =pid_theta.Step(trans->GetThetaActuatorDeg(),
							trans->GetThetaDotActuatorDeg(),
							des,
							param.kt_p(),
							param.kt_i(),
							param.kt_d(),
							param.kt_i_limit(),
							param.kt_i_range());
				act->SetDesiredControlMode(ACTUATOR_MODE_PWM);
				//Ramp in from pwm at switch-over point
				mReal pwm_on, pwm_out;
				pwm_on=pwm_on_slew.Step(1.0,1.0/MODE_PWM_ON_SLEW_TIME);
				pwm_out=pwm_on*pwm+(1.0-pwm_on)*pwm_switch;
				act->SetDesiredPwm((int)pwm_out);
				/*if (tmp_cnt++%100==0)
				{
					M3_INFO("des %3.2f sen: %3.2f pid: %3.2f out: %3.2f\n",des,trans->GetThetaActuatorDeg(),
						pwm,pwm_out);
				}*/
				break;
			}
			case JOINT_MODE_THETA_GC:
			case JOINT_MODE_THETA_GC_MJ:
			{
				if (!IsEncoderCalibrated())
				  break;
				mReal stiffness,gravity,tq_des;
				CalcThetaDesiredSmooth();
				mReal des=trans->GetThetaDesJointDeg();
				StepBrake(des,trans->GetThetaJointDeg());
				//Do PID in joint space
				tq_des =pid_theta_gc.Step(trans->GetThetaJointDeg(),
						trans->GetThetaDotJointDeg(),
						des,
						param.kq_p(),
						param.kq_i(),
						param.kq_d(),
						param.kq_i_limit(),
						param.kq_i_range());
				stiffness=CLAMP(command.q_stiffness(),0.0,1.0);
				gravity=GetTorqueGravity()*param.kq_g();
				//Ramp in from torque at switch-over point
				mReal tq_on, tq_out;
				tq_on=tq_on_slew.Step(1.0,1.0/MODE_TQ_ON_SLEW_TIME);
				tq_out=tq_on*(stiffness*tq_des-gravity)+(1.0-tq_on)*tq_switch;
				//Send out
				
				trans->SetTorqueDesJoint(tq_out);
				act->SetDesiredControlMode(ACTUATOR_MODE_TORQUE);
				act->SetDesiredTorque(trans->GetTorqueDesActuator());
				break;
			}
			case JOINT_MODE_TORQUE_GC:			
			{
			      if (!IsEncoderCalibrated())
				  break;
				mReal tq_on, tq_out,gravity;
				mReal tq_des=command.tq_desired();
				StepBrake(tq_des,trans->GetTorqueJoint());
				gravity=GetTorqueGravity()*param.kq_g();
				//Ramp in from torque at switch-over point
				tq_on=tq_on_slew.Step(1.0,1.0/MODE_TQ_ON_SLEW_TIME);
				tq_out=tq_on*(tq_des-gravity)+(1.0-tq_on)*tq_switch;
				//Send out
				trans->SetTorqueDesJoint(tq_out);
				act->SetDesiredControlMode(ACTUATOR_MODE_TORQUE);

				act->SetDesiredTorque(trans->GetTorqueDesActuator());
				break;
			}
			case JOINT_MODE_TORQUE:
			{		  
				mReal tq_on,tq_out;
				//Ramp in from torque at switch-over point
				mReal tq_des=command.tq_desired();
				
				StepBrake(tq_des,trans->GetTorqueJoint());
				tq_on=tq_on_slew.Step(1.0,1.0/MODE_TQ_ON_SLEW_TIME);
				tq_out=tq_on*tq_des+(1.0-tq_on)*tq_switch;
				//Send out
				trans->SetTorqueDesJoint(tq_out);
				act->SetDesiredControlMode(ACTUATOR_MODE_TORQUE);
				act->SetDesiredTorque(trans->GetTorqueDesActuator());			
				
				/*if (tmp_cnt++ == 100)
				{
				    M3_DEBUG("tq_des: %f\n", tq_des);
				    M3_DEBUG("tq_on: %f\n", tq_on);
				    M3_DEBUG("tq_switch: %f\n", tq_switch);
				    M3_DEBUG("tq_out: %f\n", tq_out);				    
				    M3_DEBUG("tq_act: %f\n", trans->GetTorqueDesActuator());
				    tmp_cnt = 0;
				}*/
				break;
			}
			case JOINT_MODE_OFF:
			default:
				act->SetDesiredControlMode(ACTUATOR_MODE_OFF);
				mReal des; //dummy
				StepBrake(des,0);
				break;
		};
	}
	
	mode_last=(int)command.ctrl_mode();
	
	/*if (tmp_cnt == 100)
	{
	    M3_DEBUG("tq_out: %f\n", ((M3ActuatorCommand*)act->GetCommand())->tq_desired() );
	    
	    tmp_cnt = 0;
	}*/
	
}


}

