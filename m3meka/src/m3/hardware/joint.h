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

#ifndef M3_JOINT_H
#define M3_JOINT_H

#include "m3rt/base/component.h"
#include "m3/hardware/joint.pb.h"
#include "m3/hardware/actuator_ec.pb.h"
#include "m3/hardware/actuator.pb.h"
#include "m3/hardware/actuator.h"
#include "m3/hardware/ctrl_simple.h"
#include "m3/hardware/transmission.h"
#include "m3/toolbox/trajectory.h"
#include <google/protobuf/message.h>



namespace m3
{
	using namespace std;


/////////////////////////////////////////////////////////////////////////
class M3Transmission;




//Joint space representation.
class M3Joint: public m3rt::M3Component
{
	public:
		M3Joint(): m3rt::M3Component(JOINT_PRIORITY),brake_type(BRAKE_NONE),cpj(0),pwr_on_last(0),brake_off_cmd(0),mode_last(-1),trans(0),
		  tq_switch(0), q_switch(0), pwm_switch(0), disable_pwm_ramp(false), ctrl_simple(NULL),tmp_cnt(0)
		{
			RegisterVersion("default",DEFAULT);	//RBL. Now works with default case/ambient values
			RegisterVersion("iss",ISS);		//ISS. Now displays motor-model case/ambient/motor temp values
			RegisterVersion("iq",IQ);
		}
		//API
		void SetDesiredPwm(int val){command.set_pwm_desired(val);}
		void SetDesiredThetaDeg(mReal val){command.set_q_desired(val);}
		void SetDesiredThetaDotDeg(mReal val){command.set_qdot_desired(val);}
		void SetDesiredThetaRad(mReal val){command.set_q_desired(RAD2DEG(val));}
		void SetDesiredThetaDotRad(mReal val){command.set_qdot_desired(RAD2DEG(val));}
		void SetDesiredStiffness(mReal val){command.set_q_stiffness(val);}
		void SetDesiredControlMode(JOINT_MODE val){command.set_ctrl_mode(val);}
		void SetDesiredSmoothingMode(SMOOTHING_MODE val){command.set_smoothing_mode(val);}
		void SetDesiredTorque(mReal val){command.set_tq_desired(val);}
		void SetSlewRate(mReal val){command.set_q_slew_rate(val);}
		void SetBrakeOff(bool off){command.set_brake_off(off);}
		void DisablePwmRamp(){disable_pwm_ramp = true;} // for when we use current control
		mReal GetTorque(){return status.torque();}
		mReal GetTorqueDot(){return status.torquedot();}
		mReal GetTorqueGravity(){return status.torque_gravity();}
		void  SetTorqueGravity(mReal val){status.set_torque_gravity(val);}
		M3Actuator * GetActuator(){return act;}
		mReal GetMotorTemp(){return status.motor_temp();}
		mReal GetAmpTemp(){return status.amp_temp();}
		mReal GetAmbientTemp(){return status.ambient_temp();}
		mReal GetCaseTemp(){return status.case_temp();}
		mReal GetPower(){return status.power();}
		mReal GetCurrent(){return status.current();}
		mReal GetThetaDeg(){return status.theta();}
		mReal GetThetaRad(){return DEG2RAD(status.theta());}
		mReal GetThetaDotDeg(){return status.thetadot();}
		mReal GetThetaDotRad(){return DEG2RAD(status.thetadot());}
		mReal GetThetaDotDotDeg(){return status.thetadotdot();}
		mReal GetThetaDotDotRad(){return DEG2RAD(status.thetadotdot());}
		mReal GetThetaMaxDeg(){return param.max_q();}
		mReal GetThetaMinDeg(){return param.min_q();}
		int GetPwmCmd(){return status.pwm_cmd();}
		int GetFlags(){return status.flags();}
		int64_t GetTimestamp(){return status.base().timestamp();}
		M3Joint * GetCoupledJoint(){return cpj;}
		int GetTicks(){return act->GetTicks();}		
		virtual bool IsMotorPowerOn(){return act->IsMotorPowerOn();}
		virtual bool IsLimitSwitchPosOn() {return act->IsLimitSwitchPosOn();}
		virtual bool IsLimitSwitchNegOn() {return act->IsLimitSwitchNegOn();}
		virtual bool IsEncoderCalibrated(){return act->IsEncoderCalibrated();}
		void SetLimitSwitchNegZeroEncoder(){act->SetLimitSwitchNegZeroEncoder();}
		void ClrLimitSwitchNegZeroEncoder(){act->ClrLimitSwitchNegZeroEncoder();}
		
		google::protobuf::Message * GetCommand(){return &command;}
		google::protobuf::Message * GetStatus(){return &status;}
		google::protobuf::Message * GetParam(){return &param;}			
		M3Transmission * GetTransmission(){return trans;} //Allow other components to access state
		int mNmToTicks(mReal x){return act->mNmToTicks(x);}
	protected:
		enum{DEFAULT,ISS, IQ};
		enum {BRAKE_NONE, BRAKE_AUTO, BRAKE_MANUAL};
		virtual void CalcThetaDesiredSmooth();
		virtual void StepBrake(mReal & ctrl_des, mReal curr);
		void Startup();
		void Shutdown();
		void StepStatus();
		void StepCommand();
		bool LinkDependentComponents();
		bool ReadConfig(const char * filename);
		M3BaseStatus * GetBaseStatus(){return status.mutable_base();}
		M3MinJerkTrajectory jerk_joint;
		M3PID pid_theta;
		M3PID pid_theta_gc;
		M3TimeSlew q_slew;
		M3TimeSlew q_on_slew;
		M3TimeSlew tq_on_slew;
		M3TimeSlew pwm_on_slew;
		M3TimeSlew brake_off_slew;
		mReal tq_switch, q_switch, pwm_switch;
		M3JointStatus status;
		M3JointCommand command;
		M3JointParam param;
		M3Actuator * act;
		M3Transmission * trans;
		M3Joint * cpj;   //coupled joint component (present in derived classes only)
		M3CtrlSimple * ctrl_simple;		
		string act_name;
		string ctrl_simple_name;
		int mode_last;
		int tmp_cnt;
		int pwr_on_last;
		int brake_off_cmd;
		int brake_type;
		bool disable_pwm_ramp;
		int pnt_cnt;
};


}

#endif


