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

#ifndef M3_ACTUATOR_H
#define M3_ACTUATOR_H

#include "m3rt/base/component.h"
#include "actuator.pb.h"
#include "actuator_ec.pb.h"
#include "actuator_ec.h"
#include "sensor.h"
#include "motor.h"
#include "../toolbox/toolbox.h"
#include "../toolbox/dfilter.h"
#include <google/protobuf/message.h>



namespace m3
{
	using namespace std;

	
class M3Actuator : public m3rt::M3Component
{
	public:
		M3Actuator(): m3rt::M3Component(CALIB_PRIORITY),ecc(NULL),ignore_bounds(false),use_i_torque_ctrl(false),overload_cnt(0),old_ts(0),encoder_calib_req(1),old_ts_rtai(0),old_ticks(0),old_is_calibrated(false),tmp_cnt(0),torque_shift(1.0),sensor_fault_theta_cnt(0),sensor_fault_torque_cnt(0)
		{
			RegisterVersion("default",DEFAULT);	//RBL
			RegisterVersion("iss",ISS);		//ISS. Updated safety thresholds to use motor model.
			RegisterVersion("iq",IQ);
			
		}
		google::protobuf::Message * GetCommand(){return &command;}
		google::protobuf::Message * GetStatus(){return &status;}
		google::protobuf::Message * GetParam(){return &param;}
	public:
		M3ActuatorParamPID *	ParamPIDTorque(){return param.mutable_pid_torque();}
		void SetDesiredPwm(int val){command.set_pwm_desired(val);}
		void SetDesiredControlMode(ACTUATOR_MODE val){ command.set_ctrl_mode(val);}
		void SetDesiredTorque(mReal val){ command.set_tq_desired(val);}
		void SetDesiredCurrent(mReal val){ command.set_i_desired(val);}
		void SetBrakeOff(bool off){command.set_brake_off(off);}
		virtual bool IsMotorPowerSlewedOn(){return ecc->IsMotorPowerSlewedOn();}
		
		int GetAmpControlInput(){return amp_control_input;}
		mReal GetDesiredTorque(){return command.tq_desired();}
		mReal GetMotorTemp(){return status.motor_temp();}
		mReal GetAmpTemp(){return status.amp_temp();}
		mReal GetAmbientTemp(){return status.ambient_temp();}
		mReal GetCaseTemp(){return status.case_temp();}
		mReal GetExtTemp(){return status.ext_temp();}
		mReal GetPower(){return status.power();}
		mReal GetCurrent(){return status.current();}
		mReal GetThetaDeg(){ return status.theta();}
		mReal GetThetaRad(){ return DEG2RAD(status.theta());}
		mReal GetThetaDotDeg(){return status.thetadot();}
		mReal GetThetaDotRad(){return DEG2RAD(status.thetadot());}
		mReal GetThetaDotDotDeg(){return status.thetadotdot();}
		mReal GetThetaDotDotRad(){return DEG2RAD(status.thetadotdot());}
		int64_t GetTimestamp(){return status.base().timestamp();}
		mReal GetTorque(){return status.torque();}
		mReal GetTorqueDot(){return status.torquedot();}
		int GetPwmCmd(){return status.pwm_cmd();}
		int GetFlags(){return status.flags();}
		
		virtual int GetTicks(){
			if (!GetActuatorEc()) return 0;
			unsigned int low=(unsigned short)((M3ActuatorEcStatus*)ecc->GetStatus())->qei_on();
			unsigned int high=((unsigned short)((M3ActuatorEcStatus*)ecc->GetStatus())->qei_rollover()<<16);
			int ticks = high|low;
			return ticks;
		}
		
		virtual void SetZeroEncoder(){if (GetActuatorEc()) ecc->SetZeroEncoder();}
		virtual void ClrZeroEncoder(){if (GetActuatorEc()) ecc->ClrZeroEncoder();}
		
		virtual void SetLimitSwitchNegZeroEncoder(){if (GetActuatorEc()) ecc->SetLimitSwitchNegZeroEncoder();}
		virtual void ClrLimitSwitchNegZeroEncoder(){if (GetActuatorEc()) ecc->ClrLimitSwitchNegZeroEncoder();}
		virtual bool IsMotorPowerOn()     {if (!GetActuatorEc()) return false; return ecc->IsMotorPowerOn();}
		virtual bool IsLimitSwitchPosOn() {if (!GetActuatorEc()) return false; return ecc->IsLimitSwitchPosOn();}
		virtual bool IsLimitSwitchNegOn() {if (!GetActuatorEc()) return false; return ecc->IsLimitSwitchNegOn();}
		virtual bool IsEncoderCalibrated(){if (!encoder_calib_req) return true; if (!GetActuatorEc()) return false; return ecc->IsEncoderCalibrated();}		
		int mNmToTicks(mReal x){return tq_sense.mNmToTicks(x);}
		M3ActuatorEc * GetActuatorEc(){return ecc;}
	protected:
		enum {DEFAULT, ISS, IQ};		
		M3AngleSensor q_sense;
		M3TorqueSensor tq_sense;
		M3TempSensor ex_sense;
		M3TempSensor at_sense;
		M3CurrentSensor i_sense;
		M3MotorModel motor;
		M3TimeAvg amp_temp_avg;
		M3JointFilter angle_df;
		M3DFilter torquedot_df;
		M3DitherToInt pwm_dither;
		M3PID pid_torque;
		string config_filename;
		bool ReadConfig(const char * filename);
		void Startup();
		void Shutdown();
		void StepStatus();
		void StepCommand();
		M3ActuatorStatus status;
		M3ActuatorCommand command;
		M3ActuatorParam param;
		M3BaseStatus * GetBaseStatus(){return status.mutable_base();}
		bool LinkDependentComponents();
		void StepOverloadDetect();
		string ecc_name;
		M3ActuatorEc * ecc; 	//Corresponding EtherCAT component 
		int tmp_cnt;
		bool ignore_bounds;
		bool safe_pwm_limit; 
		bool encoder_calib_req;
		bool use_i_torque_ctrl;
		int overload_cnt;
		mReal max_motor_temp; //V-DEFAULT
		mReal max_current;//V-DEFAULT
		unsigned long long old_ts;
		long long old_ts_rtai;
		int old_ticks;
		bool old_is_calibrated;
		mReal torque_shift;
		int sensor_fault_theta_cnt;
		int sensor_fault_torque_cnt;
		int amp_control_input;
		int pnt_cnt;
		int last_theta_err;
		int last_torque_err;
		bool disable_vertx_check;
		int vertx_timeout_limit_ms;
};


}

#endif


