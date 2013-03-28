 /*************************************************************************
 * 
 * REDWOOD CONFIDENTIAL
 * Author: Aaron Edsinger
 * __________________
 * 
 *  [2012] - [+] Redwood Robotics Incorporated 
 *  All Rights Reserved.
 * 
 * All information contained herein is, and remains
 * the property of Redwood Robotics Incorporated and its suppliers,
 * if any.  The intellectual and technical concepts contained
 * herein are proprietary to Redwood Robotics Incorporated
 * and its suppliers and may be covered by U.S. and Foreign Patents,
 * patents in process, and are protected by trade secret or copyright law.
 * Dissemination of this information or reproduction of this material
 * is strictly forbidden unless prior written permission is obtained
 * from Redwood Robotics Incorporated.
 */
 

#ifndef M3_CTRL_SIMPLE_H
#define M3_CTRL_SIMPLE_H


#include <google/protobuf/message.h>
#include <m3rt/base/component.h>
#include "../toolbox/toolbox.h"
#include "../toolbox/dfilter.h"
#include "ctrl_simple.pb.h"
#include "actuator.h"
#include "actuator.pb.h"

namespace m3
{
	using namespace std;
	using namespace m3rt;
	

class M3CtrlSimple: public  m3rt::M3Component
{
	public:
		M3CtrlSimple(): m3rt::M3Component(CONTROL_PRIORITY),	pnt_cnt(0),
																act(NULL)
		{
			RegisterVersion("default",DEFAULT);	
		}
		
	//Setters
		void SetDesiredControlMode(CTRL_SIMPLE_MODE x){command.set_ctrl_mode(x);}
		void SetDesiredCurrent(mReal i){command.set_desired_current(i);}
		void SetDesiredTheta(mReal q){command.set_desired_theta(q);}
		void SetDesiredTorque(mReal tq){command.set_desired_torque(tq);}
		void SetDesiredStiffness(mReal s){command.set_desired_stiffness(s);}
		void SetTorqueGravity(mReal tq){status.set_torque_gravity(tq);}
		
	//Access to Status Messages
		M3BaseStatus *			StatusBase()	{return status.mutable_base();}
		M3CtrlSimpleStatusCommand *	StatusCommand()	{return status.mutable_command();}
		M3ActuatorStatus *		StatusActuator(){return status.mutable_actuator();}
		
	//Access to Traj Params
		M3ParamTrajectory * 	ParamTrajCurrent()	{return param.mutable_traj_current();}
		M3ParamTrajectory * 	ParamTrajTheta()	{return param.mutable_traj_theta();}
		M3ParamTrajectory * 	ParamTrajTorque()	{return param.mutable_traj_torque();}
		M3ParamPID *		ParamPidTheta()		{return param.mutable_pid_theta();}
		M3ParamPID * 		ParamPidTorque()	{return param.mutable_pid_torque();}

	//Conversions
		virtual	bool	IsMotorPowerOn()		{if (!GetActuator()) return false; return act->IsMotorPowerOn();}
		M3Actuator * GetActuator()	{return act;}

	// getters
		mReal	GetJointTheta()			{return act->GetThetaRad();}
		mReal	GetJointThetaDot()		{return act->GetThetaDotRad();}
		mReal	GetJointTorque()		{return act->GetTorque();}
		mReal	GetJointTorqueDot()		{return act->GetTorqueDot();}
		
		int64_t GetTimestamp()			{return GetBaseStatus()->timestamp();}
		
		void ResetIntegrators(){pid_torque.ResetIntegrator();pid_theta.ResetIntegrator();}
		
		google::protobuf::Message * GetCommand(){return &command;}
		google::protobuf::Message * GetStatus(){return &status;}
		google::protobuf::Message * GetParam(){return &param;}

	protected:

		// required:
		enum{DEFAULT};
		void Startup();
		void Shutdown();
		void StepStatus();
		void StepCommand();
		bool LinkDependentComponents();
		bool ReadConfig(const char * filename);
		M3BaseStatus * GetBaseStatus(){return status.mutable_base();}
		
		M3CtrlSimpleStatus	status;
		M3CtrlSimpleCommand	command;
		M3CtrlSimpleParam	param;
		
		
		M3PID	pid_theta;
		M3PID	pid_torque;		
		
		M3Actuator * act;
		string	act_name;
		CTRL_SIMPLE_MODE ctrl_mode_last;
		
		int pnt_cnt;
};

}

#endif


