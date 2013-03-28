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

#ifndef M3_JOINT_ARRAY_H
#define M3_JOINT_ARRAY_H

#include "m3rt/base/component.h"
#include "m3/chains/joint_array.pb.h"
#include "m3/toolbox/toolbox.h"
#include <google/protobuf/message.h>
#include "m3/hardware/joint.h"
#include "m3/toolbox/trajectory.h"

namespace m3
{
	using namespace std;
	using namespace KDL;

///////////////////////////////////////////////////////////////////////////

class M3JointArray : public m3rt::M3Component
{
	public:
		M3JointArray(): m3rt::M3Component(CHAIN_PRIORITY)
		{
			RegisterVersion("default",DEFAULT);	//RBL
			RegisterVersion("iss",ISS);		//ISS. Safe as DEFAULT
		}
		google::protobuf::Message * GetCommand(){return &command;}
		google::protobuf::Message * GetStatus(){return &status;}
		google::protobuf::Message * GetParam(){return &param;}
		int GetNumDof(){return ndof;}
		M3Joint * GetJoint(int idx){return joints[idx];}		
		mReal GetThetaDeg(int i){return theta(i);}
		mReal GetThetaRad(int i){return DEG2RAD(theta(i));}
		mReal GetThetaDotDeg(int i){return thetadot(i);}
		mReal GetThetaDotRad(int i){return DEG2RAD(thetadot(i));}		
		mReal GetThetaDotDotDeg(int i){return thetadotdot(i);}
		mReal GetThetaDotDotRad(int i){return DEG2RAD(thetadotdot(i));}
		mReal GetThetaMaxDeg(int i){return joints[i]->GetThetaMaxDeg();}
		mReal GetThetaMinDeg(int i){return joints[i]->GetThetaMinDeg();}
		mReal GetTorque(int i){return torque(i);}
		
		string GetJointName(int i){return joint_names[i];}		
	protected:
		enum {DEFAULT, ISS};	
		bool ReadConfig(const char * filename);
		void Startup();
		void Shutdown();
		void StepStatus();
		void StepCommand();
		bool LinkDependentComponents();
	
		M3BaseStatus * GetBaseStatus(){return status.mutable_base();}
		M3JointArrayStatus status;
		M3JointArrayCommand command;
		M3JointArrayParam param;			
		int ndof;
		vector<M3Joint *> joints;		
		vector<string> joint_names;
		void StepJointTrajectory();
		M3JointTrajectory traj;
		JntArray traj_des;
		JntArray theta;
		JntArray thetadot;
		JntArray thetadotdot;
		JntArray torque;
		JntArray tw_torque;
		vector<bool> traj_active;
		int tmp_cnt;	
};

}

#endif


