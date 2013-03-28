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

#ifndef M3_HEAD_S2_CSP_CTRL_H
#define M3_HEAD_S2_CSP_CTRL_H

#include "m3rt/base/component.h"
#include "m3/toolbox/toolbox.h"
#include <google/protobuf/message.h>
#include "m3/robots/humanoid.h"
#include "m3/chains/head.h"
#include "m3/robot_ctrl/head_s2csp_ctrl.pb.h"



namespace m3
{
	using namespace std;
	using namespace KDL;
///////////////////////////////////////////////////////////////////////////
//Coordinated 7DOF Smooth Pursuit Controller. Will override the M3Humanoid command 
//J2 (head roll) is available to the client for expressive displays
class M3HeadS2CSPCtrl : public m3rt::M3Component
{
	public:
		M3HeadS2CSPCtrl(): joint_smooth(7),target_slew(3),theta_des(7),bot(NULL), head(NULL),m3rt::M3Component(ROBOT_CTRL_PRIORITY)
		{
			RegisterVersion("default",DEFAULT);	
		}
		google::protobuf::Message * GetCommand(){return &command;}
		google::protobuf::Message * GetStatus(){return &status;}
		google::protobuf::Message * GetParam(){return &param;}
	protected:
		enum {DEFAULT};
		bool ReadConfig(const char * filename);
		void Startup();
		void Shutdown();
		void StepStatus();
		void StepCommand();
		bool LinkDependentComponents();
		M3BaseStatus * GetBaseStatus(){return status.mutable_base();}
		M3HeadS2CSPCtrlStatus status;
		M3HeadS2CSPCtrlCommand command;
		M3HeadS2CSPCtrlParam param;			
		M3Humanoid * bot;
		M3Head * head;
		string bot_name,head_name;
		int tmp_cnt;	
		
		Frame wTe, eTw,wTh;
		Vector xe, xw,xh;
		vector<mReal> theta_des, theta_db, slew_des, origin;
		vector<M3TimeSlew> target_slew;
		vector<M3TimeAvg> joint_smooth;
};

}

#endif


