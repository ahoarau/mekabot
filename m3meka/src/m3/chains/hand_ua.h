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

#ifndef M3_HAND_UA_H
#define M3_HAND_UA_H

#include "m3rt/base/component.h"
#include "m3/chains/hand.h"
#include "m3/chains/hand_ua.pb.h"
#include "m3/toolbox/toolbox.h"
#include <google/protobuf/message.h>

namespace m3
{
	using namespace std;
	
///////////////////////////////////////////////////////////////////////////

class M3HandUA : public m3rt::M3Component
{
	public:
		M3HandUA() : m3rt::M3Component(CHAIN_PRIORITY), hand(NULL), ndof_finger(3)
		{
			RegisterVersion("default",DEFAULT);	//RBL
			RegisterVersion("iss",ISS);		//ISS. Safe as DEFAULT
		}
		
		google::protobuf::Message * GetCommand(){return &command;}
		google::protobuf::Message * GetStatus(){return &status;}
		google::protobuf::Message * GetParam(){return &param;}
		mReal GetThetaThumb(int dof){return status.theta_thumb(dof);}
		mReal GetThetaIndex(int dof){return status.theta_index(dof);}
		mReal GetThetaRing(int dof){return status.theta_ring(dof);}
		mReal GetThetaPinky(int dof){return status.theta_pinky(dof);}
	private:
		int tmp_cnt;
		enum {DEFAULT, ISS};	
		bool ReadConfig(const char * filename);
		void Startup();
		void Shutdown();
		void StepStatus();
		void StepCommand();
		bool LinkDependentComponents();
		M3BaseStatus * GetBaseStatus(){return status.mutable_base();}
		M3HandUAStatus status;
		M3HandUACommand command;
		M3HandUAParam param;			
		int ndof_finger;		
		M3Hand * hand;
		string hand_name;
};

}

#endif



