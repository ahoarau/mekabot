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

#ifndef M3HUMANOID_SHM_H
#define M3HUMANOID_SHM_H

#include <m3rt/base/component_shm.h>
#include "humanoid_shm.pb.h"
#include "humanoid_shm_sds.h"
#include "m3/robots/humanoid.h"
#include "m3/hardware/loadx6.h"
#include "m3/chains/hand.h"

namespace m3{
using namespace std;
using namespace m3rt;


class M3HumanoidShm : public  m3::M3CompShm{
	public:
		M3HumanoidShm(): sds_status_size(0),sds_cmd_size(0),M3CompShm(),bot(NULL),
		right_hand(NULL),right_loadx6(NULL),tmp_cnt(0),left_hand(NULL),left_loadx6(NULL),pwr(NULL)
		{		  
		  RegisterVersion("default",DEFAULT);		  
		}
		google::protobuf::Message * GetCommand(){return &command;}
		google::protobuf::Message * GetStatus(){return &status;}
		google::protobuf::Message * GetParam(){return &param;}
	protected:
		bool ReadConfig(const char * filename);		
		size_t GetStatusSdsSize();
		size_t GetCommandSdsSize();
		void SetCommandFromSds(unsigned char * data);
		void SetSdsFromStatus(unsigned char * data);
		bool LinkDependentComponents();
		void ResetCommandSds(unsigned char * sds);
		void Startup();		
		
		
		enum {DEFAULT};
		M3BaseStatus * GetBaseStatus();		
		M3HumanoidShmCommand command;
		M3HumanoidShmParam param;
		M3HumanoidShmStatus status;
		M3Humanoid * bot;
		M3LoadX6 * right_loadx6;
		M3Hand * right_hand;
		M3LoadX6 * left_loadx6;
		M3Hand * left_hand;
		M3Pwr * pwr;
		M3HumanoidShmSdsCommand command_from_sds;
		M3HumanoidShmSdsStatus status_to_sds;
		int sds_status_size;
		int sds_cmd_size;	
		string bot_name, right_hand_name, right_loadx6_name, left_hand_name, left_loadx6_name, pwr_name;
		int64_t timeout;
		int tmp_cnt;		
		bool startup_motor_pwr_on;
};

}
#endif


