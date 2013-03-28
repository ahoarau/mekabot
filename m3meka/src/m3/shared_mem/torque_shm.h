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

#ifndef M3_TORQUE_SHM_H
#define M3_TORQUE_SHM_H

#include "m3/shared_mem/comp_shm.h"
#include "m3/shared_mem/torque_shm.pb.h"
#include "m3/robots/humanoid.h"
#include "m3/hardware/loadx6.h"
#include "torque_shm_sds.h"


namespace m3{
using namespace std;


class M3TorqueShm : public  m3::M3CompShm{
	public:
		M3TorqueShm(): sds_status_size(0),sds_cmd_size(0),M3CompShm()
		{		  
		  RegisterVersion("default",DEFAULT);
		  RegisterVersion("iss",ISS);		  
		}
		google::protobuf::Message * GetCommand(){return &command;}
		google::protobuf::Message * GetStatus(){return &status;}
		google::protobuf::Message * GetParam(){return &param;}
	protected:
		bool ReadConfig(const char * filename);
		//M3ShmStatus * GetShmStatus(){return status.mutable_shm();}		
		size_t GetStatusSdsSize();
		size_t GetCommandSdsSize();
		void SetCommandFromSds(unsigned char * data);
		void SetSdsFromStatus(unsigned char * data);
		bool LinkDependentComponents();
		void ResetCommandSds(unsigned char * sds);
		void Startup();
	
		enum {DEFAULT, ISS};
		M3BaseStatus * GetBaseStatus();		
		M3TorqueShmCommand command;
		M3TorqueShmParam param;
		M3TorqueShmStatus status;
		M3Humanoid * bot;
		M3LoadX6 * right_loadx6;
		M3LoadX6 * left_loadx6;
		M3TorqueShmSdsCommand command_from_sds;
		M3TorqueShmSdsStatus status_to_sds;
		int sds_status_size;
		int sds_cmd_size;	
		string humanoid_name,right_loadx6_name,left_loadx6_name;
		int64_t timeout;
		int tmp_cnt;
};

}
#endif


