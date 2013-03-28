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

#ifndef M3OMNIBASE_SHM_H
#define M3OMNIBASE_SHM_H

#include <m3rt/base/component_shm.h>
#include "omnibase_shm.pb.h"
#include "omnibase_shm_sds.h"
#include "m3/vehicles/omnibase.h"


namespace m3{
using namespace std;
using namespace m3rt;


class M3OmnibaseShm : public  m3::M3CompShm{
	public:
		M3OmnibaseShm(): sds_status_size(0),sds_cmd_size(0),M3CompShm(),omnibase(NULL),pwr(NULL),
				  max_linear_acceleration(0.), max_rotation_acceleration(0.), max_linear_velocity(0.), max_rotation_velocity(0.)
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
		M3OmnibaseShmCommand command;
		M3OmnibaseShmParam param;
		M3OmnibaseShmStatus status;
		M3Omnibase * omnibase;
		M3Pwr * pwr;
		
		M3OmnibaseShmSdsCommand command_from_sds;
		M3OmnibaseShmSdsStatus status_to_sds;
		int sds_status_size;
		int sds_cmd_size;	
		
		string omni_name, pwr_name;
		int64_t timeout;
		int tmp_cnt;		
		bool startup_motor_pwr_on;
		mReal max_linear_acceleration;
		mReal max_rotation_acceleration;
		mReal max_linear_velocity;
		mReal max_rotation_velocity;
};

}
#endif


