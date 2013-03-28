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

#include <joint_zlift_shm.h>
#include "m3rt/base/component_factory.h"


namespace m3{
	
using namespace m3rt;
using namespace std;


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
M3BaseStatus * M3JointZLiftShm::GetBaseStatus()
{
	return status.mutable_base();
}

void  M3JointZLiftShm::Startup()
{  
  M3CompShm::Startup();
  
  sds_status_size = sizeof(M3JointZLiftShmSdsStatus);
  sds_cmd_size = sizeof(M3JointZLiftShmSdsCommand);  
  
  memset(&status_to_sds, 0, sds_status_size);
  
}

void M3JointZLiftShm::ResetCommandSds(unsigned char * sds)
{
  
  memset(sds,0,sizeof(M3JointZLiftShmSdsCommand));
  
}


size_t M3JointZLiftShm::GetStatusSdsSize()
{
	return sds_status_size;
}

size_t M3JointZLiftShm::GetCommandSdsSize()
{
	return sds_cmd_size;
}

void M3JointZLiftShm::SetCommandFromSds(unsigned char * data)
{
  
  M3JointZLiftShmSdsCommand * sds = (M3JointZLiftShmSdsCommand *) data;
    request_command();
   memcpy(&command_from_sds, sds, GetCommandSdsSize()); 
    release_command();    
    
    int64_t dt = GetBaseStatus()->timestamp()-command_from_sds.timestamp; // microseconds
    bool shm_timeout = ABS(dt) > (timeout*1000);    
       
    
  
    
  if (zlift != NULL)
  {
    if (shm_timeout)
    {
      ((M3JointCommand*)zlift->GetCommand())->set_ctrl_mode(JOINT_MODE_OFF);      
    } else {
	if (command_from_sds.control_mode == JOINT_MODE_ROS_OFF)
	{
	  ((M3JointCommand*)zlift->GetCommand())->set_ctrl_mode(JOINT_MODE_OFF);
	}
	else if (command_from_sds.control_mode == JOINT_MODE_ROS_THETA)
	{
	  ((M3JointCommand*)zlift->GetCommand())->set_ctrl_mode(JOINT_MODE_THETA);
	}
	else if (command_from_sds.control_mode == JOINT_MODE_ROS_THETA_GC)        
	{
  	  ((M3JointCommand*)zlift->GetCommand())->set_ctrl_mode(JOINT_MODE_THETA_GC);	  
	}
	else
	{
  	  ((M3JointCommand*)zlift->GetCommand())->set_ctrl_mode(JOINT_MODE_OFF);	  
	}

	zlift->SetDesiredPos(command_from_sds.position);
	zlift->SetDesiredPosDot(command_from_sds.velocity);
	zlift->SetSlewRate(command_from_sds.velocity);	  
	zlift->SetDesiredStiffness(command_from_sds.stiffness);
	((M3JointCommand*)zlift->GetCommand())->set_smoothing_mode(command_from_sds.smoothing_mode);
    }
  }
  
  /*M3_DEBUG("-----------\n");
  M3_DEBUG("theta: %f\n", ((M3JointCommand*)zlift->GetCommand())->q_desired());
  M3_DEBUG("mode: %d\n", (int)((M3JointCommand*)zlift->GetCommand())->ctrl_mode());  
  M3_DEBUG("-----------\n");*/
  
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void M3JointZLiftShm::SetSdsFromStatus(unsigned char * data)
{  
  status_to_sds.timestamp = GetBaseStatus()->timestamp(); 
  
  /*status_to_sds.x = ((M3OmnibaseStatus*)omnibase->GetStatus())->global_position(0);
  status_to_sds.y = ((M3OmnibaseStatus*)omnibase->GetStatus())->global_position(1);
  status_to_sds.yaw = ((M3OmnibaseStatus*)omnibase->GetStatus())->global_position(2);
  
  status_to_sds.x_dot = ((M3OmnibaseStatus*)omnibase->GetStatus())->global_velocity(0);
  status_to_sds.y_dot = ((M3OmnibaseStatus*)omnibase->GetStatus())->global_velocity(1);
  status_to_sds.yaw_dot = ((M3OmnibaseStatus*)omnibase->GetStatus())->global_velocity(2);*/
  
  
  if (zlift)
  {
    status_to_sds.position = zlift->GetPos();			
    status_to_sds.velocity = zlift->GetPosDot();
    status_to_sds.effort = zlift->GetForce();      
    status_to_sds.calibrated = zlift->IsEncoderCalibrated();
  }
    
  
  
  M3JointZLiftShmSdsStatus * sds = (M3JointZLiftShmSdsStatus *) data;
  request_status();  
  memcpy(sds, &status_to_sds, GetStatusSdsSize());  
  release_status();

}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool M3JointZLiftShm::LinkDependentComponents()
{
	tmp_cnt = 0;
	
	if (zlift_name.size()!=0)
	{		
		zlift = (M3JointZLift*)factory->GetComponent(zlift_name);
		if (zlift==NULL)
		{
			M3_ERR("M3JointZLift component %s declared for M3JointZLiftShm but could not be linked\n",
					zlift_name.c_str());
		    return false;
		}
	} else {
	    return false;
	}
	
	return true;
}

bool M3JointZLiftShm::ReadConfig(const char * filename)
{	
	if (!M3CompShm::ReadConfig(filename))
		return false;

	YAML::Node doc;	
	GetYamlDoc(filename, doc);	
		
	doc["zlift_component"] >> zlift_name;
		
	doc["timeout"] >> timeout;	
	
	return true;
}

} // namespace
   
