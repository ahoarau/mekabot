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

#include <omnibase_shm.h>
#include "m3rt/base/component_factory.h"


namespace m3{
	
using namespace m3rt;
using namespace std;


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
M3BaseStatus * M3OmnibaseShm::GetBaseStatus()
{
	return status.mutable_base();
}

void  M3OmnibaseShm::Startup()
{  
  M3CompShm::Startup();
  
  sds_status_size = sizeof(M3OmnibaseShmSdsStatus);
  sds_cmd_size = sizeof(M3OmnibaseShmSdsCommand);  
  
  memset(&status_to_sds, 0, sds_status_size);
  
}

void M3OmnibaseShm::ResetCommandSds(unsigned char * sds)
{
  
  memset(sds,0,sizeof(M3OmnibaseShmSdsCommand));
  
}


size_t M3OmnibaseShm::GetStatusSdsSize()
{
	return sds_status_size;
}

size_t M3OmnibaseShm::GetCommandSdsSize()
{
	return sds_cmd_size;
}

void M3OmnibaseShm::SetCommandFromSds(unsigned char * data)
{
  
  M3OmnibaseShmSdsCommand * sds = (M3OmnibaseShmSdsCommand *) data;
    request_command();
   memcpy(&command_from_sds, sds, GetCommandSdsSize()); 
    release_command();    
    
    int64_t dt = GetBaseStatus()->timestamp()-command_from_sds.timestamp; // microseconds
    bool shm_timeout = ABS(dt) > (timeout*1000);    
       
    if (pwr != NULL)
  {
    if (startup_motor_pwr_on)
	pwr->SetMotorEnable(true);      
  }
  
    
  if (omnibase != NULL)
  {
    if (shm_timeout)
    {
      ((M3OmnibaseCommand*)omnibase->GetCommand())->set_ctrl_mode(OMNIBASE_CTRL_OFF);
    } else {
        ((M3OmnibaseCommand*)omnibase->GetCommand())->set_ctrl_mode(OMNIBASE_CTRL_OPSPACE_TRAJ);
	((M3OmnibaseCommand*)omnibase->GetCommand())->set_traj_mode(OMNIBASE_TRAJ_JOYSTICK);
	((M3OmnibaseCommand*)omnibase->GetCommand())->set_joystick_x(command_from_sds.x_velocity);
	((M3OmnibaseCommand*)omnibase->GetCommand())->set_joystick_y(command_from_sds.y_velocity);
	((M3OmnibaseCommand*)omnibase->GetCommand())->set_joystick_yaw(command_from_sds.yaw_velocity);		
	((M3OmnibaseCommand*)omnibase->GetCommand())->set_joystick_button(0);
	
	((M3OmnibaseCommand*)omnibase->GetCommand())->set_max_linear_acceleration(max_linear_acceleration);
	((M3OmnibaseCommand*)omnibase->GetCommand())->set_max_rotation_acceleration(max_rotation_acceleration);
	((M3OmnibaseCommand*)omnibase->GetCommand())->set_max_linear_velocity(max_linear_velocity);
	((M3OmnibaseCommand*)omnibase->GetCommand())->set_max_rotation_velocity(max_rotation_velocity);
	
	
  
    }
  }
  
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void M3OmnibaseShm::SetSdsFromStatus(unsigned char * data)
{  
  status_to_sds.timestamp = GetBaseStatus()->timestamp(); 
  
  status_to_sds.x = ((M3OmnibaseStatus*)omnibase->GetStatus())->global_position(0);
  status_to_sds.y = ((M3OmnibaseStatus*)omnibase->GetStatus())->global_position(1);
  status_to_sds.yaw = DEG2RAD(((M3OmnibaseStatus*)omnibase->GetStatus())->global_position(2));
  
  status_to_sds.x_dot = ((M3OmnibaseStatus*)omnibase->GetStatus())->global_velocity(0);
  status_to_sds.y_dot = ((M3OmnibaseStatus*)omnibase->GetStatus())->global_velocity(1);
  status_to_sds.yaw_dot = DEG2RAD(((M3OmnibaseStatus*)omnibase->GetStatus())->global_velocity(2));
  
  bool calibrated = true;
  
  for (int i = 0; i < 4; i++)
  {
      //M3_DEBUG("uuu %d \n", i);
      if (!((M3OmnibaseStatus*)omnibase->GetStatus())->calibrated(i))
	calibrated = false;
  }
  
  status_to_sds.calibrated = calibrated;
  
  /*if (bot)
  {
    for (int i = 0; i < bot->GetNdof(RIGHT_ARM); i++)
    {          
      status_to_sds.right_arm.theta[i] = bot->GetThetaDeg(RIGHT_ARM,i);
      status_to_sds.right_arm.thetadot[i] = bot->GetThetaDotDeg(RIGHT_ARM,i);
      status_to_sds.right_arm.torque[i] = bot->GetTorque_mNm(RIGHT_ARM,i);      
    }
  }
    
  if (right_hand)
  { 
      for (int i = 0; i < right_hand->GetNumDof(); i++)
      {	
	status_to_sds.right_hand.theta[i] = right_hand->GetThetaDeg(i);
	status_to_sds.right_hand.thetadot[i] = right_hand->GetThetaDotDeg(i);
	status_to_sds.right_hand.torque[i] = right_hand->GetTorque(i);
      }
  }
  
  if (right_loadx6)
  {
    for (int i = 0; i < 6; i++)
    {      
      status_to_sds.right_loadx6.wrench[i] = right_loadx6->GetWrench(i);
    }
  }*/
  
  M3OmnibaseShmSdsStatus * sds = (M3OmnibaseShmSdsStatus *) data;
  request_status();  
  memcpy(sds, &status_to_sds, GetStatusSdsSize());  
  release_status();

}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool M3OmnibaseShm::LinkDependentComponents()
{
	tmp_cnt = 0;
	
	if (pwr_name.size()!=0)
	{		
	    
		pwr=(M3Pwr*)factory->GetComponent(pwr_name);
		
		if (pwr==NULL)
		{
			M3_WARN("M3Pwr component %s declared for M3TorqueShm but could not be linked\n",
					pwr_name.c_str());
		    //return false;
		}				
	}
	
	if (omni_name.size()!=0)
	{		
		omnibase = (M3Omnibase*)factory->GetComponent(omni_name);
		if (omnibase==NULL)
		{
			M3_ERR("M3Omnibase component %s declared for M3OmnibaseShm but could not be linked\n",
					omni_name.c_str());
		    return false;
		}
	} else {
	    return false;
	}
	
	return true;
}

bool M3OmnibaseShm::ReadConfig(const char * filename)
{	
	if (!M3CompShm::ReadConfig(filename))
		return false;

	YAML::Node doc;	
	GetYamlDoc(filename, doc);	
		
	doc["omnibase_component"] >> omni_name;
	
	try{
	  doc["pwr_component"] >> pwr_name;	 
	}
	catch(YAML::KeyNotFound& e)
	{	  
	  pwr_name="";
	}
	
	try{
	  doc["startup_motor_pwr_on"]>>startup_motor_pwr_on;
	}
	catch(YAML::KeyNotFound& e)
	{
	  startup_motor_pwr_on=false;
	}
		
	doc["timeout"] >> timeout;
	
	doc["max_linear_acceleration"] >> max_linear_acceleration;	
	doc["max_rotation_acceleration"] >> max_rotation_acceleration;	
	doc["max_linear_velocity"] >> max_linear_velocity;	
	doc["max_rotation_velocity"] >> max_rotation_velocity;	
	
	return true;
}

} // namespace
   
