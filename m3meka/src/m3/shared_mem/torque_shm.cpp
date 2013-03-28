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

#include <m3/shared_mem/torque_shm.h>
#include "m3rt/base/component_factory.h"


namespace m3{
	
using namespace m3rt;
using namespace std;


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
M3BaseStatus * M3TorqueShm::GetBaseStatus()
{
	return status.mutable_base();
}

void  M3TorqueShm::Startup()
{  
  M3CompShm::Startup();
  
  sds_status_size = sizeof(M3TorqueShmSdsStatus);
  sds_cmd_size = sizeof(M3TorqueShmSdsCommand);  
  
  for (int i = 0; i < bot->GetNdof(RIGHT_ARM); i++)
  {
      status.mutable_right_arm()->add_theta(0);
      status.mutable_right_arm()->add_thetadot(0);
      status.mutable_right_arm()->add_torque(0);
      status.mutable_right_arm()->add_ctrl_mode(JOINT_ARRAY_MODE_OFF);
      command.mutable_right_arm()->add_tq_desired(0);
  }  
  for (int i = 0; i < 6; i++)
    status.mutable_right_arm()->add_loadx6(0);
  
  for (int i = 0; i < bot->GetNdof(LEFT_ARM); i++)
  {
      status.mutable_left_arm()->add_theta(0);
      status.mutable_left_arm()->add_thetadot(0);
      status.mutable_left_arm()->add_torque(0);
      status.mutable_left_arm()->add_ctrl_mode(JOINT_ARRAY_MODE_OFF);
      command.mutable_left_arm()->add_tq_desired(0);
  }  
  for (int i = 0; i < 6; i++)
    status.mutable_left_arm()->add_loadx6(0);
  
  for (int i = 0; i < bot->GetNdof(TORSO); i++)
  {   
      status.mutable_torso()->add_theta(0);
      status.mutable_torso()->add_thetadot(0);
      status.mutable_torso()->add_torque(0);
      status.mutable_torso()->add_ctrl_mode(JOINT_ARRAY_MODE_OFF);
      command.mutable_torso()->add_tq_desired(0);
  } 
  for (int i = 0; i < 6; i++)
    status.mutable_torso()->add_loadx6(0);
  
  for (int i = 0; i < bot->GetNdof(HEAD); i++)
  {
      status.mutable_head()->add_theta(0);
      status.mutable_head()->add_thetadot(0);
      status.mutable_head()->add_torque(0);
      status.mutable_head()->add_ctrl_mode(JOINT_ARRAY_MODE_OFF);
      command.mutable_head()->add_tq_desired(0);
  }  
  for (int i = 0; i < 6; i++)
    status.mutable_head()->add_loadx6(0);
}

void M3TorqueShm::ResetCommandSds(unsigned char * sds)
{
  
  memset(sds,0,sizeof(M3TorqueShmSdsCommand));
  
}


size_t M3TorqueShm::GetStatusSdsSize()
{
	return sds_status_size;
}

size_t M3TorqueShm::GetCommandSdsSize()
{
	return sds_cmd_size;
}

void M3TorqueShm::SetCommandFromSds(unsigned char * data)
{
  M3TorqueShmSdsCommand * sds = (M3TorqueShmSdsCommand *) data;
    request_command();
   memcpy(&command_from_sds, sds, GetCommandSdsSize()); 
    release_command();
       
    command.set_timestamp(command_from_sds.timestamp);
    
    int64_t dt = GetBaseStatus()->timestamp()-command.timestamp(); // microseconds
    bool shm_timeout = ABS(dt) > (timeout*1000);    
       
  for (int i = 0; i < bot->GetNdof(RIGHT_ARM); i++)
  { 
    command.mutable_right_arm()->set_tq_desired(i, command_from_sds.right_arm.tq_desired[i]);
    if (shm_timeout)
    {      
      bot->DisableTorqueShmRightArm();
    } else{
      bot->EnableTorqueShmRightArm();
      bot->SetTorqueSharedMem_mNm(RIGHT_ARM, i, command.right_arm().tq_desired(i));
    }
  }
  for (int i = 0; i < bot->GetNdof(LEFT_ARM); i++)
  { 
    command.mutable_left_arm()->set_tq_desired(i, command_from_sds.left_arm.tq_desired[i]);
    if (shm_timeout)
    {
      bot->DisableTorqueShmLeftArm();  
    } else{
      bot->EnableTorqueShmLeftArm();
      bot->SetTorqueSharedMem_mNm(LEFT_ARM, i, command.left_arm().tq_desired(i));    
    }
  }
  for (int i = 0; i < bot->GetNdof(TORSO); i++)
  { 
    command.mutable_torso()->set_tq_desired(i, command_from_sds.torso.tq_desired[i]);
    if (shm_timeout)
    {
      bot->DisableTorqueShmTorso();
    } else{
      bot->EnableTorqueShmTorso();
      bot->SetTorqueSharedMem_mNm(TORSO, i, command.torso().tq_desired(i));    
    }
  }
  
  
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void M3TorqueShm::SetSdsFromStatus(unsigned char * data)
{  
  status_to_sds.timestamp = GetBaseStatus()->timestamp(); 
  
  for (int i = 0; i < bot->GetNdof(RIGHT_ARM); i++)
  {    
    status.mutable_right_arm()->set_theta(i, bot->GetThetaDeg(RIGHT_ARM,i));
    status.mutable_right_arm()->set_thetadot(i, bot->GetThetaDotDeg(RIGHT_ARM,i));
    status.mutable_right_arm()->set_torque(i, bot->GetTorque_mNm(RIGHT_ARM,i));
    status.mutable_right_arm()->set_ctrl_mode(i, bot->GetMode(RIGHT_ARM,i));
    status_to_sds.right_arm.theta[i] = status.right_arm().theta(i);
    status_to_sds.right_arm.thetadot[i] = status.right_arm().thetadot(i);
    status_to_sds.right_arm.torque[i] = status.right_arm().torque(i);
    status_to_sds.right_arm.ctrl_mode[i] = status.right_arm().ctrl_mode(i);
  }
  if (right_loadx6)
  {
    for (int i = 0; i < 6; i++)
    {
      status.mutable_right_arm()->set_loadx6(i, right_loadx6->GetWrench(i));
      status_to_sds.right_arm.wrench[i] = right_loadx6->GetWrench(i);
    }

  }
  for (int i = 0; i < bot->GetNdof(LEFT_ARM); i++)
  {    
    status.mutable_left_arm()->set_theta(i, bot->GetThetaDeg(LEFT_ARM,i));
    status.mutable_left_arm()->set_thetadot(i, bot->GetThetaDotDeg(LEFT_ARM,i));
    status.mutable_left_arm()->set_torque(i, bot->GetTorque_mNm(LEFT_ARM,i));
    status.mutable_left_arm()->set_ctrl_mode(i, bot->GetMode(LEFT_ARM,i));
    status_to_sds.left_arm.theta[i] = status.left_arm().theta(i);
    status_to_sds.left_arm.thetadot[i] = status.left_arm().thetadot(i);
    status_to_sds.left_arm.torque[i] = status.left_arm().torque(i);
    status_to_sds.left_arm.ctrl_mode[i] = status.left_arm().ctrl_mode(i);
  }
  if (left_loadx6)
  {
    for (int i = 0; i < 6; i++)
    {
      status.mutable_left_arm()->set_loadx6(i, left_loadx6->GetWrench(i));
      status_to_sds.left_arm.wrench[i] = left_loadx6->GetWrench(i);
    }
  }
  for (int i = 0; i < bot->GetNdof(TORSO); i++)
  {    
    status.mutable_torso()->set_theta(i, bot->GetThetaDeg(TORSO,i));
    status.mutable_torso()->set_thetadot(i, bot->GetThetaDotDeg(TORSO,i));
    status.mutable_torso()->set_torque(i, bot->GetTorque_mNm(TORSO,i));
    status.mutable_torso()->set_ctrl_mode(i, bot->GetMode(TORSO,i));
    status_to_sds.torso.theta[i] = status.torso().theta(i);
    status_to_sds.torso.thetadot[i] = status.torso().thetadot(i);
    status_to_sds.torso.torque[i] = status.torso().torque(i);
    status_to_sds.torso.ctrl_mode[i] = status.torso().ctrl_mode(i);
  }
  
  for (int i = 0; i < bot->GetNdof(HEAD); i++)
  {    
    status.mutable_head()->set_theta(i, bot->GetThetaDeg(HEAD,i));
    status.mutable_head()->set_thetadot(i, bot->GetThetaDotDeg(HEAD,i));
    status.mutable_head()->set_torque(i, bot->GetTorque_mNm(HEAD,i));
    status.mutable_head()->set_ctrl_mode(i, bot->GetMode(HEAD,i));
    status_to_sds.head.theta[i] = status.head().theta(i);
    status_to_sds.head.thetadot[i] = status.head().thetadot(i);
    status_to_sds.head.torque[i] = status.head().torque(i);
    status_to_sds.head.ctrl_mode[i] = status.head().ctrl_mode(i);
  }
  
  M3TorqueShmSdsStatus * sds = (M3TorqueShmSdsStatus *) data;
  request_status();  
  memcpy(sds, &status_to_sds, GetStatusSdsSize());  
  release_status();
	

}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool M3TorqueShm::LinkDependentComponents()
{
	if (humanoid_name.size()!=0)
	{		
		bot=(M3Humanoid*)factory->GetComponent(humanoid_name);
		
		if (bot==NULL)
		{
			M3_ERR("M3Humanoid component %s declared for M3TorqueShm but could not be linked\n",
					humanoid_name.c_str());
		    return false;
		}		
		right_loadx6=(M3LoadX6*)factory->GetComponent(right_loadx6_name); //May be null if not on this robot model
		left_loadx6=(M3LoadX6*)factory->GetComponent(left_loadx6_name);//May be null if not on this robot model
	}
	return true;
}

bool M3TorqueShm::ReadConfig(const char * filename)
{
  if (!M3CompShm::ReadConfig(filename))
		return false;
  
	YAML::Node doc;	
	GetYamlDoc(filename, doc);	
	
	doc["humanoid_component"] >> humanoid_name;
	
	try{
	  doc["right_loadx6_component"] >>right_loadx6_name;	
	}
	catch(YAML::KeyNotFound& e)
	{
	  right_loadx6_name="";
	}
	
	try{
	  doc["left_loadx6_component"] >>left_loadx6_name;	
	}
	catch(YAML::KeyNotFound& e)
	{
	  left_loadx6_name="";
	}
	
	doc["timeout"] >> timeout;	
	
	return true;
}
}
   
