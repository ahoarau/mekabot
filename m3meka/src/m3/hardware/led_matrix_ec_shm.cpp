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

#include <led_matrix_ec_shm.h>
#include "m3rt/base/component_factory.h"


namespace m3{
	
using namespace m3rt;
using namespace std;


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
M3BaseStatus * M3LedMatrixEcShm::GetBaseStatus()
{
	return status.mutable_base();
}

void  M3LedMatrixEcShm::Startup()
{  
  M3CompShm::Startup();
  
  sds_status_size = sizeof(M3LedMatrixEcShmSdsStatus);
  sds_cmd_size = sizeof(M3LedMatrixEcShmSdsCommand);  
  
  memset(&status_to_sds, 0, sds_status_size);
  
}

void M3LedMatrixEcShm::ResetCommandSds(unsigned char * sds)
{
  
  memset(sds,0,sizeof(M3LedMatrixEcShmSdsCommand));
  
}


size_t M3LedMatrixEcShm::GetStatusSdsSize()
{
	return sds_status_size;
}

size_t M3LedMatrixEcShm::GetCommandSdsSize()
{
	return sds_cmd_size;
}

void M3LedMatrixEcShm::SetCommandFromSds(unsigned char * data)
{
  
  M3LedMatrixEcShmSdsCommand * sds = (M3LedMatrixEcShmSdsCommand *) data;
    request_command();
   memcpy(&command_from_sds, sds, GetCommandSdsSize()); 
    release_command();    
    
    int64_t dt = GetBaseStatus()->timestamp()-command_from_sds.timestamp; // microseconds
    bool shm_timeout = ABS(dt) > (timeout*1000);    
       
            
    
  if (led_matrix != NULL)
  {
    if (shm_timeout)
    {
      ((M3LedMatrixEcCommand*)led_matrix->GetCommand())->set_enable(0);      
    } else {
	for (int i = 0; i < led_matrix->GetNumRows(); i++)
	{	  
	  M3LedMatrixEcRGBRow * row = ((M3LedMatrixEcCommand*)led_matrix->GetCommand())->mutable_row(i);
	  for (int j = 0; j < led_matrix->GetNumCols(); j++)
	  {
	    row->mutable_column(j)->set_r(command_from_sds.r[i][j]);
	    row->mutable_column(j)->set_g(command_from_sds.g[i][j]);
	    row->mutable_column(j)->set_b(command_from_sds.b[i][j]);	    
	  }
	}
	((M3LedMatrixEcCommand*)led_matrix->GetCommand())->set_enable(command_from_sds.enable);
	
    }
  }
  
  /*M3_DEBUG("-----------\n");
  M3_DEBUG("theta: %f\n", ((M3JointCommand*)zlift->GetCommand())->q_desired());
  M3_DEBUG("mode: %d\n", (int)((M3JointCommand*)zlift->GetCommand())->ctrl_mode());  
  M3_DEBUG("-----------\n");*/
  
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void M3LedMatrixEcShm::SetSdsFromStatus(unsigned char * data)
{  
  status_to_sds.timestamp = GetBaseStatus()->timestamp(); 
  
  /*status_to_sds.x = ((M3OmnibaseStatus*)omnibase->GetStatus())->global_position(0);
  status_to_sds.y = ((M3OmnibaseStatus*)omnibase->GetStatus())->global_position(1);
  status_to_sds.yaw = ((M3OmnibaseStatus*)omnibase->GetStatus())->global_position(2);
  
  status_to_sds.x_dot = ((M3OmnibaseStatus*)omnibase->GetStatus())->global_velocity(0);
  status_to_sds.y_dot = ((M3OmnibaseStatus*)omnibase->GetStatus())->global_velocity(1);
  status_to_sds.yaw_dot = ((M3OmnibaseStatus*)omnibase->GetStatus())->global_velocity(2);*/
  
      
  
  
  M3LedMatrixEcShmSdsStatus * sds = (M3LedMatrixEcShmSdsStatus *) data;
  request_status();  
  memcpy(sds, &status_to_sds, GetStatusSdsSize());  
  release_status();

}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool M3LedMatrixEcShm::LinkDependentComponents()
{
	tmp_cnt = 0;
	
	if (led_matrix_name.size()!=0)
	{		
		led_matrix = (M3LedMatrixEc*)factory->GetComponent(led_matrix_name);
		if (led_matrix==NULL)
		{
			M3_ERR("M3LedMatrixEc component %s declared for M3LedMatrixEcShm but could not be linked\n",
					led_matrix_name.c_str());
		    return false;
		}
	} else {
	    return false;
	}
	
	return true;
}

bool M3LedMatrixEcShm::ReadConfig(const char * filename)
{	
	if (!M3CompShm::ReadConfig(filename))
		return false;

	YAML::Node doc;	
	GetYamlDoc(filename, doc);	
		
	doc["led_matrix_component"] >> led_matrix_name;
		
	doc["timeout"] >> timeout;	
	
	return true;
}

} // namespace
   
