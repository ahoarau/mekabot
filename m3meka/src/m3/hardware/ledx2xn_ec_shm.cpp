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

#include <ledx2xn_ec_shm.h>
#include "m3rt/base/component_factory.h"


namespace m3{
	
using namespace m3rt;
using namespace std;


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
M3BaseStatus * M3LedX2XNEcShm::GetBaseStatus()
{
	return status.mutable_base();
}

void  M3LedX2XNEcShm::Startup()
{  
  M3CompShm::Startup();
  
  sds_status_size = sizeof(M3LedX2XNEcShmSdsStatus);
  sds_cmd_size = sizeof(M3LedX2XNEcShmSdsCommand);  
  
  memset(&status_to_sds, 0, sds_status_size);
  
}

void M3LedX2XNEcShm::ResetCommandSds(unsigned char * sds)
{
  
  memset(sds,0,sizeof(M3LedX2XNEcShmSdsCommand));
  
}


size_t M3LedX2XNEcShm::GetStatusSdsSize()
{
	return sds_status_size;
}

size_t M3LedX2XNEcShm::GetCommandSdsSize()
{
	return sds_cmd_size;
}

void M3LedX2XNEcShm::SetCommandFromSds(unsigned char * data)
{
  
  M3LedX2XNEcShmSdsCommand * sds = (M3LedX2XNEcShmSdsCommand *) data;
    request_command();
   memcpy(&command_from_sds, sds, GetCommandSdsSize()); 
    release_command();    
    
    int64_t dt = GetBaseStatus()->timestamp()-command_from_sds.timestamp; // microseconds
    bool shm_timeout = ABS(dt) > (timeout*1000);    

/*	if (tmp_cnt++ == 200)
	{
		M3_DEBUG("hi\n");
		tmp_cnt = 0;
	
	}*/

       
  if (led_x2xn != NULL)
  {
    if (shm_timeout)
    {
/*	if (tmp_cnt++ == 200)
	{
		M3_DEBUG("hi\n");
		tmp_cnt = 0;
	
	}*/

      ((M3LedX2XNEcCommand*)led_x2xn->GetCommand())->set_enable_a(0);
      ((M3LedX2XNEcCommand*)led_x2xn->GetCommand())->set_enable_b(0);            
    } else {
/*	if (tmp_cnt++ == 200)
	{
		M3_DEBUG("here\n");
		tmp_cnt = 0;
	
	}*/
	((M3LedX2XNEcCommand*)led_x2xn->GetCommand())->set_enable_a(command_from_sds.enable_a);
        ((M3LedX2XNEcCommand*)led_x2xn->GetCommand())->set_enable_b(command_from_sds.enable_b);  
	for (int i = 0; i < NUM_PER_BRANCH; i++)
	{	  
		((M3LedX2XNEcCommand*)led_x2xn->GetCommand())->mutable_branch_a()->set_r(i,command_from_sds.r_a[i]);
		((M3LedX2XNEcCommand*)led_x2xn->GetCommand())->mutable_branch_a()->set_g(i,command_from_sds.g_a[i]);
		((M3LedX2XNEcCommand*)led_x2xn->GetCommand())->mutable_branch_a()->set_b(i,command_from_sds.b_a[i]);

		((M3LedX2XNEcCommand*)led_x2xn->GetCommand())->mutable_branch_b()->set_r(i,command_from_sds.r_b[i]);
		((M3LedX2XNEcCommand*)led_x2xn->GetCommand())->mutable_branch_b()->set_g(i,command_from_sds.g_b[i]);
		((M3LedX2XNEcCommand*)led_x2xn->GetCommand())->mutable_branch_b()->set_b(i,command_from_sds.b_b[i]);
	}

    }
  }
    
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void M3LedX2XNEcShm::SetSdsFromStatus(unsigned char * data)
{  
  status_to_sds.timestamp = GetBaseStatus()->timestamp(); 
  

  
  
  M3LedX2XNEcShmSdsStatus * sds = (M3LedX2XNEcShmSdsStatus *) data;
  request_status();  
  memcpy(sds, &status_to_sds, GetStatusSdsSize());  
  release_status();

}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool M3LedX2XNEcShm::LinkDependentComponents()
{
	tmp_cnt = 0;
	
	if (led_x2xn_name.size()!=0)
	{		
		led_x2xn = (M3LedX2XNEc*)factory->GetComponent(led_x2xn_name);
		if (led_x2xn==NULL)
		{
			M3_ERR("M3LedX2XNEc component %s declared for M3LedX2XNEcShm but could not be linked\n",
					led_x2xn_name.c_str());
		    return false;
		}
	} else {
	    return false;
	}
	
	return true;
}

bool M3LedX2XNEcShm::ReadConfig(const char * filename)
{	
	if (!M3CompShm::ReadConfig(filename))
		return false;

	YAML::Node doc;	
	GetYamlDoc(filename, doc);	
		
	doc["led_x2xn_component"] >> led_x2xn_name;
		
	doc["timeout"] >> timeout;
	
	return true;
}

} // namespace
   
