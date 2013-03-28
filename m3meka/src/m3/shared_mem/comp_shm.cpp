/* 
M3 -- Meka Robotics Real-Time Control System
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

#include <m3/shared_mem/comp_shm.h>

namespace m3
{

bool M3CompShm::ReadConfig(const char * filename)
{
	YAML::Node doc;
	
	if (!M3Component::ReadConfig(filename)) return false;
	GetYamlDoc(filename, doc);
	
	doc["shm_id"] >> shm_id;
	
	return true;
}


void  M3CompShm::StepStatus()
{
	if (!shm)
	{
		SetStateSafeOp();
		return;
	}
	if (!IsStateError())
		SetSdsFromStatus(shm->status);
}

void  M3CompShm::StepCommand()
{
      
	if (!shm) return;
	if (!IsStateOp())
		ResetCommandSds(shm->cmd);
	else	
		SetCommandFromSds(shm->cmd);
	
}

void  M3CompShm::Startup()
{
  SetStateSafeOp();
  
  command_sem = rt_typed_sem_init(nam2num((shm_id+"C").c_str()), 1, BIN_SEM | FIFO_Q );
  status_sem = rt_typed_sem_init(nam2num((shm_id+"S").c_str()), 1, BIN_SEM | FIFO_Q );
  
  shm = (M3Sds*)rt_shm_alloc(nam2num((shm_id+"M").c_str()),sizeof(M3Sds),USE_VMALLOC);  
  memset(shm,0,sizeof(M3Sds));
  
}

void  M3CompShm::Shutdown()
{  
  rt_shm_free(nam2num((shm_id+"M").c_str()));
  rt_sem_delete(command_sem);
  rt_sem_delete(status_sem);
}

}	