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

#ifndef  M3RT_COMP_SHM_H
#define  M3RT_COMP_SHM_H
#include <rtai_registry.h>
#include <m3rt/base/component.h>
#include <m3rt/base/component_base.pb.h>
#include <m3/toolbox/toolbox.h>
#include <rtai.h>
#include <rtai_lxrt.h>
#include <rtai_shm.h>
#include <rtai_sched.h>
#include <rtai_nam2num.h>
#include <rtai_sem.h>

#include <rtai_malloc.h> 
#include "m3rt/base/m3rt_def.h"
#include "m3rt/base/m3ec_def.h"
#include "m3rt/base/toolbox.h"


namespace m3
{
////////////////////////////////////////////////////////////////
 
class M3CompShm: public M3Component{
	public:
		M3CompShm(int p=EC_PRIORITY):M3Component(p),shm(NULL),status_sem(NULL), command_sem(NULL){}
		
	protected:		
		virtual size_t GetStatusSdsSize()=0;
		virtual size_t GetCommandSdsSize()=0;		
		virtual void SetCommandFromSds(unsigned char * data)=0;
		virtual void SetSdsFromStatus(unsigned char * data)=0;	
		virtual bool ReadConfig(const char * filename);			
		virtual void Startup();
		virtual void Shutdown();
		virtual void StepStatus();
		virtual void StepCommand();		
		virtual void ResetCommandSds(unsigned char * sds){memset(sds,0,MAX_SDS_SIZE_BYTES);}
		void request_status(){rt_sem_wait(status_sem);}
		void release_status(){rt_sem_signal(status_sem);}
		void request_command(){rt_sem_wait(command_sem);}
		void release_command(){rt_sem_signal(command_sem);}
	private:
		M3Sds * shm;				
		SEM * status_sem;
		SEM * command_sem;	
		string shm_id;
};

}
#endif

