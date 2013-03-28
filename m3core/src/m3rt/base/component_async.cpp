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

#include "m3rt/base/component_async.h"
#include "m3rt/base/m3rt_def.h"
#include "m3rt/base/component_factory.h"

#include "inttypes.h"

namespace m3rt{
	
using namespace std;


#ifdef __RTAI__

///////////////////////////////////////////////////////

void *async_io_thread(void * arg)
{
  M3ComponentAsync * aio = (M3ComponentAsync *)arg;    
 
  M3_INFO("Starting async thread %d.\n", num_asyncs);
  
  RT_TASK *task;
  
  char buffer [50];
	int n;
	n=sprintf (buffer, "%d",num_asyncs);
	
	// note this will only handle up to 1000 components
	char c_name [7] = "M3CXXX";
	char s_name [7] = "M3SXXX";
	char t_name [7] = "M3TXXX";
	
	n = MIN(3,n);
	for (int i = 0; i < n; i++)
	{
	  c_name[i+3] = buffer[i];
	  s_name[i+3] = buffer[i];
	  t_name[i+3] = buffer[i];
	}
	
  
  task = rt_task_init_schmod(nam2num(t_name), 0, 0, 0, SCHED_FIFO, 0xFF);
  if (task==NULL)
  {
	  M3_ERR("Failed to create RT-TASK M3ASY\n",0);	  
	  return 0;
  }
  
  rt_allow_nonroot_hrt();
	
  mlockall(MCL_CURRENT | MCL_FUTURE);
  
  rt_make_soft_real_time();
  
  aio->cmd_mutex = rt_typed_sem_init(nam2num(c_name), 1, BIN_SEM);
  aio->status_mutex = rt_typed_sem_init(nam2num(s_name), 1, BIN_SEM);
  
  aio->initializing = false;
    
   while (!aio->IsStopping())
   {     
	rt_sem_wait( aio->cmd_mutex );		
	aio->GetCommandAsync()->CopyFrom(*(aio->GetCommandShared()));
	aio->GetParamAsync()->CopyFrom(*(aio->GetParamShared()));
	rt_sem_signal( aio->cmd_mutex ); 
          
        aio->StepAsync();         
      	
	rt_sem_wait( aio->status_mutex );	
	aio->GetStatusShared()->CopyFrom(*(aio->GetStatusAsync()));	
	rt_sem_signal( aio->status_mutex );
    }    
     
   if (aio->cmd_mutex != NULL)
  {
	rt_sem_delete(aio->cmd_mutex);
	aio->cmd_mutex = NULL;
  }
  
  if (aio->status_mutex != NULL)
  {
	rt_sem_delete(aio->status_mutex);
	aio->status_mutex = NULL;
  }  
    
}


void M3ComponentAsync::Startup()
{
	
	stop_thread = false;
	initializing = true;

	
	rc = rt_thread_create((void*)async_io_thread, (void*)this, 1000000);
	
	//M3_DEBUG("rc: %d",(int)rc);
	
	while(IsInitializing())
	  usleep(100000);
	  //M3_DEBUG("waiting for init\n");
	
		  
	if (!cmd_mutex)
	{
		M3_ERR("rt Unable to create the cmd_mutex semaphore.\n",0);
		stop_thread = true;
		return;
	}
	
	
	  
	if (!status_mutex)
	{
		M3_ERR("rt Unable to create the status_mutex semaphore.\n",0);
		stop_thread = true;
		return;
	}
	
	
	num_asyncs++;
}

void M3ComponentAsync::Shutdown()
{            
      if (rc)
      {
	stop_thread = true;	
	rt_thread_join(rc);	
	rc = NULL;
      }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool M3ComponentAsync::ReadConfig(const char * filename)
{	
	int val;
	mReal mval;
	YAML::Node doc;

	if (!M3Component::ReadConfig(filename))
		return false;
	GetYamlDoc(filename, doc);
	
	
	return true;
}

bool M3ComponentAsync::LinkDependentComponents()
{
	
	return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void M3ComponentAsync::StepStatus()
{	
      
	rt_sem_wait( status_mutex );	
	GetStatusThread()->CopyFrom(*(GetStatusShared()));	
	rt_sem_signal( status_mutex );
			
}

void M3ComponentAsync::StepCommand()
{			
    
	rt_sem_wait( cmd_mutex );		
	GetCommandShared()->CopyFrom(*(GetCommand()));
	GetParamShared()->CopyFrom(*(GetParam()));
	rt_sem_signal( cmd_mutex ); 
        
  
}

#endif

}