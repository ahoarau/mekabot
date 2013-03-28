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

#include "rt_data_service.h"
#include "m3rt/base/m3rt_def.h"

#ifdef __RTAI__
#include <rtai.h>
#include <rtai_shm.h>
#include <rtai_sched.h>
#include <rtai_nam2num.h>
#include <rtai_sem.h>
#include <rtai_lxrt.h>
#endif

namespace m3rt
{
	
///////////////////////////////////////////////////////////


void data_thread(void * arg)
{
	stringstream ss;
	string rt_name;	
	M3RtDataService * svc = (M3RtDataService *)arg;
	svc->data_thread_active=true;
	svc->data_thread_end=false;
#ifdef __RTAI__
	RT_TASK *task;

	//Need to consider multiple threads, name conflict
	ss << "M3DSV" << svc->instances;
	ss >> rt_name;
	task = rt_task_init_schmod(nam2num(rt_name.c_str()), 0, 0, 0, SCHED_FIFO, 0xF); 
	rt_allow_nonroot_hrt();
	svc->instances++;
	if (task==NULL)
	{
		M3_ERR("Failed to create M3RtDataService RT Task\n",0);
		return;
	}
	mlockall(MCL_CURRENT | MCL_FUTURE);
#endif
	if (!svc->StartServer()) //blocks until connection
	{
		svc->data_thread_active=false;
		return;
	}
	while(!svc->data_thread_end)
	{
		if (!svc->Step())
		{
		   svc->data_thread_error=true;		   
		   break;
		}
		usleep(10000); //100hz
	}	
#ifdef __RTAI__
	rt_task_delete(task);
#endif
	if (svc->data_thread_error)
	  M3_INFO("Exiting M3 Data Server Thread Prematurely\n",0);
	else
	  M3_INFO("Exiting M3 Data Server Thread\n",0);
	svc->data_thread_active=false;
	return;
}
////////////////////////////////////////////////////////////

int M3RtDataService::instances = 0;

bool M3RtDataService::Startup()
{
	if (data_thread_active)
	{
		M3_ERR("M3RtDataService thread already active\n",0);
		return false;
	}
	M3_INFO("Startup of M3RtDataService, port %d...\n",portno);
	ext_sem=sys->GetExtSem();
#ifdef __RTAI__
	hdt=rt_thread_create((void*)data_thread,this,10000);  // wait until thread starts
#else
	pthread_create((pthread_t *)&hdt, NULL, (void *(*)(void *))data_thread, (void*)this);
#endif
	usleep(100000);
	if (!data_thread_active)
	{
		M3_ERR("Unable to start M3RtDataSevice\n",0);
		return false;
	}
	return data_thread_active;
}

void M3RtDataService::Shutdown()
{
	M3_INFO("Shutdown of M3RtDataService , port %d\n",portno);
	data_thread_end=true;
#ifdef __RTAI__
	rt_thread_join(hdt);
#else
	pthread_join((pthread_t)hdt, NULL);
#endif
	if (data_thread_active)
		M3_WARN("M3RtDataService thread did not shut down correctly\n");
	server.Shutdown();
	
}


void M3RtDataService::ClientSubscribeStatus(string name)
{
	for (int i=0;i<status_names.size();i++)
		if (name.compare(status_names[i])==0)
			return;
	status_names.push_back(name);
}
	
bool M3RtDataService::Step()
{
	int nw,nr,res;
	res=server.ReadStringFromPort(sread, nr);
	if (res==-1) //error
	  return false;
	//If receive cmd data, parse and reply with status data (or just reply with status data if no cmd)
	if(res)
	{
		M3CommandAll * c=NULL;
		if (nr>0)
		{
			c = new M3CommandAll;
			c->ParseFromString(sread);
		}
		
#ifdef __RTAI__
		rt_sem_wait(ext_sem);
#else
		sem_wait(ext_sem);
#endif
		if (c!=NULL)
		{
			sys->ParseCommandFromExt(*c);
			delete c;
		}
		if (!sys->SerializeStatusToExt(status,status_names))
		{
#ifdef __RTAI__
			rt_sem_signal(ext_sem);
#else
			sem_post(ext_sem);
#endif
			return false;
		}
#ifdef __RTAI__
		rt_sem_signal(ext_sem);
#else
			sem_post(ext_sem);
#endif
		status.SerializeToString(&swrite);
		nw=server.WriteStringToPort(swrite);
		if(nw<0)
			return false;
	}
	return true;
}
////////////////////////////////////////////////////////////
}