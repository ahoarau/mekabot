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

#include "m3rt/rt_system/rt_service.h"

#include <stdio.h>
#include <unistd.h>
#include <string>
#include <signal.h>

static bool svc_thread_active=false;
static bool svc_thread_end=false;
//////////////////////////////////////////////////////////////////////////////////////
//Monitor services for errors, etc
static void* service_thread(void * arg)
{
	M3RtService * svc = (M3RtService *)arg;
	svc_thread_active=true;
	svc_thread_end=false;
	while(!svc_thread_end)
	{
		usleep(100000);
		if (svc->IsDataServiceError())
		{
		      m3rt::M3_ERR("Detected dropped M3RtDataService. Stopping RtSystem;\n");
		      svc->RemoveRtSystem();
		      break;
		 }
	}	
	m3rt::M3_INFO("Exiting M3 Service Thread\n",0);
	svc_thread_active=false;
	return 0;
}
//////////////////////////////////////////////////////////////////////////////////////

M3RtService::~M3RtService()
{
	Shutdown();
}

bool M3RtService::Startup()
{
#ifdef __RTAI__
	rt_allow_nonroot_hrt();
#endif
	
	if (svc_thread_active)
	{
		m3rt::M3_ERR("M3RtService thread already active\n",0);
		return false;
	}
	pthread_create((pthread_t *)&hlt, NULL, (void *(*)(void *))service_thread, (void*)this);
	usleep(100000);
	if (!svc_thread_active)
	{
		m3rt::M3_ERR("Unable to start M3RtService\n",0);
		return false;
	}
	return svc_thread_active;
}

void M3RtService::Shutdown()
{
	svc_thread_end=true;
	pthread_join((pthread_t)hlt, NULL);

	if (svc_thread_active) m3rt::M3_WARN("M3RtService thread did not shut down correctly\n");
	
	RemoveRtSystem();
}

//////////////////////////////////////////////////////////////////////////////////////
int M3RtService::AttachRtSystem()
{
	if(rt_system!=NULL)
		return ++num_rtsys_attach;
	
	rt_system = new m3rt::M3RtSystem(&factory);

	if (!rt_system->Startup())
	{
		m3rt::M3_INFO("Startup of M3RtSystem failed. Shutting down\n",0);
		rt_system->Shutdown();
		delete rt_system;
		rt_system=NULL;
		num_rtsys_attach=0;
		return 0;
	}

	return ++num_rtsys_attach;
}

int M3RtService::RemoveRtSystem()
{
	
	if (rt_system==NULL)
	{
		num_rtsys_attach=0;
		return 0;
	}
	if (num_rtsys_attach>1)
		return --num_rtsys_attach;
	for (int i=0; i<data_services.size(); i++)
	{
		if (ports[i])
			RemoveDataService(ports[i]);
	}
	if (IsRosServiceRunning())
	  RemoveRosService();
	if (!IsDataServiceRunning())
	{		
		RemoveLogService();
		bool success=rt_system->Shutdown();
		delete rt_system;
		rt_system=NULL;
		if (!success)
		{
			m3rt::M3_INFO("Failure in RTSystem shutdown. Exiting now. Restart server\n");
			exit(0);
		}
					
	}
	num_rtsys_attach=0;
	return 0;
}

//////////////////////////////////////////////////////////////////////////////////////
bool M3RtService::AttachLogService(char * name, char * path, double freq,int page_size,int verbose)
{
	m3rt::M3_DEBUG("Attaching M3RtLogService: %s\n",name);
	if (rt_system==NULL || IsLogServiceRunning())
		return false;
	log_service = new m3rt::M3RtLogService(rt_system,std::string(name),std::string(path),freq,page_size,verbose);
	for(int i=0;i<log_components.size();i++)
		log_service->AddComponent(log_components[i]);
	if (!log_service->Startup())
	{
		m3rt::M3_INFO("M3RtLogService %s failed to start\n",name);
		log_service->Shutdown();
		delete log_service;
		log_service=NULL;
		log_components.clear();
		m3rt::M3_INFO("Shutting down RTSystem due to RtLogService startup failure\n");
		RemoveRtSystem();
		return false;
	}
	rt_system->AttachLogService(log_service);
	return true;
}

bool M3RtService::RemoveLogService()
{
	if (IsLogServiceRunning())
	{
		rt_system->RemoveLogService();
		while (rt_system->logging); // wait in case we were stepping
		log_service->Shutdown();
		delete log_service;
		log_components.clear();
		log_service=NULL;
		return true;
	}
	else
		return false;
}

bool M3RtService::IsDataServiceError()
{
  for (int i=0; i<data_services.size(); i++)
    if (data_services[i] && data_services[i]->data_thread_error)
	 return true;
  return false;
}
bool  M3RtService::IsDataServiceRunning()
{
	bool running=false;
	for (int i=0; i<data_services.size(); i++)
	{		
		if (data_services[i] != NULL)
			running = true;
	}
	return running;
}

//////////////////////////////////////////////////////////////////////////////////////
int M3RtService::AttachDataService()
{
	if (rt_system==NULL)
		return -1;
	data_services.push_back(new m3rt::M3RtDataService(rt_system,next_port));
	if (!data_services.back()->Startup())
	{
		data_services.back()->Shutdown();
		delete data_services.back();
		data_services.pop_back();
		m3rt::M3_INFO("Shutting down RTSystem due to DataService startup failure\n");
		RemoveRtSystem();
		return -1;
	}
	ports.push_back(next_port);
	next_port++;	
	return next_port-1;
}

bool M3RtService::RemoveDataService(int port)
{	
	int idx=-1;
	for (int i=0; i<data_services.size(); i++)
	{
		if (data_services[i] && (ports[i] == port))
		{					
			idx=i;	
			data_services[i]->Shutdown();		
			delete data_services[i];			
			data_services[i] = NULL;			
			ports[i] = NULL;			
			break;
		}
	}
	if (idx!=-1)
	{
		data_services.erase(data_services.begin()+idx);
		ports.erase(ports.begin()+idx);
		return true;
	}
	return false;
}
//////////////////////////////////////////////////////////////////////////////////////

bool M3RtService::ClientSubscribeStatus(const char * name, int port)
{
	if (IsDataServiceRunning())
	{
		for (int i=0; i<data_services.size(); i++)
		{
			if (data_services[i] && ports[i] == port)
			{
				data_services[i]->ClientSubscribeStatus(std::string(name));
				return true;
			}
		}
		
	}
	return false;
}

int M3RtService::GetNumComponents()
{
	if (!rt_system)
		return 0;
	return rt_system->GetNumComponents();
}

const char *  M3RtService::GetComponentName(int idx)
{
	if (!rt_system )
		return "";
	return rt_system->GetComponentName(idx).c_str();
}

const char *  M3RtService::GetComponentType(int idx)
{
	if (!rt_system)
		return "";
	return rt_system->GetComponentType(idx).c_str();
}

int M3RtService::GetComponentIdx(const char * name)
{
	if (!rt_system)
		return -1;
	 return rt_system->GetComponentIdx(std::string(name));
}

int M3RtService::GetComponentState(const char * name)
{
	int idx=GetComponentIdx(name);
	if (idx>=0)
		return rt_system->GetComponentState(idx);
	return -1;
}

bool  M3RtService::PrettyPrintComponent(const char * name)
{
	int idx=GetComponentIdx(name);
	if (idx>=0)
	{
		rt_system->PrettyPrintComponent(idx);
		return true;
	}
	return false;
}

bool M3RtService::SetComponentStateOp(char * name)
{
	
	int idx=GetComponentIdx(name);
	
	if (idx>=0)
	{
		rt_system->SetComponentStateOp(idx);
		//printf("M3RtService Setting Op %s\n",name);
		return true;
	}
	return false;
}
bool M3RtService::SetComponentStateSafeOp(char * name)
{
	
	int idx=GetComponentIdx(name);
	
	if (idx>=0)
	{
		rt_system->SetComponentStateSafeOp(idx);
		//printf("M3RtService Setting SafeOp %s\n",name);
		return true;
	}
	return false;
}
bool M3RtService::PrettyPrintRtSystem()
{
	if (rt_system!=NULL)
	{
		rt_system->PrettyPrint();
		return true;
	}
	return false;
}

/*bool M3RtService::AddRosComponent(const char * name)
{
    if (rt_system==NULL)
	return false;
    
    if (!IsRosServiceRunning())
    {
	return false;
    }
    m3rt::M3_INFO("here!\n");
    return ros_service->AddComponent(name);
}*/

bool M3RtService::AttachRosService()
{
	
	return true;
}

bool M3RtService::RemoveRosService()
{
	return true;
}


