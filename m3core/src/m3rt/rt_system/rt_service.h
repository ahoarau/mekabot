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

#ifndef RT_SERVICE_H
#define RT_SERVICE_H

#include "m3rt/base/m3rt_def.h"
#include "m3rt/base/toolbox.h"
#include "m3rt/base/component_factory.h"
#include "m3rt/rt_system/rt_data_service.h"
#include "m3rt/rt_system/rt_log_service.h"
#include "m3rt/rt_system/rt_system.h"
#include <pthread.h>

#ifdef __RTAI__
#include <rtai.h>
#include "rtai_sem.h"
#else
#include <semaphore.h>
#endif


//No m3rt namespace for swig-ability
	
class M3RtService{
public:
	M3RtService():rt_system(NULL),log_service(NULL),svc_task(NULL),next_port(10000),num_rtsys_attach(0){factory.Startup();}
	~M3RtService();
	bool Startup();
	void Shutdown();
	int AttachRtSystem(); //Return number attached: 0=error
	int RemoveRtSystem();//Return number attached: 0=all removed
	bool IsRtSystemOperational(){if (rt_system==NULL) return false; return rt_system->IsOperational();}
	bool SetComponentStateSafeOp(char * name);
	bool SetComponentStateOp(char * name);
	int AttachDataService();
	bool AttachRosService();
	bool RemoveRosService();
	bool RemoveDataService(int port);
	bool AttachLogService(char * name, char * path, double freq,int page_size,int verbose);
	//bool AddRosComponent(const char * name);
	bool AddLogComponent(char * name){log_components.push_back(name);}
	bool RemoveLogService();
	bool IsDataServiceRunning();
	bool IsLogServiceRunning(){return log_service!=NULL;}
	bool IsRosServiceRunning(){return false;}
	bool IsRtSystemRunning(){return rt_system !=NULL;}
	int GetNumComponents();
	const char *  GetComponentName(int idx);
	const char *  GetComponentType(int idx);
	int GetComponentState(const char * name);
	int GetComponentIdx(const char * name);
	bool PrettyPrintComponent(const char * name);
	bool PrettyPrintRtSystem();
	bool ClientSubscribeStatus(const char * name, int port);
	bool IsDataServiceError();
private:
  	int hlt;
	m3rt::M3RtSystem  * rt_system;
	m3rt::M3ComponentFactory factory; //Can only create one instance of this.
	std::vector<m3rt::M3RtDataService*> data_services;
	m3rt::M3RtLogService *log_service;
	std::vector<std::string> log_components;
#ifdef __RTAI__
	RT_TASK *svc_task;
#else
	int * svc_task; // to preserve initializer
#endif
	std::vector<int> ports;
	int next_port;
	int num_rtsys_attach;
};


#endif
