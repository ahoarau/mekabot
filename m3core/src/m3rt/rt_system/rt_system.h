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

#ifndef RT_SYSTEM_H
#define RT_SYSTEM_H

#include "m3rt/base/m3rt_def.h"
#include "m3rt/base/m3ec_def.h"
#include "m3rt/base/toolbox.h"
#include "m3rt/base/component.h"
#include "m3rt/base/component_ec.h"
#include "m3rt/base/component_factory.h"
#include "m3rt/base/component_base.pb.h" 
#include "m3rt/rt_system/rt_log_service.h"
//#include "m3rt/rt_system/rt_ros_service.h"
#include <string>
#include <vector>

#ifdef __RTAI__
#include <rtai.h>
#include "rtai_sem.h"
#else
#include <semaphore.h>
#include <pthread.h>
#include <sys/time.h>
#endif
 

namespace m3rt
{
using namespace std;


class M3RtSystem
{
public:
	M3RtSystem(M3ComponentFactory * f):log_service(NULL),
			   shm_ec(0),shm_sem(0),ext_sem(NULL),sync_sem(0),factory(f),logging(false),
			   safeop_required(false){GOOGLE_PROTOBUF_VERIFY_VERSION;}
	friend class M3RtDataService;
	~M3RtSystem();
	bool Startup();
	bool StartupComponents();
	bool Shutdown();
	bool Step(bool safeop_only);
	void PrettyPrint();
	void PrettyPrintComponents();
	void PrettyPrintComponent(int idx);
	void PrettyPrintComponentNames();
	M3Component * 	GetComponent(string name){return factory->GetComponent(name);}
	M3Component *  	GetComponent(int idx){return factory->GetComponent(idx);}
	string  	GetComponentName(int idx){return factory->GetComponentName(idx);}
	string  	GetComponentType(int idx){return factory->GetComponentType(idx);}
	int 		GetNumComponents(){return factory->GetNumComponents();}	
	int 		GetComponentIdx(string name){return factory->GetComponentIdx(name);}
	int			GetComponentState(int idx);	
	bool SetComponentStateOp(int idx);
	bool SetComponentStateSafeOp(int idx);
	bool IsOperational(){return !safeop_required;}
#ifdef __RTAI__
	int GetEcCounter(){return shm_ec->counter;}
#else
	int GetEcCounter(){return 0;}
#endif
	void SetFactory(M3ComponentFactory * f){factory=f;}
	void AttachLogService(M3RtLogService * l){log_service=l;}
	
	void RemoveLogService(){log_service=NULL;M3_DEBUG("Log service stopped at %d\n",log_service);}
	bool ParseCommandFromExt(M3CommandAll & msg);  //Must be thread safe
	bool SerializeStatusToExt(M3StatusAll & msg, vector<string>& names); //Must be thread safe
	bool logging;
	int over_step_cnt;
protected:
#ifdef __RTAI__
	SEM * GetExtSem(){return ext_sem;}
#else
	sem_t * GetExtSem(){return ext_sem;}
#endif
	bool ReadConfigEc(const char * filename);
	bool ReadConfigRt(const char * filename);
private:
	
	void CheckComponentStates();
	M3ComponentFactory * factory;
	M3EcSystemShm *  shm_ec;
	bool safeop_required;
	vector<M3ComponentEc *>	m3ec_list;
	vector<M3Component *>	m3rt_list;
#ifdef __RTAI__
	SEM * shm_sem;
	SEM * sync_sem;
	SEM * ext_sem;	
	RTIME last_cycle_time;
#else
	sem_t * shm_sem;
	sem_t * sync_sem;
	sem_t * ext_sem;
	long long last_cycle_time;
#endif
	M3RtLogService * log_service;
	
	vector<int> idx_map_ec;
	vector<int> idx_map_rt;
	long hst;
	double test;
	
};


}
#endif


