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

#include "m3rt/rt_system/rt_system.h"
//#include "m3rt/base/m3ec_pdo_v1_def.h"

#ifdef __RTAI__
#include <rtai.h>
#include <rtai_sem.h>
#include <rtai_sched.h>
#include <rtai_nam2num.h>
#include <rtai_shm.h>
#include <rtai_malloc.h> 
#include <string>
#endif

namespace m3rt
{

static bool sys_thread_active;
static bool sys_thread_end;
static int step_cnt=0;

unsigned long long getNanoSec (void)  {
	struct timeval tp;
	struct timezone tzp;

	tzp.tz_minuteswest = 0;

	(void) gettimeofday(&tp, &tzp); //
	return 1000000000LL * ( long long ) tp.tv_sec + 
			1000LL * ( long long ) tp.tv_usec;
}

void * rt_system_thread(void * arg)
{
	M3RtSystem * m3sys = (M3RtSystem *)arg;
	sys_thread_end=false;
	
	
	//M3_INFO("Starting M3RtSystem real-time thread\n",0);
	if (!m3sys->StartupComponents())
	{
		sys_thread_active=false;
		return 0;
	}
#ifdef __RTAI__	
	RT_TASK *task;
	RTIME start, end, dt;
	M3_INFO("Beginning RTAI Initialization.\n");
	task = rt_task_init_schmod(nam2num("M3SYS"), 0, 0, 0, SCHED_FIFO, 0xFF);
	if (task==NULL)
	{
		m3rt::M3_ERR("Failed to create RT-TASK M3SYS\n",0);
		sys_thread_active=false;
		return 0;
	}
	M3_INFO("RT Task Scheduled.\n");
	rt_allow_nonroot_hrt();
	M3_INFO("Nonroot hrt initialized.\n");
	rt_task_use_fpu(task, 1);
	M3_INFO("Use fpu initialized.\n");
	mlockall(MCL_CURRENT | MCL_FUTURE);
	M3_INFO("Mem lock all initialized.\n");
	RTIME tick_period = nano2count(RT_TIMER_TICKS_NS + 200000); 
	RTIME now = rt_get_time();
	if (1)
		rt_make_hard_real_time();

	if (0)
	{
		rt_make_soft_real_time();
		M3_INFO("M3Sytem running in soft real time! For debugging only!!!\n");
	}
	M3_INFO("Hard real time initialized.\n");
#else	
	long long start, end, dt;	
#endif	
	sys_thread_active=true;
	//Give other threads a chance to load before starting
#ifdef __RTAI__
	rt_sleep(nano2count(1000000000));
	rt_task_make_periodic(task, now + tick_period, tick_period); 
#else	
	usleep(50000);
#endif
	M3_INFO("Periodic task initialized.\n");
	bool safeop_only = false;
	
	int tmp_cnt = 0;
	m3sys->over_step_cnt = 0;
		
	while(!sys_thread_end)
	{
#ifdef __RTAI__
		start = nano2count(rt_get_cpu_time_ns());
#else		
		start = getNanoSec();
#endif

		if (!m3sys->Step(safeop_only)) //This waits on m3ec.ko semaphore for timing
			break;
#ifdef __RTAI__
		end = nano2count(rt_get_cpu_time_ns());
		dt=end-start;

		//if (tmp_cnt++ == 1000)
		//{
		 // M3_INFO("%f\n",double(count2nano(dt)/1000));
		//  tmp_cnt = 0;
		//}
		/*
		Check the time it takes to run components, and if it takes longer
		than our period, make us run slower. Otherwise this task locks
		up the CPU.*/
		if (dt > tick_period && step_cnt>10) 
		{
			m3sys->over_step_cnt++;			
			m3rt::M3_DEBUG("WARNING: Previous period: %f ns overrun\n", (double)count2nano(dt)-(double)count2nano(tick_period));
			if (m3sys->over_step_cnt > 10)
			{
			  m3rt::M3_WARN("Step %d: Computation time of components is too long. Forcing all components to state SafeOp.\n",step_cnt);
			  m3rt::M3_WARN("Previous period: %f. New period: %f\n", (double)count2nano(tick_period));
			  tick_period=dt;
			  rt_task_make_periodic(task, end + tick_period,tick_period);
			  safeop_only = true;
			}
		} else {
		    if (m3sys->over_step_cnt > 0)
		      m3sys->over_step_cnt--;
		}
		
		rt_task_wait_period(); //No longer need as using sync semaphore of m3ec.ko
#else		
		end = getNanoSec();
		dt=end-start;
		usleep((RT_TIMER_TICKS_NS-((unsigned int)dt))/1000);
#endif
	}	
#ifdef __RTAI__
	rt_make_soft_real_time();
	rt_task_delete(task);
#endif
	sys_thread_active=false;
	return 0;
}

M3RtSystem::~M3RtSystem(){}

////////////////////////////////////////////////////////////////////////////////////////////

bool M3RtSystem::Startup()
{
	sys_thread_active=false;
	sys_thread_end=false;
	BannerPrint(60,"Startup of M3RtSystem");
#ifdef __RTAI__
	hst=rt_thread_create((void*)rt_system_thread, (void*)this, 1000000);
#else		
	pthread_create((pthread_t *)&hst, NULL, (void *(*)(void *))rt_system_thread, (void*)this);
#endif
	for (int i=0;i<50;i++)
	{
		usleep(100000); //Wait until enters hard real-time and components loaded. Can take some time if alot of components.
		if (sys_thread_active)
			break;
	}
	if (!sys_thread_active)
	{
		m3rt::M3_INFO("Startup of M3RtSystem thread failed.\n");
		return false;
	}
	//Debugging
	if (0)
	{
		/*M3_INFO("PDO Size M3ActPdoV1Cmd %d\n",(int)sizeof(M3ActPdoV1Cmd));
		M3_INFO("PDO Size M3ActPdoV1Status %d\n\n",(int)sizeof(M3ActPdoV1Status));
		
		M3_INFO("PDO Size M3ActX1PdoV1Cmd %d\n",(int)sizeof(M3ActX1PdoV1Cmd));
		M3_INFO("PDO Size M3ActX2PdoV1Cmd %d\n",(int)sizeof(M3ActX2PdoV1Cmd));
		M3_INFO("PDO Size M3ActX3PdoV1Cmd %d\n",(int)sizeof(M3ActX3PdoV1Cmd));
		M3_INFO("PDO Size M3ActX4PdoV1Cmd %d\n\n",(int)sizeof(M3ActX4PdoV1Cmd));
		
		M3_INFO("PDO Size M3ActX1PdoV1Status %d\n",(int)sizeof(M3ActX1PdoV1Status));
		M3_INFO("PDO Size M3ActX2PdoV1Status %d\n",(int)sizeof(M3ActX2PdoV1Status));
		M3_INFO("PDO Size M3ActX3PdoV1Status %d\n",(int)sizeof(M3ActX3PdoV1Status));
		M3_INFO("PDO Size M3ActX4PdoV1Status %d\n",(int)sizeof(M3ActX4PdoV1Status));*/
		
	}
	return true;
}

bool M3RtSystem::Shutdown()
{
	M3_INFO("Begin shutdown of M3RtSystem...\n");
	//Stop RtSystem thread
	sys_thread_end=true;
#ifdef __RTAI__
	rt_thread_join(hst);
#else	
	pthread_join((pthread_t)hst, NULL);
#endif
	if (sys_thread_active)
	{
		m3rt::M3_WARN("M3RtSystem thread did not shutdown correctly\n");
		return false;
	}
#ifdef __RTAI__
	if (shm_ec!=NULL)
#endif
	{
		//Send out final shutdown command to EC slaves
		for(int i=0;i<GetNumComponents();i++)
			GetComponent(i)->Shutdown();
#ifdef __RTAI__
		rt_shm_free(nam2num(SHMNAM_M3MKMD));
#endif
	}
	if (ext_sem!=NULL)
	{
#ifdef __RTAI__
		rt_sem_delete(ext_sem);
#else
		delete ext_sem;
#endif	
		ext_sem=NULL;
	}
	shm_ec=NULL;
	shm_sem=NULL;
	sync_sem=NULL;
	factory->ReleaseAllComponents();
	M3_INFO("Shutdown of M3RtSystem complete\n");
	return true;
}

////////////////////////////////////////////////////////////////////////////////////////////
	
bool M3RtSystem::StartupComponents()
{
#ifdef __RTAI__
	if (shm_ec = (M3EcSystemShm*) rtai_malloc (nam2num(SHMNAM_M3MKMD),1))
		M3_PRINTF("Found %d active M3 EtherCAT slaves\n",shm_ec->slaves_active);
	else
	{
		M3_ERR("Rtai_malloc failure for SHMNAM_M3KMOD\n",0);
		return false;
	}
	shm_sem=(SEM*)rt_get_adr(nam2num(SEMNAM_M3LSHM));
	if (!shm_sem)
	{
		M3_ERR("Unable to find the SEMNAM_M3LSHM semaphore.\n",0);
		return false;
	}
	
	sync_sem=(SEM*)rt_get_adr(nam2num(SEMNAM_M3SYNC));
	if (!sync_sem)
	{
		M3_ERR("Unable to find the SYNCSEM semaphore.\n",0);
		return false;
	}
	ext_sem=rt_typed_sem_init(nam2num(SEMNAM_M3LEXT), 1, BIN_SEM);
#else	
	ext_sem = new sem_t();
	sem_init(ext_sem,1,1);
#endif	
	if (!ext_sem)
	{
		M3_ERR("Unable to find the M3LEXT semaphore.\n",0);
		return false;
	}
	M3_INFO("Reading component config files ...\n"); 
#ifdef __RTAI__	
	if (!ReadConfigEc(M3_CONFIG_FILENAME))
		return false;
#endif
	if (!ReadConfigRt(M3_CONFIG_FILENAME))
		return false;
	M3_INFO("Done reading component config files...\n"); 
	//Link dependent components. Drop failures. 
	//Keep dropping until no failures
	vector<M3Component *> bad_link;
	bool failure=true;
	
	while (GetNumComponents()>0 && failure)
	{
		bad_link.clear();
		failure=false;
		for(int i=0;i<GetNumComponents();i++)
		{
			if (!GetComponent(i)->LinkDependentComponents())
			{
				M3_WARN("Failure LinkDependentComponents for %s\n",GetComponent(i)->GetName().c_str());
				failure=true;
				bad_link.push_back(GetComponent(i));
			}
		}
		if (failure)
		{
			vector<M3Component *>::iterator ci;
			vector<M3ComponentEc *>::iterator eci;
			for (int i=0;i<bad_link.size();i++)
			{
				for (eci=m3ec_list.begin();eci!=m3ec_list.end();++eci)
					if ((*eci)==bad_link[i])
					{
						//(*eci)->Shutdown();
						m3ec_list.erase(eci);
						break;
					}
				for (ci=m3rt_list.begin();ci!=m3rt_list.end();++ci)
					if ((*ci)==bad_link[i])
					{
						//(*ci)->Shutdown();
						m3rt_list.erase(ci);
						break;
					}
				factory->ReleaseComponent(bad_link[i]);
			}
		}
	}
	if (GetNumComponents()==0)
	{
		M3_WARN("No M3 Components could be loaded....\n",0);
		return false;
	}
	for(int i=0;i<GetNumComponents();i++)
	{
		GetComponent(i)->Startup();
	}
	CheckComponentStates();
	PrettyPrintComponentNames();
	//Setup Monitor
	M3MonitorStatus * s=factory->GetMonitorStatus();
	for(int i=0;i<GetNumComponents();i++)
	{
		M3MonitorComponent * c=s->add_components();
		c->set_name(GetComponent(i)->GetName());
	}
	for (int i=0; i < NUM_EC_DOMAIN; i++)
	{
	    s->add_ec_domains();
	}
	return true;
}

bool M3RtSystem::ParseCommandFromExt(M3CommandAll & msg)
{
	int idx,i;
	string s;

	for (i=0;i<msg.name_cmd_size();i++)
	{
		idx=GetComponentIdx( msg.name_cmd(i));
		if (idx>=0)
		{
			s=msg.datum_cmd(i);
			GetComponent(idx)->ParseCommand(s);
		}
		else
		{
			//M3_WARN("Invalid Command component name %s in ParseCommandFromExt\n",s.c_str());
			M3_WARN("Invalid Command component name %s in ParseCommandFromExt\n",msg.name_cmd(i).c_str());
			
		}
	}
	
	for (i=0;i<msg.name_param_size();i++)
	{
		idx=GetComponentIdx( msg.name_param(i));
		if (idx>=0)
		{
			s=msg.datum_param(i);
			GetComponent(idx)->ParseParam(s);
		}
		else
		{
			M3_WARN("Invalid Param component name %s in ParseCommandFromExt\n",s.c_str());
		}
	}
	
	return true;
}

bool M3RtSystem::SerializeStatusToExt(M3StatusAll & msg,vector<string>& names)
{
	for(int i=0;i<names.size(); i++)
	{
		M3Component * m=GetComponent(names[i]);
		if (m!=NULL)
		{
			string datum;
			if (!m->SerializeStatus(datum))
			{
				//Bug where SerializeToString fails for bad message size
				//Patched protobuf/message.cc to allow but return false
				//FixME!
				//datum.clear();
			}
			if (i>=msg.datum_size())
			{   //Grow message
				msg.add_datum(datum);
				msg.add_name(names[i]);
			}
			else
			{
				msg.set_datum(i,datum);
				msg.set_name(i,names[i]);
			}
		}
	}
	return true;
}



void M3RtSystem::CheckComponentStates()
{
	for(int i=0;i<GetNumComponents();i++)
	{
		if (GetComponent(i)->IsStateError() && !safeop_required) //All or none in OP
		{
			M3_WARN("Component error detected for %s. Forcing all in to state SAFEOP\n",GetComponent(i)->GetName().c_str());

			safeop_required=true;
			for(int j=0;j<GetNumComponents();j++)
			{
				GetComponent(j)->SetStateSafeOp();
			}
			return;
		}	
	}
}

bool M3RtSystem::SetComponentStateOp(int idx)
{
	if (safeop_required)
		return false;
	if (idx<GetNumComponents()&&idx>=0)
		if (GetComponent(idx)->IsStateSafeOp())
		{
			GetComponent(idx)->SetStateOp();
			return true;
		}
	return false;
}

bool M3RtSystem::SetComponentStateSafeOp(int idx)
{
	if (idx<GetNumComponents()&&idx>=0)
	{
		GetComponent(idx)->SetStateSafeOp();
		return true;
	}
	return false;
}
int M3RtSystem::GetComponentState(int idx)
{
	if (idx<GetNumComponents()&&idx>=0)
		return GetComponent(idx)->GetState();
}

void M3RtSystem::PrettyPrint()
{
	int nece=0, nrte=0, necs=0, nrts=0, neco=0, nrto=0;
	vector<M3ComponentEc*>::iterator i;
	for(i=m3ec_list.begin(); i!=m3ec_list.end(); ++i)
	{
		if ((*i)->IsStateError()) nece++;
		if ((*i)->IsStateSafeOp()) necs++;
		if ((*i)->IsStateOp()) neco++;
	}
	vector<M3Component*>::iterator j;
	for(j=m3rt_list.begin(); j!=m3rt_list.end(); ++j)
	{
		if ((*j)->IsStateError()) nrte++;
		if ((*j)->IsStateSafeOp()) nrts++;
		if ((*j)->IsStateOp()) nrto++;
	}
		
	BannerPrint(80,"M3 SYSTEM");
	M3_PRINTF("Operational: %s\n",IsOperational()?"yes":"no");
	M3_PRINTF("Ec components: %d\n",(int)m3ec_list.size());
	M3_PRINTF("Rt components: %d\n",(int)m3rt_list.size());
	M3_PRINTF("Ec components in error: %d\n",nece);
	M3_PRINTF("Rt components in error: %d\n",nrte);
	M3_PRINTF("Ec components in safeop: %d\n",necs);
	M3_PRINTF("Rt components in safeop: %d\n",nrts);
	M3_PRINTF("Ec components in op: %d\n",neco);
	M3_PRINTF("Rt components in op: %d\n",nrto);
}

	

void M3RtSystem::PrettyPrintComponentNames()
{
	BannerPrint(60,"M3 SYSTEM COMPONENTS");
	for(int i=0;i<GetNumComponents();i++)
		M3_PRINTF("%s\n",GetComponentName(i).c_str());
	BannerPrint(60,"");
}

void M3RtSystem::PrettyPrintComponents()
{
	PrettyPrint();
	for(int i=0;i<GetNumComponents();i++)
		GetComponent(i)->PrettyPrint();
	M3_PRINTF("\n\n\n",0);
}

void M3RtSystem::PrettyPrintComponent(int idx)
{
	if (idx<GetNumComponents()&&idx>=0)
		GetComponent(idx)->PrettyPrint();
}
	
bool M3RtSystem::Step(bool safeop_only)
{
	int i;
#ifdef __RTAI__
	RTIME start, end, dt,start_c,end_c, start_p,end_p;
#else
	long long start, end, dt,start_c,end_c, start_p,end_p;
#endif
	vector<M3ComponentEc*>::iterator j;
	step_cnt++;

	/*
		1: Block External Data Service from chaging state
		2: Wait until EtherCAT mod signals finished a cycle (synchronize)
		3: Acquire lock on EtherCAT shared mem
		4: Get data from EtherCAT shared mem
		5: Step all components
		6: Transmit newly computed commands to EtherCAT shared mem
		7: Upload status to logger
		8: Release locks, allow External Data Service to update components if new data is available.
	*/
	
	/*
		The order of components in m3ec_list and m3rt_list is important as they can overwrite eachother.
		Given components A, B, where A computes and sets value B.x . 
		If we place A before B in the component list of m3_config.yml, then A.Step() will run before B.Step() within once cycle.
		Otherwise, B.Step() uses the A.x value from the previous cycle.
	
		If we have an External Data Service that sets B.x=e periodically, then A.Step() will overwrite value e.
		Therefore if we want to directly communicate with B.x from the outside world, we must not publish to component A.	
	*/
#ifdef __RTAI__
	rt_sem_wait(ext_sem);
	rt_sem_wait(sync_sem);
	rt_sem_wait(shm_sem);
	start = rt_get_cpu_time_ns();
#else
	
	sem_wait(ext_sem);
	start = getNanoSec();
#endif	
	if (safeop_only) // in case we are too slow
	{
	  for(int i=0;i<GetNumComponents();i++)
	    GetComponent(i)->SetStateSafeOp();
	  
	}
	
	//Do some bookkeeping
	M3MonitorStatus * s=factory->GetMonitorStatus();
	int nop=0,nsop=0,nerr=0;
	for (int i=0;i<GetNumComponents();i++)
	{
		if (GetComponent(i)->IsStateError()) nerr++;
		if (GetComponent(i)->IsStateSafeOp()) nsop++;
		if (GetComponent(i)->IsStateOp()) nop++;
		M3MonitorComponent* c=s->mutable_components(i);
		c->set_state((M3COMP_STATE)GetComponent(i)->GetState());
	}
	if (m3ec_list.size() != 0)
	{
	  for (int i=0;i<NUM_EC_DOMAIN;i++)
	  {
	      s->mutable_ec_domains(i)->set_t_ecat_wait_rx(shm_ec->monitor[i].t_ecat_wait_rx);
	      s->mutable_ec_domains(i)->set_t_ecat_rx(shm_ec->monitor[i].t_ecat_rx);
	      s->mutable_ec_domains(i)->set_t_ecat_wait_shm(shm_ec->monitor[i].t_ecat_wait_shm);
	      s->mutable_ec_domains(i)->set_t_ecat_shm(shm_ec->monitor[i].t_ecat_shm);
	      s->mutable_ec_domains(i)->set_t_ecat_wait_tx(shm_ec->monitor[i].t_ecat_wait_tx);
	      s->mutable_ec_domains(i)->set_t_ecat_tx(shm_ec->monitor[i].t_ecat_tx);	
	  }
	}
	s->set_num_components_safeop(nsop);
	s->set_num_components_op(nop);
	s->set_num_components_err(nerr);
	s->set_num_components(GetNumComponents());
	s->set_num_components_ec(m3ec_list.size());
	s->set_num_components_rt(m3rt_list.size());
	s->set_operational(IsOperational());
	s->set_num_ethercat_cycles(GetEcCounter());
#ifdef __RTAI__
	//Set timestamp for all
	int64_t ts=shm_ec->timestamp_ns/1000;
#else
	int64_t ts=getNanoSec()/1000;
#endif
	for(i=0;i<GetNumComponents();i++) 
		GetComponent(i)->SetTimestamp(ts);
#ifdef __RTAI__
	start_p = rt_get_cpu_time_ns();
#else	
	start_p = getNanoSec();
#endif

#ifdef __RTAI__
	//Get Status from EtherCAT
	for (int j=0; j<=MAX_PRIORITY; j++)
	{
		for(int i=0;i<m3ec_list.size();i++) //=m3ec_list.begin(); j!=m3ec_list.end(); ++j)
		{
			if (m3ec_list[i]->GetPriority() == j)
			{
				start_c = rt_get_cpu_time_ns();
				m3ec_list[i]->StepStatus();
				end_c = rt_get_cpu_time_ns();
				M3MonitorComponent* c=s->mutable_components(idx_map_ec[i]);
				c->set_cycle_time_status_us((mReal)(end_c-start_c)/1000);
			}
		}
	}
#endif
	//Set Status on non-EC components
	for (int j=0; j<=MAX_PRIORITY; j++)
	{
		for(i=0; i<m3rt_list.size(); i++)
		{
			if (m3rt_list[i]->GetPriority() == j)
			{
#ifdef __RTAI__
				start_c = rt_get_cpu_time_ns();
#else
				start_c = getNanoSec();
#endif
				m3rt_list[i]->StepStatus();
#ifdef __RTAI__
				end_c = rt_get_cpu_time_ns();
#else
				end_c = getNanoSec();
#endif
				M3MonitorComponent* c=s->mutable_components(idx_map_rt[i]);
				c->set_cycle_time_status_us((mReal)(end_c-start_c)/1000);
			}	
		}
	}
#ifdef __RTAI__
	end_p = rt_get_cpu_time_ns();
#else
	end_p = getNanoSec();
#endif
	s->set_cycle_time_status_us((mReal)(end_p-start_p)/1000);
	//Set Command on non-EC components
	//Step components in reverse order	
#ifdef __RTAI__
	start_p = rt_get_cpu_time_ns();
#else
	start_p = getNanoSec();
#endif
	for (int j=MAX_PRIORITY; j>=0; j--)
	{
		for(i=0; i<m3rt_list.size(); i++)
		{
				if (m3rt_list[i]->GetPriority() == j)
				{
#ifdef __RTAI__
					start_c = rt_get_cpu_time_ns();
#else
					start_c = getNanoSec();
#endif
					m3rt_list[i]->StepCommand();
#ifdef __RTAI__
					end_c = rt_get_cpu_time_ns();
#else
					end_c = getNanoSec();
#endif
					M3MonitorComponent* c=s->mutable_components(idx_map_rt[i]);
					c->set_cycle_time_command_us((mReal)(end_c-start_c)/1000);
				}
		}
	}
#ifdef __RTAI__
	//Send Command to EtherCAT
	for (int j=MAX_PRIORITY; j>=0; j--)
	{
		for(int i=0;i<m3ec_list.size();i++)
		{
			if (m3ec_list[i]->GetPriority() == j)
			{
				start_c = rt_get_cpu_time_ns();
				m3ec_list[i]->StepCommand();
				end_c = rt_get_cpu_time_ns();
				M3MonitorComponent* c=s->mutable_components(idx_map_ec[i]);
				c->set_cycle_time_command_us((mReal)(end_c-start_c)/1000);
			}
			
		}
	}
	end_p = rt_get_cpu_time_ns();
#else
	end_p = getNanoSec();
#endif
	s->set_cycle_time_command_us((mReal)(end_p-start_p)/1000);
	//Now see if any errors raised
	CheckComponentStates();
	
	if (log_service)
	{
	  logging = true;
	  if (!log_service->Step())
	      M3_DEBUG("Step() of log service failed.\n");
	   logging = false;
	}
#ifdef __RTAI__
	end = rt_get_cpu_time_ns();
#else
	end = getNanoSec();
#endif
	mReal elapsed=(mReal)(end-start)/1000;
	if (elapsed>s->cycle_time_max_us()&&step_cnt>10)
		s->set_cycle_time_max_us(elapsed);
	s->set_cycle_time_us(elapsed);
	int64_t period = end-last_cycle_time;
	mReal rate=1/(mReal)period;
	s->set_cycle_frequency_hz((mReal)(rate*1000000000.0));
	last_cycle_time=end;
#ifdef __RTAI__
	rt_sem_signal(shm_sem);
	rt_sem_signal(ext_sem);
#else
	sem_post(ext_sem);
#endif
	return true;
}

bool M3RtSystem::ReadConfigEc(const char * filename)
{
	YAML::Node doc;
	GetYamlDoc(filename, doc);

	//Make sure an ec_component section
	if(!doc.FindValue("ec_components") ) 
	{
		M3_INFO("No ec_components key in m3_config.yml. Proceeding without it...\n");
		return true;
	}
	
	for(YAML::Iterator it=doc["ec_components"].begin();it!=doc["ec_components"].end();++it) 
	{
   		string dir;
		
    		it.first() >> dir;
		
		for(YAML::Iterator it_dir=doc["ec_components"][dir.c_str()].begin();
			it_dir!=doc["ec_components"][dir.c_str()].end();++it_dir) 
		{
			string  name, type;
			it_dir.first() >> name;
			it_dir.second() >> type;
			
			M3ComponentEc * m=NULL;
			m =(M3ComponentEc*) factory->CreateComponent(type);
			if (m!=NULL)
			{			
				m->SetFactory(factory);
				string f=dir+"/"+name+".yml";
				try 
				{
					if (m->ReadConfig(f.c_str()))
					{
						if (!m->SetSlaveEcShm(shm_ec->slave,shm_ec->slaves_responding)) 
							factory->ReleaseComponent(m);
						else
						{
							m3ec_list.push_back(m);
							idx_map_ec.push_back(GetNumComponents()-1);
						}
					} 
					else factory->ReleaseComponent(m);
				} catch(YAML::TypedKeyNotFound<string> e) {
					M3_WARN("Missing key: %s in config file for EC component %s \n", e.key.c_str(), name.c_str());
					factory->ReleaseComponent(m);
				} catch(YAML::RepresentationException e) {
					M3_WARN("%s while parsing config files for EC component %s \n",e.what(), name.c_str());
					factory->ReleaseComponent(m);
				} catch(...)
				{
					M3_WARN("Error while parsing config files for EC component %s \n",name.c_str());
					factory->ReleaseComponent(m);
				}
			}
		}
	}

	return true;
}

bool M3RtSystem::ReadConfigRt(const char * filename)
{	
	YAML::Node doc;
	GetYamlDoc(filename, doc);
	
	if(!doc.FindValue("rt_components")) 
	{
		M3_INFO("No rt_components key in m3_config.yml. Proceeding without it...\n");
		return true;
	}

	for(YAML::Iterator it=doc["rt_components"].begin();it!=doc["rt_components"].end();++it) 
	{
   		string dir;

    		it.first() >> dir;
		
		for(YAML::Iterator it_dir=doc["rt_components"][dir.c_str()].begin();
			it_dir!=doc["rt_components"][dir.c_str()].end();++it_dir) 
		{
			string  name, type;
			it_dir.first() >> name;
			it_dir.second() >> type;
			M3Component * m=factory->CreateComponent(type);
			if (m!=NULL)
			{
				m->SetFactory(factory);
				string f=dir+"/"+name+".yml";
				try 
				{
					if (m->ReadConfig(f.c_str()))
					{
						m3rt_list.push_back(m);
						idx_map_rt.push_back(GetNumComponents()-1);
					}
					else 
					{
						factory->ReleaseComponent(m);
						M3_ERR("Error reading config for %s\n",name.c_str());
					}
				} catch(YAML::TypedKeyNotFound<string> e) {
					M3_WARN("Missing key: %s in config file for RT component %s \n", e.key.c_str(), name.c_str());
					factory->ReleaseComponent(m);
				} catch(YAML::RepresentationException e) {
					M3_WARN("%s while parsing config files for RT component %s \n",e.what(), name.c_str());
					factory->ReleaseComponent(m);
				} catch(...)
				{
					M3_WARN("Error while parsing config files for RT component %s \n",name.c_str());
					factory->ReleaseComponent(m);
				}
				
			}	
		}
	}
	return true;
}

}










