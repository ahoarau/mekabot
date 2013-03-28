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
#include <rtai_sched.h>
#include <stdio.h>
#include <signal.h>
#include <rtai_shm.h>
#include <rtai.h>
#include <rtai_sem.h>
#include <m3rt/base/m3ec_def.h>
#include <m3rt/base/m3rt_def.h>
#include <rtai_nam2num.h>
#include <rtai_registry.h>
#include "m3/shared_mem/torque_shm_sds.h"
#include "m3/robots/chain_name.h"

#define RT_TASK_FREQUENCY_TORQUE_SHM 400
#define RT_TIMER_TICKS_NS_TORQUE_SHM (1000000000 / RT_TASK_FREQUENCY_TORQUE_SHM)		//Period of rt-timer 
#define TORQUE_SHM "TSHMM"
#define TORQUE_CMD_SEM "TSHMC"
#define TORQUE_STATUS_SEM "TSHMS"

////////////////////////////////////////////////////////////////////////////////////
static int sys_thread_active = 0;
static int sys_thread_end=0;
static int end=0;
static int hst;
static M3TorqueShmSdsCommand cmd;
static M3TorqueShmSdsStatus status;
static int sds_status_size;
static int sds_cmd_size;
static void endme(int dummy) { end=1; }
////////////////////////////////////////////////////////////////////////////////////
////// TorqueShm API:

mReal GetThetaDeg(enum M3Chain chain,unsigned int  idx);
mReal GetThetaDotDeg(enum M3Chain chain,unsigned int  idx);
mReal GetTorque_mNm(enum M3Chain chain,unsigned int  idx);
JOINT_ARRAY_MODE GetMode(enum M3Chain chain,unsigned int  idx);
void SetTorque_mNm(enum M3Chain chain,unsigned int  idx, mReal tq_desired);
void SetTimestamp(int64_t  timestamp); //ToShm
int64_t GetTimestamp(); //FromShm

///////  Periodic Control Loop:
void StepTorqueShm();

/////////////////////////////////////
mReal GetThetaDeg(enum M3Chain chain,unsigned int  idx)
{  
  if (idx > MAX_NDOF)
    return 0.;
  
 switch(chain)
    {
      case RIGHT_ARM:
	  return status.right_arm.theta[idx];
      case LEFT_ARM:
	  return status.left_arm.theta[idx];
      case TORSO:	
	  return status.torso.theta[idx];	
      case HEAD:	
	  return status.head.theta[idx];
    }
    return 0.; 
}
mReal GetWrench(enum M3Chain chain,unsigned int  idx)
{  
  if (idx >= 6)
    return 0.;
  
 switch(chain)
    {
      case RIGHT_ARM:
	  return status.right_arm.wrench[idx];
      case LEFT_ARM:
	  return status.left_arm.wrench[idx];
      case TORSO:	
	  return status.torso.wrench[idx];	
      case HEAD:	
	  return status.head.wrench[idx];
    }
    return 0.; 
}

mReal GetThetaDotDeg(enum M3Chain chain,unsigned int  idx)
{  
  if (idx > MAX_NDOF)
    return 0.;
  
 switch(chain)
    {
      case RIGHT_ARM:
	  return status.right_arm.thetadot[idx];
      case LEFT_ARM:
	  return status.left_arm.thetadot[idx];
      case TORSO:	
	  return status.torso.thetadot[idx];	
      case HEAD:	
	  return status.head.thetadot[idx];
    }
    return 0.; 
}

mReal GetTorque_mNm(enum M3Chain chain,unsigned int  idx)
{  
  if (idx > MAX_NDOF)
    return 0.;
  
 switch(chain)
    {
      case RIGHT_ARM:
	  return status.right_arm.torque[idx];
      case LEFT_ARM:
	  return status.left_arm.torque[idx];
      case TORSO:	
	  return status.torso.torque[idx];	
      case HEAD:	
	  return status.head.torque[idx];
    }
    return 0.; 
}

JOINT_ARRAY_MODE GetMode(enum M3Chain chain,unsigned int  idx)
{  
  if (idx > MAX_NDOF)
    return JOINT_ARRAY_MODE_OFF;
  
 switch(chain)
    {
      case RIGHT_ARM:
	  return status.right_arm.ctrl_mode[idx];
      case LEFT_ARM:
	  return status.left_arm.ctrl_mode[idx];
      case TORSO:	
	  return status.torso.ctrl_mode[idx];	
      case HEAD:	
	  return status.head.ctrl_mode[idx];
    }
    return JOINT_ARRAY_MODE_OFF; 
}

void SetTorque_mNm(enum M3Chain chain,unsigned int  idx, mReal tq_desired)
{  
  if (idx > MAX_NDOF)
    return;
  
 switch(chain)
    {
      case RIGHT_ARM:
	  cmd.right_arm.tq_desired[idx] = tq_desired;
	  return;
      case LEFT_ARM:
	  cmd.left_arm.tq_desired[idx] = tq_desired;
	  return;
      case TORSO:	
	  cmd.torso.tq_desired[idx] = tq_desired;
	  return;
      case HEAD:	
	  cmd.head.tq_desired[idx] = tq_desired;
	  return;
    }
    return; 
}

void SetTimestamp(int64_t  timestamp)
{  
  cmd.timestamp = timestamp;
    return; 
}

int64_t GetTimestamp()
{  
    return status.timestamp; 
}

////////////////////////// MAIN COMPUTATION METHOD /////////////////////////////

/**************************/
//  Read Status and update Tq Cmd here. 
/**************************/
static mReal torque_start;
static mReal torque_last;
static mReal torque_new;
static bool torque_shm_start = false;
static bool going_up=true;
    
//This demo ramps the torque of J0...
void StepTorqueShm()
{   
    
    mReal torque_amp = 5000.0; //mNm
    mReal time_period = 5.0;
    
    
    mReal torque_step = torque_amp / (RT_TASK_FREQUENCY_TORQUE_SHM * time_period); //mNm
    //printf("ts:%f\n",torque_step);
    SetTimestamp(GetTimestamp()); //Pass back timestamp as a heartbeat
    //printf("ts: %lld\n",GetTimestamp());
    /*
    printf("th[0]: %f\n",GetThetaDeg(RIGHT_ARM,0));
    printf("thdot[0]: %f\n",GetThetaDotDeg(RIGHT_ARM,0));
    printf("tq[0]: %f\n",GetTorque_mNm(RIGHT_ARM,0));*/
    printf("Fx: %f\n",GetWrench(RIGHT_ARM,0));
    printf("Fy: %f\n",GetWrench(RIGHT_ARM,1));
    printf("Fz: %f\n",GetWrench(RIGHT_ARM,2));
    printf("Tx: %f\n",GetWrench(RIGHT_ARM,3));
    printf("Ty: %f\n",GetWrench(RIGHT_ARM,4));
    printf("Tz: %f\n",GetWrench(RIGHT_ARM,5));
    if ((GetMode(RIGHT_ARM,0) == JOINT_ARRAY_MODE_TORQUE_SHM) && !torque_shm_start) //Start Condition, switched into TORQUE_SHM mode
    {
      torque_start = GetTorque_mNm(RIGHT_ARM,0); //Initialize at current torque
      torque_last = torque_start;
      printf("st:%f\n",torque_start);
      torque_shm_start = true;
      going_up = true;
    }

    if (GetMode(RIGHT_ARM,0) != JOINT_ARRAY_MODE_TORQUE_SHM) //Switched out of TORQUE_SHM mode
    {      
      torque_shm_start = false;
      going_up = true;
    }

    if (torque_shm_start)
    {
      if (going_up) {
	torque_new = torque_last + torque_step;
	SetTorque_mNm(RIGHT_ARM,0,torque_new);
	//SetTorque_mNm(RIGHT_ARM,0,1234.0);
	torque_last = torque_new;
	if ((torque_last - torque_start) > torque_amp)
	  going_up = false;
      } else {
	torque_new = torque_last - torque_step;
	SetTorque_mNm(RIGHT_ARM,0,torque_new);
	//SetTorque_mNm(RIGHT_ARM,0,345.0);
	torque_last = torque_new;
	if ((torque_last - torque_start) < -torque_amp)
	  going_up = true;	
      }
      //printf("t:%f\n",torque_last);
      //if (going_up)
	//printf("g:%d\n",1);
      //else
	//printf("g:%d\n",0);
    }
    //SetTorque_mNm(RIGHT_ARM,0,0.123);
}

////////////////////////// RTAI PROCESS BOILERPLATE /////////////////////////////

static void* rt_system_thread(void * arg)
{	
	SEM * status_sem;
	SEM * command_sem;
	RT_TASK *task;
	
	M3Sds * sds = (M3Sds *)arg;
	printf("Starting real-time thread\n",0);
		
	
	sds_status_size = sizeof(M3TorqueShmSdsStatus);
	sds_cmd_size = sizeof(M3TorqueShmSdsCommand);
	
	task = rt_task_init_schmod(nam2num("TSHMP"), 0, 0, 0, SCHED_FIFO, 0xF);
	rt_allow_nonroot_hrt();
	if (task==NULL)
	{
		printf("Failed to create RT-TASK TSHMP\n",0);
		return 0;
	}
	status_sem=(SEM*)rt_get_adr(nam2num(TORQUE_STATUS_SEM));
	command_sem=(SEM*)rt_get_adr(nam2num(TORQUE_CMD_SEM));
	if (!status_sem)
	{
		printf("Unable to find the %s semaphore.\n",TORQUE_STATUS_SEM);
		rt_task_delete(task);
		return 0;
	}
	if (!command_sem)
	{
		printf("Unable to find the %s semaphore.\n",TORQUE_CMD_SEM);
		rt_task_delete(task);
		return 0;
	}
	
	
	RTIME tick_period = nano2count(RT_TIMER_TICKS_NS_TORQUE_SHM); 
	RTIME now = rt_get_time();
	rt_task_make_periodic(task, now + tick_period, tick_period); 
	mlockall(MCL_CURRENT | MCL_FUTURE);
	rt_make_hard_real_time();
	long long start_time, end_time, dt;
	long long step_cnt = 0;
	sys_thread_active=1;
	
	while(!sys_thread_end)
	{
		start_time = nano2count(rt_get_cpu_time_ns());
		rt_sem_wait(status_sem);
		memcpy(&status, sds->status, sds_status_size);		
		rt_sem_signal(status_sem);
		
		StepTorqueShm();		
		
		rt_sem_wait(command_sem);
		memcpy(sds->cmd, &cmd, sds_cmd_size);		
		rt_sem_signal(command_sem);
				
		end_time = nano2count(rt_get_cpu_time_ns());
		dt=end_time-start_time;
		/*
		Check the time it takes to run components, and if it takes longer
		than our period, make us run slower. Otherwise this task locks
		up the CPU.*/
		if (dt > tick_period && step_cnt>10) 
		{
			printf("Step %lld: Computation time of components is too long. Forcing all components to state SafeOp.\n",step_cnt);
			printf("Previous period: %f. New period: %f\n", (double)count2nano(tick_period),(double)count2nano(dt));
 			tick_period=dt;
			rt_task_make_periodic(task, end + tick_period,tick_period);			
		}
		step_cnt++;
		rt_task_wait_period();
	}	
	printf("Exiting RealTime Thread...\n",0);
	rt_make_soft_real_time();
	rt_task_delete(task);
	sys_thread_active=0;
	return 0;
}

////////////////////////////////////////////////////////////////////////////////////
int main (void)
{	
	RT_TASK *task;
	M3Sds * sys;
	int cntr=0;
	
	signal(SIGINT, endme);

	if (sys = (M3Sds*)rt_shm_alloc(nam2num(TORQUE_SHM),sizeof(M3Sds),USE_VMALLOC))
		printf("Found shared memory starting torque_shm.");
	else
	{
		printf("Rtai_malloc failure for %s\n",TORQUE_SHM);
		return 0;
	}

	rt_allow_nonroot_hrt();
	/*if (!(task = rt_task_init_schmod(nam2num("TSHM"), RT_TASK_PRIORITY, 0, 0, SCHED_FIFO, 0xF)))
	{
		rt_shm_free(nam2num(TORQUE_SHM));
		printf("Cannot init the RTAI task %s\n","TSHM");
		return 0;
	}*/
	hst=rt_thread_create((void*)rt_system_thread, sys, 10000);
	usleep(100000); //Let start up
	if (!sys_thread_active)
	{
		rt_task_delete(task);
		rt_shm_free(nam2num(TORQUE_SHM));
		printf("Startup of thread failed.\n",0);
		return 0;
	}
	while(!end)
	{		
		usleep(250000);
		
	}
	printf("Removing RT thread...\n",0);
	sys_thread_end=1;
	rt_thread_join(hst);
	if (sys_thread_active)printf("Real-time thread did not shutdown correctly\n");
	rt_task_delete(task);
	rt_shm_free(nam2num(TORQUE_SHM));
	return 0;
}

