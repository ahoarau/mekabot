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
#include "m3/robots/humanoid_shm_sds.h"


#define RT_TASK_FREQUENCY_BOT_SHM 400
#define RT_TIMER_TICKS_NS_BOT_SHM (1000000000 / RT_TASK_FREQUENCY_BOT_SHM)		//Period of rt-timer 
#define BOT_SHM "TSHMM"
#define BOT_CMD_SEM "TSHMC"
#define BOT_STATUS_SEM "TSHMS"

////////////////////////////////////////////////////////////////////////////////////
static int sys_thread_active = 0;
static int sys_thread_end=0;
static int end=0;
static int hst;
static M3HumanoidShmSdsCommand cmd;
static M3HumanoidShmSdsStatus status;
static int sds_status_size;
static int sds_cmd_size;
static long step_cnt = 0;
static void endme(int dummy) { end=1; }
////////////////////////////////////////////////////////////////////////////////////
////// TorqueShm API:

///////  Periodic Control Loop:
void StepHumanoidShm();

///////////////////////////////

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

    

void StepHumanoidShm(int cntr)
{   
    SetTimestamp(GetTimestamp()); //Pass back timestamp as a heartbeat
  
    
      
      
      for (int i = 0; i < 7; i++)
      {
	//cmd.right_arm.ctrl_mode[i] = JOINT_ARRAY_MODE_TORQUE;
	cmd.right_arm.ctrl_mode[i] = JOINT_ARRAY_MODE_THETA_GC;
	cmd.right_arm.q_desired[i] = 30.0;
	cmd.right_arm.tq_desired[i] = 40.0;
	cmd.right_arm.slew_rate_q_desired[i] = 20.0;
	cmd.right_arm.q_stiffness[i] = 1.0;	
      }
      

      if (cntr % RT_TASK_FREQUENCY_BOT_SHM == 0)
      {
	printf("------------------------------\n");
	printf("timestamp: %ld\n",GetTimestamp() );
	 
	/*printf("Fx: %f\n", status.right_loadx6.wrench[0]);
	printf("Fy: %f\n", status.right_loadx6.wrench[1]);
	printf("Fz: %f\n", status.right_loadx6.wrench[2]);
	printf("Tx: %f\n", status.right_loadx6.wrench[3]);
	printf("Ty: %f\n", status.right_loadx6.wrench[4]);
	printf("Tz: %f\n", status.right_loadx6.wrench[5]);*/
	  
	for (int i = 0; i < 7; i++)
	{
	    //printf("jnt_tq: %i, %f\n", i, status.right_arm.torque[i]);
	    printf("joint_angle_read: %i, %f\n", i, status.right_arm.theta[i]);
	    printf("joint_angle_command: %i, %f\n", i, cmd.right_arm.q_desired[i]);	    
	    //printf("jnt_vel: %i, %f\n", i, status.right_arm.thetadot[i]);	    
	}
	
	
	
      }
}

////////////////////////// RTAI PROCESS BOILERPLATE /////////////////////////////

static void* rt_system_thread(void * arg)
{	
	SEM * status_sem;
	SEM * command_sem;
	RT_TASK *task;
	int cntr=0;
	M3Sds * sds = (M3Sds *)arg;
	printf("Starting real-time thread\n",0);
		
	
	sds_status_size = sizeof(M3HumanoidShmSdsStatus);
	sds_cmd_size = sizeof(M3HumanoidShmSdsCommand);
	
	memset(&cmd, 0, sds_cmd_size);
	
	task = rt_task_init_schmod(nam2num("TSHMP"), 0, 0, 0, SCHED_FIFO, 0xF);
	rt_allow_nonroot_hrt();
	if (task==NULL)
	{
		printf("Failed to create RT-TASK TSHMP\n",0);
		return 0;
	}
	status_sem=(SEM*)rt_get_adr(nam2num(BOT_STATUS_SEM));
	command_sem=(SEM*)rt_get_adr(nam2num(BOT_CMD_SEM));
	if (!status_sem)
	{
		printf("Unable to find the %s semaphore.\n",BOT_STATUS_SEM);
		rt_task_delete(task);
		return 0;
	}
	if (!command_sem)
	{
		printf("Unable to find the %s semaphore.\n",BOT_CMD_SEM);
		rt_task_delete(task);
		return 0;
	}
	
	
	RTIME tick_period = nano2count(RT_TIMER_TICKS_NS_BOT_SHM); 
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
		
		StepHumanoidShm(cntr);		
		
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
		cntr++;
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

	if (sys = (M3Sds*)rt_shm_alloc(nam2num(BOT_SHM),sizeof(M3Sds),USE_VMALLOC))
		printf("Found shared memory starting m3humandoid_shm_test.\n");
	else
	{
		printf("Rtai_malloc failure for %s\n",BOT_SHM);
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
		rt_shm_free(nam2num(BOT_SHM));
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
	rt_shm_free(nam2num(BOT_SHM));
	return 0;
}

