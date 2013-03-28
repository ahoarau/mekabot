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
#include <unistd.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <signal.h>
#include <rtai_shm.h>
#include <rtai.h>
#include <rtai_sem.h>

#include <rtai_nam2num.h>
#include <rtai_registry.h>

#include "ecrt.h"
	
#include "pthread.h"

#include "m3rt/base/m3ec_def.h"
#include "m3rt/base/m3rt_def.h"
#include "m3/hardware/m3meka_ec_def.h"
#include "stdio.h"

#define TIMESTAT_INIT 0xABCDEF
#define NS_TO_MS(a) (((double)a)/1000000.0)
#define HICKUP_THRESHOLD 3000000   // 3 ms
////////////////////////////////////////////////////////////////////////////////////
void InitStats(void);
void PrintStats(M3EcSystemShm * sys);
void CalcStats(M3EcSystemShm * sys);

////////////////////////////////////////////////////////////////////////////////////
static int sys_thread_active = 0;
static int sys_thread_end=0;
static int end=0;
static int hst;
static int64_t hickups[MAX_NUM_SLAVE];
static int64_t time_last[MAX_NUM_SLAVE];
static int64_t time_new[MAX_NUM_SLAVE];
static int64_t time_to_step_diff[MAX_NUM_SLAVE];
static int64_t time_to_step_sum[MAX_NUM_SLAVE];
static int64_t time_to_step_best[MAX_NUM_SLAVE];
static int64_t time_to_step_worst[MAX_NUM_SLAVE];
static int64_t time_to_mstr_diff[MAX_NUM_SLAVE];
static int64_t time_to_mstr_sum[MAX_NUM_SLAVE];
static int64_t time_to_mstr_best[MAX_NUM_SLAVE];
static int64_t time_to_mstr_worst[MAX_NUM_SLAVE];
static int64_t cntr_sum[MAX_NUM_SLAVE];
static int64_t time_slaves_diff[MAX_NUM_SLAVE][MAX_NUM_SLAVE];
static int64_t time_slaves_sum[MAX_NUM_SLAVE][MAX_NUM_SLAVE];
static int64_t time_slaves_best[MAX_NUM_SLAVE][MAX_NUM_SLAVE];
static int64_t time_slaves_worst[MAX_NUM_SLAVE][MAX_NUM_SLAVE];
static int64_t cntr_slaves_sum[MAX_NUM_SLAVE][MAX_NUM_SLAVE];

static void endme(int dummy) { end=1; }
////////////////////////////////////////////////////////////////////////////////////


	
static inline void count2timeval(RTIME rt, struct timeval *t)
{
	t->tv_sec = rtai_ulldiv(count2nano(rt), 1000000000, (unsigned long *)&t->tv_usec);
	t->tv_usec /= 1000;
}

static void* rt_system_thread(void * arg)
{
	unsigned char slaves_OP=0;
	struct timeval tv;	
	int64_t ts1, ts2;
	SEM * shm_sem;
	SEM * sync_sem;
	RT_TASK *task;
	int i, j;
	M3EcSystemShm * sys = (M3EcSystemShm *)arg;
	printf("Starting real-time thread\n",0);
	RTIME t_last;
	int64_t cntr=0;	
	task = rt_task_init_schmod(nam2num("M3SYSP"), 0, 0, 0, SCHED_FIFO, 0xF);
	rt_allow_nonroot_hrt();
	if (task==NULL)
	{
		printf("Failed to create RT-TASK M3SYSP\n",0);
		return 0;
	}
	shm_sem=(SEM*)rt_get_adr(nam2num(SEMNAM_M3LSHM));
	if (!shm_sem)
	{
		printf("Unable to find the SEMNAM_M3LSHM semaphore.\n",0);
		rt_task_delete(task);
		return 0;
	}
	else
		printf("Allocated shm_sem semaphore  %08x \n",shm_sem);
	
	sync_sem=(SEM*)rt_get_adr(nam2num(SEMNAM_M3SYNC));
	if (!sync_sem)
	{
		printf("Unable to find the SEMNAM_M3SYNC semaphore.\n",0);
		rt_task_delete(task);
		rt_sem_delete(shm_sem);
		return 0;
	}
	else
		printf("Allocated sync_sem semaphore  %08x \n",sync_sem);
	
	RTIME tick_period = nano2count(RT_TIMER_TICKS_NS); 
	RTIME now = rt_get_time();
	rt_task_make_periodic(task, now + tick_period, tick_period); 
	mlockall(MCL_CURRENT | MCL_FUTURE);
	rt_make_hard_real_time();
	t_last=now;
	sys_thread_active=1;
	printf("Waiting for Slaves to be in OP state\n");
	
	while((!slaves_OP)&&(!sys_thread_end))
	{
		rt_sem_wait(sync_sem);
		rt_sem_wait(shm_sem);
		slaves_OP=1;
		for(i=0;i<sys->slaves_responding;i++)
		{				
			if((sys->slave[i].al_state!=8)&&(sys->slave[i].active))
			{
				//printf("al_state %d : %d\n",i,sys->slave[i].al_state);
				slaves_OP=0;
			}
		}
		rt_sem_signal(shm_sem);
		rt_task_wait_period();
	}
	
	
	
	uint64_t tl;
	while(!sys_thread_end)
	{
		
		rt_sem_wait(sync_sem);
		rt_sem_wait(shm_sem);	
							
		if(cntr%200==0)		
			PrintStats(sys);
		
		CalcStats(sys);
	
		cntr++;
		rt_sem_signal(shm_sem);
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
	int i,j;
	M3EcSystemShm * sys;
	RT_TASK *task;
	pthread_t ptsys;
	int cntr=0;
	signal(SIGINT, endme);
	sys = rtai_malloc (nam2num(SHMNAM_M3MKMD),1);
	int ns=sys->slaves_active;
	InitStats();
	printf("Found %d active M3 EtherCAT slaves\n",ns);
	if (ns==0)
	{
		printf("No slaves available. Exiting...\n");
		return 0;
	}
	rt_allow_nonroot_hrt();
	if (!(task = rt_task_init_schmod(nam2num("M3MAIN"), RT_TASK_PRIORITY, 0, 0, SCHED_FIFO, 0xF)))
	{
		rt_shm_free(nam2num(SHMNAM_M3MKMD));
		printf("Cannot init the RTAI task %s\n","M3MAIN");
		return 0;
	}
	hst=rt_thread_create((void*)rt_system_thread, sys, 10000);
	usleep(100000); //Let start up
	if (!sys_thread_active)
	{
		rt_task_delete(task);
		rt_shm_free(nam2num(SHMNAM_M3MKMD));
		printf("Startup of thread failed.\n",0);
		return 0;
	}
	while(!end)
	{
		//SysEcShmPrettyPrint(sys);
		usleep(250000);
		
	}
	printf("Removing RT thread...\n",0);
	sys_thread_end=1;
	rt_thread_join(hst);
	if (sys_thread_active)printf("Real-time thread did not shutdown correctly\n");
	rt_task_delete(task);
	rt_shm_free(nam2num(SHMNAM_M3MKMD));
	return 0;
}



void CalcStats(M3EcSystemShm * sys)
{
	int i,j;
	for (i=0;i<sys->slaves_responding;i++)
	{
		if (sys->slave[i].active)
		{	
			if (sys->slave[i].product_code==M3ACTX1_PRODUCT_CODE)								
				time_new[i] = (int64_t)((M3ActX1PdoStatus *) sys->slave[i].status)->timestamp;
			else if (sys->slave[i].product_code==M3LOADX6_PRODUCT_CODE)
				time_new[i] = (int64_t)((M3LoadX6PdoStatus *) sys->slave[i].status)->timestamp;
			else if (sys->slave[i].product_code==M3PWR_PRODUCT_CODE)
				time_new[i] = (int64_t)((M3PwrPdoStatus *) sys->slave[i].status)->timestamp;			
			else
				time_new[i] = 0; // TODO Handle unknown product codes?
				
			if(time_last[i]!=TIMESTAT_INIT)
			{
					// Calc step to step jitter for slave
				time_to_step_diff[i] = time_new[i]-time_last[i]-RT_TIMER_TICKS_NS;
				if(ABS(time_to_step_diff[i])>ABS(time_to_step_worst[i]))
					time_to_step_worst[i]=time_to_step_diff[i];
				if(ABS(time_to_step_diff[i])<ABS(time_to_step_best[i]))
					time_to_step_best[i]=time_to_step_diff[i];
				time_to_step_sum[i]+=ABS(time_to_step_diff[i]);
					// Calc diff between slave and master				
				time_to_mstr_diff[i]=sys->timestamp_ns-time_new[i];
				if(time_to_mstr_diff[i]>time_to_mstr_worst[i])
					time_to_mstr_worst[i]=time_to_mstr_diff[i];
				if(time_to_mstr_diff[i]<time_to_mstr_best[i])
					time_to_mstr_best[i]=time_to_mstr_diff[i];
				time_to_mstr_sum[i]+=time_to_mstr_diff[i];
				if(time_to_mstr_diff[i]>HICKUP_THRESHOLD)
					hickups[i]++;
				cntr_sum[i]++;
			}
			time_last[i]=time_new[i];				
		}
	}
	for (i=0;i<sys->slaves_responding;i++)
	{
		for (j=0;j<sys->slaves_responding;j++)
		{
			if ((sys->slave[i].active)&&(sys->slave[j].active)&&(i!=j)&&
						  (time_last[i]!=TIMESTAT_INIT)&&(time_last[j]!=TIMESTAT_INIT))			 
			{
					// Calc diff between slaves				
				time_slaves_diff[i][j]=time_new[i]-time_new[j];
				if(ABS(time_slaves_diff[i][j])>ABS(time_slaves_worst[i][j]))
					time_slaves_worst[i][j]=time_slaves_diff[i][j];
				if(ABS(time_slaves_diff[i][j])<ABS(time_slaves_best[i][j]))
					time_slaves_best[i][j]=time_slaves_diff[i][j];
				time_slaves_sum[i][j]+=time_slaves_diff[i][j];
				cntr_slaves_sum[i][j]++;
			}
		}
	}
	return;
}

void PrintStats(M3EcSystemShm * sys)
{
	int i,j;
	for(i=0;i<sys->slaves_responding;i++)
	{			
		if((sys->slave[i].active)&&(time_last[i]!=TIMESTAT_INIT))
		{
			printf("\n\nTIMESTATS for Slave %d\n", i);
			printf("Worst jitter between steps (ms) : %f\n",NS_TO_MS(time_to_step_worst[i]));
			printf("Best jitter between steps (ms) : %f\n",NS_TO_MS(time_to_step_best[i]));
			printf("Avg jitter mag between steps (ms) : %f\n",NS_TO_MS(time_to_step_sum[i]/cntr_sum[i]));
			printf("\nWorst delay between master (ms) : %f\n",NS_TO_MS(time_to_mstr_worst[i]));
			printf("Best delay between master (ms) : %f\n",NS_TO_MS(time_to_mstr_best[i]));
			printf("Avg delay between master (ms) : %f\n",NS_TO_MS(time_to_mstr_sum[i]/cntr_sum[i]));
			printf("Hickups (delay > %fms) : %d\n",NS_TO_MS(HICKUP_THRESHOLD),hickups[i]);
			for (j=0;j<sys->slaves_responding;j++)
			{
				if((sys->slave[j].active)&&(time_last[j]!=TIMESTAT_INIT)&&(i!=j))
				{					
					printf("\nJitter between slaves %d and %d\n",i,j);		
					printf("Worst jitter between slaves (ms) : %f\n",NS_TO_MS(time_slaves_worst[i][j]));
					printf("Best jitter between slaves (ms) : %f\n",NS_TO_MS(time_slaves_best[i][j]));
					printf("Avg jitter mag between slaves (ms) : %f\n",NS_TO_MS(time_slaves_sum[i][j]/cntr_slaves_sum[i][j]));
				}					
			}
		}
	}
	return;
}

void InitStats(void)
{	
	int i,j;
	for (i=0;i<MAX_NUM_SLAVE;i++)
	{
		hickups[i]=0;
		cntr_sum[i]=0;
		time_last[i]=TIMESTAT_INIT;		
		time_to_step_best[i]=99999999;
		time_to_step_worst[i]=-99999;
		time_to_step_sum[i]=0;		
		time_to_mstr_best[i]=99999999;
		time_to_mstr_worst[i]=-999999;
		time_to_mstr_sum[i]=0;
		for (j=0;j<MAX_NUM_SLAVE;j++)
		{
			time_slaves_best[i][j]=99999999;
			time_slaves_worst[i][j]=-99999;
			time_slaves_sum[i][j]=0;		
		}
	}
	return;
}