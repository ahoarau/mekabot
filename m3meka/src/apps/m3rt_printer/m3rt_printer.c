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

#include "m3/hardware/m3meka_ec_def.h"
#include "m3rt/base/m3ec_def.h"
#include "m3/hardware/m3ec_pdo_v0_def.h"
#include "m3/hardware/m3ec_pdo_v1_def.h"
#include "m3/hardware/m3ec_pdo_v2_def.h"
#include "m3/hardware/m3ec_pdo_v3_def.h"
#include "m3/hardware/m3ec_pdo_v4_def.h"
#include "m3rt/base/m3rt_def.h"
#include "stdio.h"


////////////////////////////////////////////////////////////////////////////////////
void SlaveEcShmPrettyPrint(M3EcSlaveShm * shm);
void SysEcShmPrettyPrint(M3EcSystemShm * shm);

void M3ActPdoStatusPrettyPrint(M3ActPdoV1Status * d, int sn,int ch);
void M3ActPdoCommandPrettyPrint(M3ActPdoV1Cmd * d, int sn,int ch);
void M3ActPdoV4StatusPrettyPrint(M3ActPdoV4Status * d, int sn, int ch);
void M3ActX1PdoStatusPrettyPrint(M3ActX1PdoV1Status * d, int sn);
void M3ActX1PdoV2StatusPrettyPrint(M3ActX1PdoV2Status * d, int sn);
void M3ActX1PdoV4StatusPrettyPrint(M3ActX1PdoV4Status * d, int sn);
void M3ActX1PdoCommandPrettyPrint(M3ActX1PdoV1Cmd * d, int sn);
void M3ActX1PdoV2CommandPrettyPrint(M3ActX1PdoV2Cmd * d, int sn);
void M3ActX1PdoV4CommandPrettyPrint(M3ActX1PdoV4Cmd * d, int sn);
void M3ActX2PdoStatusPrettyPrint(M3ActX2PdoV1Status * d, int sn);
void M3ActX2PdoCommandPrettyPrint(M3ActX2PdoV1Cmd * d, int sn);
void M3ActX3PdoStatusPrettyPrint(M3ActX3PdoV1Status * d, int sn);
void M3ActX3PdoCommandPrettyPrint(M3ActX3PdoV1Cmd * d, int sn);
void M3ActX4PdoStatusPrettyPrint(M3ActX4PdoV1Status * d, int sn);
void M3ActX4PdoCommandPrettyPrint(M3ActX4PdoV1Cmd * d, int sn);
void M3TactX2PdoStatusPrettyPrint(M3TactX2PdoV1Status * d, int sn);
void M3TactX2PdoCommandPrettyPrint(M3TactX2PdoV1Cmd * d, int sn);
void M3LoadX6PdoV1StatusPrettyPrint(M3LoadX6PdoV1Status * d, int sn);
void M3LoadX6PdoV1CommandPrettyPrint(M3LoadX6PdoV1Cmd * d, int sn);
void M3PwrPdoV1StatusPrettyPrint(M3PwrPdoV1Status * d, int sn);
void M3PwrPdoV1CommandPrettyPrint(M3PwrPdoV1Cmd * d, int sn);
void M3PwrPdoV2StatusPrettyPrint(M3PwrPdoV2Status * d, int sn);
void M3PwrPdoV2CommandPrettyPrint(M3PwrPdoV2Cmd * d, int sn);
void M3LedX2PdoCmdPrettyPrint(M3LedX2PdoV1Cmd * dd, int sn);
void M3LedX2PdoStatusPrettyPrint(M3LedX2PdoV1Status * dd, int sn);

//////// Legacy /////////////
void M3GmbPdoCmdV0PrettyPrint(M3GmbX2PdoV0Cmd * dd);
void M3GmbPdoStatusV0PrettyPrint(M3GmbX2PdoV0Status * dd);
void M3SeaPdoV0StatusPrettyPrint(M3SeaPdoV0Status * d,int sn);
void M3SeaPdoV0CommandPrettyPrint(M3SeaPdoV0Cmd * d,int sn);
void M3PwrPdoV0StatusPrettyPrint(M3PwrPdoV0Status * d, int sn);
void M3PwrPdoV0CommandPrettyPrint(M3PwrPdoV0Cmd * d, int sn);
void M3LoadX6PdoV0StatusPrettyPrint(M3LoadX6PdoV0Status * d, int sn);
void M3LoadX6PdoV0CommandPrettyPrint(M3LoadX6PdoV0Cmd * d, int sn);
////////////////////////////////////////////////////////////////////////////////////
static int sys_thread_active = 0;
static int sys_thread_end=0;
static int end=0;
static int hst;
static void endme(int dummy) { end=1; }
////////////////////////////////////////////////////////////////////////////////////

static inline void count2timeval(RTIME rt, struct timeval *t)
{
	t->tv_sec = rtai_ulldiv(count2nano(rt), 1000000000, (unsigned long *)&t->tv_usec);
	t->tv_usec /= 1000;
}

static void* rt_system_thread(void * arg)
{
	struct timeval tv;	
	int64_t ts1, ts2;
	SEM * shm_sem;
	SEM * sync_sem;
	RT_TASK *task;
	
	M3EcSystemShm * sys = (M3EcSystemShm *)arg;
	printf("Starting real-time thread\n",0);
	RTIME t_last;
	int cntr=0;
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
	//else
	//	printf("Allocated shm_sem semaphore  %08x \n",shm_sem);
	
	sync_sem=(SEM*)rt_get_adr(nam2num(SEMNAM_M3SYNC));
	if (!sync_sem)
	{
		printf("Unable to find the SEMNAM_M3SYNC semaphore.\n",0);
		rt_task_delete(task);
		rt_sem_delete(shm_sem);
		return 0;
	}
	//else
	//	printf("Allocated sync_sem semaphore  %08x \n",sync_sem);
	
	RTIME tick_period = nano2count(RT_TIMER_TICKS_NS); 
	RTIME now = rt_get_time();
	rt_task_make_periodic(task, now + tick_period, tick_period); 
	mlockall(MCL_CURRENT | MCL_FUTURE);
	rt_make_hard_real_time();
	t_last=now;
	sys_thread_active=1;
	uint64_t tl;
	while(!sys_thread_end)
	{
		rt_sem_wait(sync_sem);
		rt_sem_wait(shm_sem);
		if (cntr%200==0)
		{
			now=rt_get_time_ns();
			float dt = (now-t_last)/1000000.0;
			count2timeval(nano2count(rt_get_real_time_ns()), &tv);
			printf("\n\nM3 Cycle: %d: 200 cycles in %4.3f ms. EC cycles: %d\n", cntr,dt, sys->counter);
			printf("DT: timestamp_dt (uS) : %lld\n",(sys->timestamp_ns-tl)/1000);
			t_last=now;
			SysEcShmPrettyPrint(sys);
		}
		tl=sys->timestamp_ns;
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
	
	M3EcSystemShm * sys;
	RT_TASK *task;
	pthread_t ptsys;
	int cntr=0;
	
	signal(SIGINT, endme);

	sys = rtai_malloc (nam2num(SHMNAM_M3MKMD),1);
	if (sys==-1) 
	{
		printf("Error allocating shared memory\n");
		return 0;
	}
	int ns=sys->slaves_active;
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

int64_t ts1_last=10000000;
int tcnt=0;
////////////////////////////////////////////////////////////////////////////////////
void SysEcShmPrettyPrint(M3EcSystemShm * shm)
{
	int64_t ts1, ts2;
	int i;
	printf("----- SysEcShm -----\n");
	printf("slaves_responding : %d\n",shm->slaves_responding );
	printf("slaves_active : %d\n",shm->slaves_active );
	printf("slaves_dropped : %d\n",shm->slaves_dropped );
	printf("link_up : %d\n",shm->link_up );
	printf("watchdog : %d\n",shm->watchdog );
	printf("timestamp_ns : %lld\n",shm->timestamp_ns );

	for (i=0;i<shm->slaves_responding;i++)
		if (shm->slave[i].active)
			SlaveEcShmPrettyPrint(&(shm->slave[i]));		       	
}

//////////////////////////LEGACY///////////////////////////////////////////////////////
void M3SeaPdoV0StatusPrettyPrint(M3SeaPdoV0Status * d,int sn)
{
	printf("----- Status -----\n",0);
	printf("sn %d: timestamp: %d\n",sn,(int) d->timestamp);
	printf("sn %d: qei_period: %d\n",sn,(int) d->qei_period);
	printf("sn %d: p_term: %d\n",sn,(int) d->p_term);
	printf("sn %d: i_term: %d\n",sn,(int) d->i_term);
	printf("sn %d: d_term: %d\n",sn,(int) d->d_term);
	printf("sn %d: qei_on: %d\n",sn,(int) d->qei_on);
	printf("sn %d: debug: %d\n",sn,(int) d->debug);
	printf("sn %d: adc_torque: %d\n",sn,(int) d->adc_torque);
	printf("sn %d: adc_motor_temp: %d\n",sn,(int) d->adc_motor_temp);
	printf("sn %d: adc_ext: %d\n",sn,(int) d->adc_ext);
	printf("sn %d: adc_amp_temp: %d\n",sn,(int) d->adc_amp_temp);
	printf("sn %d: adc_current_a: %d\n",sn,(int) d->adc_current_a);
	printf("sn %d: adc_current_b: %d\n",sn,(int) d->adc_current_b);
	printf("sn %d: pwm_cmd: %d\n",sn,(int) d->pwm_cmd);
	printf("sn %d: flags: %d\n",sn,(int) d->flags);
}
///////////////////////////LEGACY/////////////////////////////////////////////////////
void M3SeaPdoV0CmdPrettyPrint(M3SeaPdoV0Cmd * d,int sn)
{
	printf("----- Command -----\n",0);
	printf("sn %d: k_p: %d\n",sn,(int) d->k_p);
	printf("sn %d: k_i: %d\n",sn,(int) d->k_i);
	printf("sn %d: k_d: %d\n",sn,(int) d->k_d);
	printf("sn %d: k_p_shift: %d\n",sn,(int) d->k_p_shift);
	printf("sn %d: k_i_shift: %d\n",sn,(int) d->k_i_shift);
	printf("sn %d: k_d_shift: %d\n",sn,(int) d->k_d_shift);
	printf("sn %d: k_i_limit: %d\n",sn,(int) d->k_i_limit);
	printf("sn %d: t_desire: %d\n",sn,(int) d->t_desire);
	printf("sn %d: t_max: %d\n",sn,(int) d->t_max);
	printf("sn %d: t_min: %d\n",sn,(int) d->t_min);
	printf("sn %d: mode: %d\n",sn,(int) d->mode);
	printf("sn %d: config: %d\n",sn,(int) d->config);
}
///////////////////////////LEGACY///////////////////////////////////////////////////
void M3GmbPdoV0StatusPrettyPrint(M3GmbX2PdoV0Status * dd, int sn)
{
	int i,j,k,m;
	printf("----- Status -----\n",0);
	int nb=sizeof(M3GmbX2PdoV0Status)/2;
	int16_t * p=&dd;

	printf("timestamp: %d\n",(int) dd->timestamp);
	for (i=0;i<2;i++)
	{
		M3GmbPdoV0Status * d = &(dd->status[i]);
		M3TactilePPS22PdoV0Status * t= &(dd->tactile[i]);
		printf("------------ CHID %d -------------\n",i);
		printf("id %d-%d: qei_period: %d\n",sn,i,(int) d->qei_period);
		printf("id %d-%d: qei_on: %d\n",sn,i,(int) d->qei_on);
		printf("id %d-%d: debug: %d\n",sn,i,(int) d->debug);
		printf("id %d-%d: adc_torque: %d\n",sn,i,(int) d->adc_torque);
		printf("id %d-%d: adc_motor_temp: %d\n",sn,i,(int) d->adc_motor_temp);
		printf("id %d-%d: adc_amp_temp: %d\n",sn,i,(int) d->adc_amp_temp);
		printf("id %d-%d: adc_ext_a: %d\n",sn,i,(int) d->adc_ext_a);
		printf("id %d-%d: adc_ext_b: %d\n",sn,i,(int) d->adc_ext_b);
		printf("id %d-%d: pwm_cmd: %d\n",sn,i,(int) d->pwm_cmd);
		printf("id %d-%d: flags: %d\n",sn,i,(int) d->flags);
		for (j=0;j<22;j++)
		{
			printf("id %d-%d: taxel %d: %d\n",sn,i,j,t->taxel[j]);
		}
	}
}
///////////////////////////LEGACY///////////////////////////////////////////////////
void M3GmbPdoV0CmdPrettyPrint(M3GmbX2PdoV0Cmd * dd, int sn)
{
	int i;
	printf("----- Command -----\n",0);
	for (i=0;i<2;i++)
	{
		printf("------------ CHID %d -------------\n",i);
		M3GmbPdoV0Cmd * d = &(dd->command[i]);
		printf("id %d-%d: k_p: %d\n",sn,i,(int) d->k_p);
		printf("id %d-%d: k_i: %d\n",sn,i,(int) d->k_i);
		printf("id %d-%d: k_d: %d\n",sn,i,(int) d->k_d);
		printf("id %d-%d: k_p_shift: %d\n",sn,i,(int) d->k_p_shift);
		printf("id %d-%d: k_i_shift: %d\n",sn,i,(int) d->k_i_shift);
		printf("id %d-%d: k_d_shift: %d\n",sn,i,(int) d->k_d_shift);
		printf("id %d-%d: k_i_limit: %d\n",sn,i,(int) d->k_i_limit);
		printf("id %d-%d: t_desire: %d\n",sn,i,(int) d->t_desire);
		printf("id %d-%d: t_max: %d\n",sn,i,(int) d->t_max);
		printf("id %d-%d: t_min: %d\n",sn,i,(int) d->t_min);
		printf("id %d-%d: mode: %d\n",sn,i,(int) d->mode);
		printf("id %d-%d: config: %d\n",sn,i,(int) d->config);
	}
}
////////////////////////////////////////////////////////////////////////////////////
void M3ActPdoStatusPrettyPrint(M3ActPdoV1Status * d, int sn, int ch)
{
	printf("id %d-%d: qei_period: %d\n",sn,ch,(int) d->qei_period);
	printf("id %d-%d: qei_on: %d\n",sn,ch,(int) d->qei_on);
	printf("id %d-%d: qei_rollover: %d\n",sn,ch,(int) d->qei_rollover);
	printf("id %d-%d: debug: %llu\n",sn,ch,(uint64_t) d->debug);
	printf("id %d-%d: adc_torque: %d\n",sn,ch,(int) d->adc_torque);
	printf("id %d-%d: adc_motor_temp: %d\n",sn,ch,(int) d->adc_motor_temp);
	printf("id %d-%d: adc_amp_temp: %d\n",sn,ch,(int) d->adc_amp_temp);
	printf("id %d-%d: adc_ext_a: %d\n",sn,ch,(int) d->adc_ext_a);
	printf("id %d-%d: adc_ext_b: %d\n",sn,ch,(int) d->adc_ext_b);
	printf("id %d-%d: adc_current_a: %d\n",sn,ch,(int) d->adc_current_a);
	printf("id %d-%d: adc_current_b: %d\n",sn,ch,(int) d->adc_current_b);
	printf("id %d-%d: pwm_cmd: %d\n",sn,ch,(int) d->pwm_cmd);
	printf("id %d-%d: flags: %d\n",sn,ch,(int) d->flags);
}

void M3ActPdoV4StatusPrettyPrint(M3ActPdoV4Status * d, int sn, int ch)
{
	printf("Printing M3ActPdoV3Status\n");
	printf("id %d-%d: qei_period: %d\n",sn,ch,(int) d->qei_period);
	printf("id %d-%d: qei_on: %d\n",sn,ch,(int) d->qei_on);
	printf("id %d-%d: qei_rollover: %d\n",sn,ch,(int) d->qei_rollover);
	printf("id %d-%d: debug: %llu\n",sn,ch,(uint64_t) d->debug);
	printf("id %d-%d: adc_torque: %d\n",sn,ch,(int) d->adc_torque);
	//printf("id %d-%d: adc_motor_temp: %d\n",sn,ch,(int) d->adc_motor_temp);
	printf("id %d-%d: adc_amp_temp: %d\n",sn,ch,(int) d->adc_amp_temp);
	printf("id %d-%d: adc_current_a: %d\n",sn,ch,(int) d->adc_current_a);
	printf("id %d-%d: adc_current_b: %d\n",sn,ch,(int) d->adc_current_b);
	printf("id %d-%d: pwm_cmd: %d\n",sn,ch,(int) d->pwm_cmd);
	printf("id %d-%d: flags: %d\n",sn,ch,(int) d->flags);
	printf("id %d-%d: current_ma: %d\n",sn,ch, (int)d->current_ma);
}


void M3ActPdoV3StatusPrettyPrint(M3ActPdoV3Status * d, int sn, int ch)
{
	printf("Printing M3ActPdoV3Status\n");
	printf("id %d-%d: qei_period: %d\n",sn,ch,(int) d->qei_period);
	printf("id %d-%d: qei_on: %d\n",sn,ch,(int) d->qei_on);
	printf("id %d-%d: qei_rollover: %d\n",sn,ch,(int) d->qei_rollover);
	printf("id %d-%d: debug: %llu\n",sn,ch,(uint64_t) d->debug);
	printf("id %d-%d: adc_torque: %d\n",sn,ch,(int) d->adc_torque);
	printf("id %d-%d: adc_motor_temp: %d\n",sn,ch,(int) d->adc_motor_temp);
	printf("id %d-%d: adc_amp_temp: %d\n",sn,ch,(int) d->adc_amp_temp);
	printf("id %d-%d: adc_current_a: %d\n",sn,ch,(int) d->adc_current_a);
	printf("id %d-%d: adc_current_b: %d\n",sn,ch,(int) d->adc_current_b);
	printf("id %d-%d: pwm_cmd: %d\n",sn,ch,(int) d->pwm_cmd);
	printf("id %d-%d: flags: %d\n",sn,ch,(int) d->flags);
	printf("id %d-%d: current_ma: %d\n",sn,ch, (int)d->current_ma);
}

void M3ActPdoV2StatusPrettyPrint(M3ActPdoV2Status * d, int sn, int ch)
{
	int i;
	printf("id %d-%d: qei_period: %d\n",sn,ch,(int) d->qei_period);
	printf("id %d-%d: qei_on: %d\n",sn,ch,(int) d->qei_on);
	printf("id %d-%d: qei_rollover: %d\n",sn,ch,(int) d->qei_rollover);
	printf("id %d-%d: adc_torque: %d\n",sn,ch,(int) d->adc_torque);
	printf("id %d-%d: pwm_cmd: %d\n",sn,ch,(int) d->pwm_cmd);
	printf("id %d-%d: flags: %d\n",sn,ch,(int) d->flags);
	printf("id %d-%d: ext_start_idx: %d\n",sn,ch,(int) d->ext_start_idx);
	for (i=0;i<M3ACT_PDO_V2_STATUS_EXT_SZ/2;i++)
	  printf("id %d-%d: ext %d:  %d\n",sn,ch,i,((int16_t*) d->ext)[i]);
}

////////////////////////////////////////////////////////////////////////////////////
void M3ActPdoCmdPrettyPrint(M3ActPdoV1Cmd * d, int sn, int ch)
{	 
	printf("id %d-%d: k_p: %d\n",sn,ch,(int) d->k_p);
	printf("id %d-%d: k_i: %d\n",sn,ch,(int) d->k_i);
	printf("id %d-%d: k_d: %d\n",sn,ch,(int) d->k_d);
	printf("id %d-%d: k_ff: %d\n",sn,ch,(int) d->k_ff);
	printf("id %d-%d: k_p_shift: %d\n",sn,ch,(int) d->k_p_shift);
	printf("id %d-%d: k_i_shift: %d\n",sn,ch,(int) d->k_i_shift);
	printf("id %d-%d: k_d_shift: %d\n",sn,ch,(int) d->k_d_shift);
	printf("id %d-%d: k_ff_shift: %d\n",sn,ch,(int) d->k_ff_shift);
	printf("id %d-%d: k_i_limit: %d\n",sn,ch,(int) d->k_i_limit);
	printf("id %d-%d: k_ff_zero: %d\n",sn,ch,(int) d->k_ff_zero);
	printf("id %d-%d: t_desire: %d\n",sn,ch,(int) d->t_desire);	
	printf("id %d-%d: t_max: %d\n",sn,ch,(int) d->t_max);
	printf("id %d-%d: t_min: %d\n",sn,ch,(int) d->t_min);
	printf("id %d-%d: pwm_max: %d\n",sn,ch,(int) d->pwm_max);
	printf("id %d-%d: qei_min: %d\n",sn,ch,(int) d->qei_min);
	printf("id %d-%d: qei_max: %d\n",sn,ch,(int) d->qei_max);
	printf("id %d-%d: mode: %d\n",sn,ch,(int) d->mode);
	printf("id %d-%d: config: %d\n",sn,ch,(int) d->config);		
}

void M3ActPdoV4CmdPrettyPrint(M3ActPdoV4Cmd * d, int sn, int ch)
{	 
	printf("id %d-%d: k_p: %d\n",sn,ch,(int) d->k_p);
	printf("id %d-%d: k_i: %d\n",sn,ch,(int) d->k_i);
	printf("id %d-%d: k_d: %d\n",sn,ch,(int) d->k_d);
	//printf("id %d-%d: k_ff: %d\n",sn,ch,(int) d->k_ff);
	printf("id %d-%d: k_p_shift: %d\n",sn,ch,(int) d->k_p_shift);
	printf("id %d-%d: k_i_shift: %d\n",sn,ch,(int) d->k_i_shift);
	printf("id %d-%d: k_d_shift: %d\n",sn,ch,(int) d->k_d_shift);
	//printf("id %d-%d: k_ff_shift: %d\n",sn,ch,(int) d->k_ff_shift);
	printf("id %d-%d: k_i_limit: %d\n",sn,ch,(int) d->k_i_limit);
	//printf("id %d-%d: k_ff_zero: %d\n",sn,ch,(int) d->k_ff_zero);
	printf("id %d-%d: t_desire: %d\n",sn,ch,(int) d->t_desire);	
	//printf("id %d-%d: t_max: %d\n",sn,ch,(int) d->t_max);
	//printf("id %d-%d: t_min: %d\n",sn,ch,(int) d->t_min);
	printf("id %d-%d: pwm_max: %d\n",sn,ch,(int) d->pwm_max);
	printf("id %d-%d: qei_min: %d\n",sn,ch,(int) d->qei_min);
	printf("id %d-%d: qei_max: %d\n",sn,ch,(int) d->qei_max);
	printf("id %d-%d: mode: %d\n",sn,ch,(int) d->mode);
	printf("id %d-%d: config: %d\n",sn,ch,(int) d->config);		
}


void M3ActPdoV2CmdPrettyPrint(M3ActPdoV2Cmd * d, int sn, int ch)
{
	int i;
	printf("id %d-%d: t_desire: %d\n",sn,ch,(int) d->t_desire);	
	printf("id %d-%d: mode: %d\n",sn,ch,(int) d->mode);
	printf("id %d-%d: config: %d\n",sn,ch,(int) d->config);
	printf("id %d-%d: ext_start_idx: %d\n",sn,ch,(int) d->ext_start_idx);
	for ( i=0;i<M3ACT_PDO_V2_CMD_EXT_SZ/2;i++)
	  printf("id %d-%d: ext %d:  %d\n",sn,ch,i,((int16_t*) d->ext)[i]);
}
////////////////////////////////////////////////////////////////////////////////////
void M3ActX1PdoStatusPrettyPrint(M3ActX1PdoV1Status * d, int sn)
{
	printf("----- Status -----\n",0);
	printf("sn %d: timestamp: %lld\n",sn, (uint64_t)d->timestamp);
	M3ActPdoStatusPrettyPrint(&(d->status[0]),sn,0);
}
void M3ActX1PdoV2StatusPrettyPrint(M3ActX1PdoV2Status * d, int sn)
{
	printf("----- Status -----\n",0);
	printf("sn %d: timestamp: %lld\n",sn, (uint64_t)d->timestamp);
	M3ActPdoV2StatusPrettyPrint(&(d->status[0]),sn,0);
}
void M3ActX1PdoV3StatusPrettyPrint(M3ActX1PdoV3Status * d, int sn)
{
	printf("----- Status -----\n",0);
	printf("sn %d: timestamp: %lld\n",sn, (uint64_t)d->timestamp);
	M3ActPdoV3StatusPrettyPrint(&(d->status[0]),sn,0);
}

void M3ActX1PdoV4StatusPrettyPrint(M3ActX1PdoV4Status * d, int sn)
{
	printf("----- Status -----\n",0);
	printf("sn %d: timestamp: %lld\n",sn, (uint64_t)d->timestamp);
	M3ActPdoV4StatusPrettyPrint(&(d->status[0]),sn,0);
}

void M3ActX1PdoCommandPrettyPrint(M3ActX1PdoV1Cmd * d, int sn)
{
	printf("----- Command -----\n",0);
	M3ActPdoCmdPrettyPrint(&(d->command[0]),sn,0);
}
void M3ActX1PdoV2CommandPrettyPrint(M3ActX1PdoV2Cmd * d, int sn)
{
	printf("----- Command -----\n",0);
	M3ActPdoV2CmdPrettyPrint(&(d->command[0]),sn,0);
}

void M3ActX1PdoV4CommandPrettyPrint(M3ActX1PdoV4Cmd * d, int sn)
{
	printf("----- Command -----\n",0);
	M3ActPdoV4CmdPrettyPrint(&(d->command[0]),sn,0);
}

////////////////////////////////////////////////////////////////////////////////////
void M3ActX2PdoStatusPrettyPrint(M3ActX2PdoV1Status * d, int sn)
{
	int i;
	printf("----- Status -----\n",0);
	printf("sn %d: timestamp: %lld\n",sn, (uint64_t)d->timestamp);
	for (i=0;i<2;i++)
	{
		printf("--- Channel %d ---\n",i);
		M3ActPdoStatusPrettyPrint(&(d->status[i]),sn,i);
	}
}

void M3ActX2PdoCommandPrettyPrint(M3ActX2PdoV1Cmd * d, int sn)
{
	int i;
	printf("----- Command -----\n",0);
	for (i=0;i<2;i++)
	{
		printf("--- Channel %d ---\n",i);
		M3ActPdoCmdPrettyPrint(&(d->command[i]),sn,i);
	}
	
}
////////////////////////////////////////////////////////////////////////////////////
void M3ActX3PdoStatusPrettyPrint(M3ActX3PdoV1Status * d, int sn)
{
	int i;
	printf("----- Status -----\n",0);
	printf("sn %d: timestamp: %lld\n",sn, (uint64_t)d->timestamp);
	for (i=0;i<3;i++)
	{
		printf("--- Channel %d ---\n",i);
		M3ActPdoStatusPrettyPrint(&(d->status[i]),sn,i);
	}
}

void M3ActX3PdoCommandPrettyPrint(M3ActX3PdoV1Cmd * d, int sn)
{
	int i;
	printf("----- Command -----\n",0);
	for (i=0;i<3;i++)
	{
		printf("--- Channel %d ---\n",i);
		M3ActPdoCmdPrettyPrint(&(d->command[i]),sn,i);
	}
}
////////////////////////////////////////////////////////////////////////////////////
void M3ActX4PdoStatusPrettyPrint(M3ActX4PdoV1Status * d, int sn)
{
	int i;
	printf("----- Status -----\n",0);
	printf("sn %d: timestamp: %lld\n", sn,(uint64_t)d->timestamp);
	for (i=0;i<4;i++)
	{
		printf("--- Channel %d ---\n",i);
		M3ActPdoStatusPrettyPrint(&(d->status[i]),sn,i);
	}
}

void M3ActX4PdoCommandPrettyPrint(M3ActX4PdoV1Cmd * d, int sn)
{
	int i;
	printf("----- Command -----\n",0);
	for (i=0;i<4;i++)
	{
		printf("--- Channel %d ---\n",i);
		M3ActPdoCmdPrettyPrint(&(d->command[i]),sn,i);
	}
}

void M3TactX2PdoStatusPrettyPrint(M3TactX2PdoV1Status * d, int sn)
{
	int i,j;
	printf("----- Status -----\n",0);
	printf("sn %d: timestamp: %lld\n", sn,(uint64_t)d->timestamp);
	for (i=0;i<2;i++)
	{
		printf("--- Channel %d ---\n",i);
		M3ActPdoStatusPrettyPrint(&(d->status[i]),sn,i);
		printf("--- Tactile %d \n",i);
		for (j=0;j<22;j++)
		{
			printf("sn %d: taxel %d: %d\n",sn,i,j,d->tactile[i].taxel[j]);
		}

	}
}

void M3TactX2PdoCommandPrettyPrint(M3TactX2PdoV1Cmd * d, int sn)
{
	M3ActX2PdoCommandPrettyPrint((M3ActX2PdoV1Cmd *) d,sn);
}

///////////////////////////////LEGACY/////////////////////////////////////////////////////
void M3PwrPdoV0CmdPrettyPrint(M3PwrPdoV0Cmd * d,int sn)
{
	printf("----- Command -----\n",0);
	printf("sn %d: config: %d\n",sn,(int) d->config);
	printf("sn %d: enable_motor: %d\n",sn,(int) d->enable_motor);
}
/////////////////////////////LEGACY///////////////////////////////////////////////////////
void M3PwrPdoV0StatusPrettyPrint(M3PwrPdoV0Status * d,int sn)
{
	printf("----- Status -----\n",0);
	printf("sn %d: timestamp: %d\n",sn,(int) d->timestamp);
	printf("sn %d: mode_remote: %d\n",sn,(int) d->mode_remote);
	printf("sn %d: motor_enabled: %d\n",sn,(int) d->motor_enabled);
	printf("sn %d: adc_bus_voltage: %d\n",sn,(int) d->adc_bus_voltage);
	printf("sn %d: adc_current_digital: %d\n",sn,(int) d->adc_current_digital);
	printf("sn %d: adc_ext: %d\n",sn,(int) d->adc_ext);
	printf("sn %d: flags: %d\n",sn,(int) d->flags);
}
////////////////////////////////////////////////////////////////////////////////////
void M3PwrPdoV1CmdPrettyPrint(M3PwrPdoV1Cmd * d, int sn)
{
	printf("----- Command -----\n",0);
	printf("sn %d: config: %d\n",sn,(int) d->config);
	printf("sn %d: enable_motor: %d\n",sn,(int) d->enable_motor);
}

////////////////////////////////////////////////////////////////////////////////////
void M3PwrPdoV1StatusPrettyPrint(M3PwrPdoV1Status * d, int sn)
{
	printf("----- Status -----\n",0);
	printf("sn %d: timestamp: %lld\n", sn,(uint64_t) d->timestamp);
	printf("sn %d: mode_remote: %d\n",sn,(int) d->mode_remote);
	printf("sn %d: motor_enabled: %d\n",sn,(int) d->motor_enabled);
	printf("sn %d: adc_bus_voltage: %d\n",sn,(int) d->adc_bus_voltage);
	printf("sn %d: adc_current_digital: %d\n",sn,(int) d->adc_current_digital);
	printf("sn %d: adc_ext: %d\n",sn,(int) d->adc_ext);
	printf("sn %d: flags: %d\n",sn,(int) d->flags);
}
////////////////////////////////////////////////////////////////////////////////////
void M3PwrPdoV2CmdPrettyPrint(M3PwrPdoV2Cmd * d, int sn)
{
	d->enable_motor = 1;
  
	printf("----- Command -----\n",0);
	printf("sn %d: config: %d\n",sn,(int) d->config);
	printf("sn %d: enable_motor: %d\n",sn,(int) d->enable_motor);
}

////////////////////////////////////////////////////////////////////////////////////
void M3PwrPdoV2StatusPrettyPrint(M3PwrPdoV2Status * d, int sn)
{
	printf("----- Status -----\n",0);
	printf("sn %d: timestamp: %lld\n", sn,(uint64_t) d->timestamp);
	printf("sn %d: motor_enabled: %d\n",sn,(int) d->motor_enabled);
	printf("sn %d: adc_bus_voltage: %d\n",sn,(int) d->adc_bus_voltage);
	printf("sn %d: adc_current_digital: %d\n",sn,(int) d->adc_current_digital);
	printf("sn %d: adc_ext: %d\n",sn,(int) d->adc_ext);
	printf("sn %d: flags: %d\n",sn,(int) d->flags);
}
/////////////////////////////LEGACY///////////////////////////////////////////////////////
void M3LoadX6PdoV0CmdPrettyPrint(M3LoadX6PdoV0Cmd * d, int sn)
{
	printf("----- Command -----\n",0);
	printf("sn %d: config: %d\n",sn,(int) d->config);
}
//////////////////////////////LEGACY//////////////////////////////////////////////////////
void M3LoadX6PdoV0StatusPrettyPrint(M3LoadX6PdoV0Status * d, int sn)
{
	printf("----- Status -----\n",0);
	printf("sn %d: timestamp: %lld\n",sn,(uint32_t) d->timestamp);
	printf("sn %d: dig_ext_0: %d\n",sn,(int) d->dig_ext_0);
	printf("sn %d: adc_ext_0: %d\n",sn,(int) d->adc_ext_0);
	printf("sn %d: adc_ext_1: %d\n",sn,(int) d->adc_ext_1);
	printf("sn %d: adc_ext_2: %d\n",sn,(int) d->adc_ext_2);
	printf("sn %d: adc_load_0: %d\n",sn,(int) d->adc_load_0);
	printf("sn %d: adc_load_1: %d\n",sn,(int) d->adc_load_1);
	printf("sn %d: adc_load_2: %d\n",sn,(int) d->adc_load_2);
	printf("sn %d: adc_load_3: %d\n",sn,(int) d->adc_load_3);
	printf("sn %d: adc_load_4: %d\n",sn,(int) d->adc_load_4);
	printf("sn %d: adc_load_5: %d\n",sn,(int) d->adc_load_5);
	printf("sn %d: flags: %d\n",sn,(int) d->flags);
}
////////////////////////////////////////////////////////////////////////////////////
void M3LoadX6PdoV1CmdPrettyPrint(M3LoadX6PdoV1Cmd * d, int sn)
{
	printf("----- Command -----\n",0);
	printf("sn %d: config: %d\n",sn,(int) d->config);
}
////////////////////////////////////////////////////////////////////////////////////
void M3LoadX6PdoV1StatusPrettyPrint(M3LoadX6PdoV1Status * d, int sn)
{
	printf("----- Status -----\n",0);
	printf("sn %d: timestamp: %lld\n",sn,(uint64_t) d->timestamp);
	printf("sn %d: dig_ext_0: %d\n",sn,(int) d->dig_ext_0);
	printf("sn %d: adc_ext_0: %d\n",sn,(int) d->adc_ext_0);
	printf("sn %d: adc_ext_1: %d\n",sn,(int) d->adc_ext_1);
	printf("sn %d: adc_ext_2: %d\n",sn,(int) d->adc_ext_2);
	printf("sn %d: adc_load_0: %d\n",sn,(int) d->adc_load_0);
	printf("sn %d: adc_load_1: %d\n",sn,(int) d->adc_load_1);
	printf("sn %d: adc_load_2: %d\n",sn,(int) d->adc_load_2);
	printf("sn %d: adc_load_3: %d\n",sn,(int) d->adc_load_3);
	printf("sn %d: adc_load_4: %d\n",sn,(int) d->adc_load_4);
	printf("sn %d: adc_load_5: %d\n",sn,(int) d->adc_load_5);
	printf("sn %d: flags: %d\n",sn,(int) d->flags);
}
////////////////////////////////////////////////////////////////////////////////////
void M3LedX2PdoCmdPrettyPrint(M3LedX2PdoV1Cmd * dd, int sn)
{
	int i;
	printf("----- Command -----\n",0);
	printf("sn %d: config: %d\n",sn,(int) dd->config);
	printf("sn %d: enable_a: %d\n",sn,(int) dd->enable_a);
	printf("sn %d: enable_b: %d\n",sn,(int) dd->enable_b);
	printf("sn %d:: BranchA.BoardA.RGB: %d %d %d\n",sn,
		   (int) dd->branch_a.board_a.r,(int) dd->branch_a.board_a.g,(int) dd->branch_a.board_a.b);
	printf("sn %d:: BranchA.BoardB.RGB: %d %d %d\n",sn,
		   (int) dd->branch_a.board_b.r,(int) dd->branch_a.board_b.g,(int) dd->branch_a.board_b.b);
	printf("sn %d: BranchB.BoardA.RGB: %d %d %d\n",sn,
		   (int) dd->branch_b.board_a.r,(int) dd->branch_b.board_a.g,(int) dd->branch_b.board_a.b);
	printf("sn %d: BranchB.BoardB.RGB: %d %d %d\n",sn,
		   (int) dd->branch_b.board_b.r,(int) dd->branch_b.board_b.g,(int) dd->branch_b.board_b.b);
}
////////////////////////////////////////////////////////////////////////////////////

void M3LedX2PdoStatusPrettyPrint(M3LedX2PdoV1Status * dd, int sn)
{
	printf("----- Status -----\n");
	printf("sn %d: timestamp: %lld\n",sn, (uint64_t)dd->timestamp);
	printf("sn %d: flags: %d\n",sn,(int) dd->flags);
	printf("sn %d: adc_ext_a: %d\n",sn,(int) dd->adc_ext_a);
	printf("sn %d: adc_ext_b: %d\n",sn,(int) dd->adc_ext_b);
	printf("sn %d: adc_ext_c: %d\n",sn,(int) dd->adc_ext_c);
	printf("sn %d: adc_ext_d: %d\n",sn,(int) dd->adc_ext_d);
	printf("sn %d: debug: %d\n",sn,(int) dd->debug);
}
////////////////////////////////////////////////////////////////////////////////////


void SlaveEcShmPrettyPrint(M3EcSlaveShm * shm)
{
	printf("\n\n----------------- Slave: %d -----------------\n",shm->network_id);
	printf("active : %d\n",shm->active);
	printf("network_id : %d\n",shm->network_id);
	printf("serial_number : %d\n",shm->serial_number);
	printf("product_code : %d\n",shm->product_code);
	
	printf("online : %d\n",shm->online);
	printf("operational : %d\n",shm->operational);
	printf("al_state : %d\n",shm->al_state);
	
	/////////// Legacy ///////////////////
	if (shm->product_code==M3GMB_PRODUCT_CODE)
	{
		M3GmbPdoV0StatusPrettyPrint((M3GmbX2PdoV0Status *) shm->status,shm->serial_number);
		M3GmbPdoV0CmdPrettyPrint((M3GmbX2PdoV0Cmd *) shm->cmd,shm->serial_number);
	}
	if (shm->product_code==M3SEA_PRODUCT_CODE)
	{
		M3SeaPdoV0StatusPrettyPrint((M3SeaPdoV0Status *) shm->status,shm->serial_number);
		M3SeaPdoV0CmdPrettyPrint((M3SeaPdoV0Cmd *) shm->cmd,shm->serial_number);
	}
	////////////////////////////////////////
	if (shm->product_code==M3ACTX1_PRODUCT_CODE)
	{
	   // printf("%d: Status: %d %d %d\n",shm->serial_number,shm->n_byte_status,sizeof(M3ActX1PdoV2Status),MAX_PDO_ENTRY_SIZE);
	  if (shm->n_byte_status == MAX_PDO_ENTRY_SIZE) //V2 is just one entry...hack way to detect.
	  {
	    M3ActX1PdoV2StatusPrettyPrint((M3ActX1PdoV2Status *) shm->status,shm->serial_number);
	    M3ActX1PdoV2CommandPrettyPrint((M3ActX1PdoV2Cmd *) shm->cmd,shm->serial_number);
	  }
	  else
	  {
		  //M3ActX1PdoStatusPrettyPrint((M3ActX1PdoV1Status *) shm->status,shm->serial_number);
		  //M3ActX1PdoV3StatusPrettyPrint((M3ActX1PdoV3Status *) shm->status,shm->serial_number);
		  M3ActX1PdoV4StatusPrettyPrint((M3ActX1PdoV4Status *) shm->status,shm->serial_number);
		  //M3ActX1PdoCommandPrettyPrint((M3ActX1PdoV1Cmd *) shm->cmd,shm->serial_number);
		  M3ActX1PdoV4CommandPrettyPrint((M3ActX1PdoV4Cmd *) shm->cmd,shm->serial_number);
	  }
	}
	if (shm->product_code==M3ACTX2_PRODUCT_CODE)
	{
		M3ActX2PdoStatusPrettyPrint((M3ActX2PdoV1Status *) shm->status,shm->serial_number);
		M3ActX2PdoCommandPrettyPrint((M3ActX2PdoV1Cmd *) shm->cmd,shm->serial_number);
	}
	if (shm->product_code==M3ACTX3_PRODUCT_CODE)
	{
		M3ActX3PdoStatusPrettyPrint((M3ActX3PdoV1Status *) shm->status,shm->serial_number);
		M3ActX3PdoCommandPrettyPrint((M3ActX3PdoV1Cmd *) shm->cmd,shm->serial_number);
	}
	if (shm->product_code==M3ACTX4_PRODUCT_CODE)
	{
		M3ActX4PdoStatusPrettyPrint((M3ActX4PdoV1Status *) shm->status,shm->serial_number);
		M3ActX4PdoCommandPrettyPrint((M3ActX4PdoV1Cmd *) shm->cmd,shm->serial_number);
	}
	
	if (shm->product_code==M3TACTX2_PRODUCT_CODE)
	{
		M3TactX2PdoStatusPrettyPrint((M3TactX2PdoV1Status *) shm->status,shm->serial_number);
		M3TactX2PdoCommandPrettyPrint((M3TactX2PdoV1Cmd *) shm->cmd,shm->serial_number);
	}
	if (shm->product_code==M3LOADX6_PRODUCT_CODE)
	{
	  //Hack. Use SN to determine the PDO version. This is roughly correct.
	    if (shm->serial_number<200 )//V0
	    {
		M3LoadX6PdoV0StatusPrettyPrint((M3LoadX6PdoV0Status *) shm->status,shm->serial_number);
		M3LoadX6PdoV0CmdPrettyPrint((M3LoadX6PdoV0Cmd *) shm->cmd,shm->serial_number);
	    }
	    else//V1
	    {
		M3LoadX6PdoV1StatusPrettyPrint((M3LoadX6PdoV1Status *) shm->status,shm->serial_number);
		M3LoadX6PdoV1CmdPrettyPrint((M3LoadX6PdoV1Cmd *) shm->cmd,shm->serial_number);
	    }
	}
	if (shm->product_code==M3PWR_PRODUCT_CODE)
	{
		//Hack. Use SN to determine the PDO version. This is roughly correct.
		if (shm->serial_number<200 )//PDOV0
		{
			M3PwrPdoV0StatusPrettyPrint((M3PwrPdoV0Status *) shm->status,shm->serial_number);
			M3PwrPdoV0CmdPrettyPrint((M3PwrPdoV0Cmd *) shm->cmd,shm->serial_number);
		}
		if (shm->serial_number>=200 && shm->serial_number<500)//PDOV1
		{
			M3PwrPdoV1StatusPrettyPrint((M3PwrPdoV1Status *) shm->status,shm->serial_number);
			M3PwrPdoV1CmdPrettyPrint((M3PwrPdoV1Cmd *) shm->cmd,shm->serial_number);
		}
		if (shm->serial_number>=500)//PDOV2
		{
			M3PwrPdoV2StatusPrettyPrint((M3PwrPdoV2Status *) shm->status,shm->serial_number);
			M3PwrPdoV2CmdPrettyPrint((M3PwrPdoV2Cmd *) shm->cmd,shm->serial_number);
		}
	}
	if (shm->product_code==M3LEDX2_PRODUCT_CODE)
	{
		M3LedX2PdoStatusPrettyPrint((M3LedX2PdoV1Status *) shm->status,shm->serial_number);
		M3LedX2PdoCmdPrettyPrint((M3LedX2PdoV1Cmd *) shm->cmd,shm->serial_number);
	}
}
////////////////////////////////////////////////////////////////////////////////////
