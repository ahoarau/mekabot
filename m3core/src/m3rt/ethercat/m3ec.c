/*
Copyright � 2008, Meka Robotics
All rights reserved.
http://mekabot.com

Redistribution and use in source and binary forms, with or without
modification, are permitted. 


THIS SOFTWARE IS PROVIDED BY THE Copyright HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
FOR A PARtICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
Copyright OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIREcT, INDIREcT,
INCIDENTAL, SPEcIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORt (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/
#include <linux/module.h>

#include "../base/m3rt_def.h"
#include "../base/m3ec_def.h"

// Linux

#include <asm/io.h>
//#include <math.h>


// RtAI
#include <rtai.h>
#include <rtai_shm.h>
#include <rtai_sched.h>
#include <rtai_nam2num.h>
#include <rtai_sem.h>
#include <rtai_registry.h>

#include "ecrt.h"
#include "slave.h"
#include "slave_config.h"



//Convenience printf-like macros for printing M3-specific information.
#define M3_INFO(fmt, args...) printk(KERN_INFO "M3 INFO: " fmt, ##args)
#define M3_ERR(fmt, args...) printk(KERN_ERR "M3 ERROR: " fmt, ##args)
#define M3_WARN(fmt, args...) printk(KERN_WARNING "M3 WARNING: " fmt, ##args)

//#define USE_DISTRIBUTED_CLOCKS //Version 1.0 and newer

#define NUM_EC_CYCLES_PER_RT 1
#define RT_KMOD_FREQUENCY (RT_TASK_FREQUENCY*NUM_EC_CYCLES_PER_RT*NUM_EC_DOMAIN)	//Frequency of rt kernel module (HZ) (3000)


typedef struct 
{
	M3EcSystemShm * shm;
	unsigned int offset_status[MAX_NUM_SLAVE];
	unsigned int offset_command[MAX_NUM_SLAVE];	
	uint8_t *domain_pd[NUM_EC_DOMAIN];
	
	ec_slave_config_t  *     slave_config[MAX_NUM_SLAVE];
	ec_slave_config_state_t  slave_state[MAX_NUM_SLAVE];
	
	ec_master_t *master;
	ec_domain_t *domain[NUM_EC_DOMAIN];
	
	ec_master_state_t master_state;
	ec_domain_state_t domain_state[NUM_EC_DOMAIN];
	int num_domain;
	int domain_idx;
		
} M3SystemEc;

/*
Unresolved issue. With many slaves (~35) the total size of a single domain spans
3 ethernet frames. When this happens, the bus starts to bomb. It works OK for two frames. Possible causes:
	* Ethernet NIC buffer size is too small (likely?)
	* Total bandwidth consumed (unlikely)
	* Etherlab Master has too much overhead (maybe)
If we split data into two domains, we can run the loop at 2khz and alternatively update either domain. This is effectively a 1khz update, however the time-stamping will be off by 500us on 1/2 the slaves. Because this works, the issue is not likely bandwidth (dropping to 100hz doesn't fix it either.

Solution (temporary): multiplex domains. We should be able to do all slaves a 1khz. Perhaps should look into the NIC specs.
*/
/*****************************************************************************/
M3SystemEc sys={};

//Things that need to be freed on completion
RT_TASK task;
SEM master_sem;
SEM shm_sem;
SEM sync_sem;


spinlock_t master_lock = SPIN_LOCK_UNLOCKED;


cycles_t t_last_cycle;
cycles_t t_critical;


/*****************************************************************************/


int check_master_state(void)
{
	ec_master_state_t ms;

	spin_lock(&master_lock);
	ecrt_master_state(sys.master, &ms);
	spin_unlock(&master_lock);

	sys.shm->link_up=ms.link_up;
	sys.shm->slaves_responding=ms.slaves_responding;
	
	if (ms.slaves_responding != sys.master_state.slaves_responding)
	{
		M3_INFO("Dropped slaves. Now %u slave(s). Was %u slaves\n",ms.slaves_responding,sys.master_state.slaves_responding);
		sys.shm->slaves_dropped= sys.master_state.slaves_responding-ms.slaves_responding;
		return 0;
	}
		
	if (ms.al_states != sys.master_state.al_states)
		M3_INFO("AL states: 0x%02X.\n", ms.al_states);
	if (ms.link_up != sys.master_state.link_up)
	{
		M3_INFO("Link is %s.\n", ms.link_up ? "up" : "down");
		if (!sys.master_state.link_up)
			return 0;
	}

	sys.master_state = ms;
	return 1;
}
/*****************************************************************************/
void check_domain_state(void)
{
	int i;
	ec_domain_state_t ds;
	for (i=0;i<sys.num_domain;i++)
	{
		spin_lock(&master_lock);
		ecrt_domain_state(sys.domain[i], &ds);
		spin_unlock(&master_lock);

	//This can bog down syslog...
		if (ds.working_counter != sys.domain_state[i].working_counter)
			M3_INFO("Domain: WC %u.\n", ds.working_counter);
		if (ds.wc_state != sys.domain_state[i].wc_state)
			{
				if (sys.domain_state[i].wc_state==EC_WC_INCOMPLETE)
					M3_INFO("Domain: State EC_WC_INCOMPLETE\n");
				if (sys.domain_state[i].wc_state==EC_WC_ZERO)
					M3_INFO("Domain: State EC_WC_ZERO\n");
				if (sys.domain_state[i].wc_state==EC_WC_COMPLETE)
					M3_INFO("Domain: State EC_WC_COMPLETE\n");
		}
		sys.domain_state[i] = ds;
	}
}
/*****************************************************************************/
void check_slave_state(void)
{
	int i;
	M3EcSlaveShm * ss;
	
	for (i=0;i<sys.shm->slaves_responding;i++)
	{
		if (sys.slave_config[i]!=NULL)
		{
			ec_slave_config_state_t s;
			ss=&(sys.shm->slave[i]);
			
			spin_lock(&master_lock);
			ecrt_slave_config_state(sys.slave_config[i], &s);
			spin_unlock(&master_lock);
		
			if (s.al_state != sys.slave_state[i].al_state)
				M3_INFO("Slave %d: State 0x%02X.\n",i, s.al_state);
			if (s.online != sys.slave_state[i].online)
				M3_INFO("Slave %d: %s.\n",i, s.online ? "online" : "offline");
			if (s.operational != sys.slave_state[i].operational)
				M3_INFO("Slave %d: %soperational.\n",i,	s.operational ? "" : "Not ");
			sys.slave_state[i]= s;
			ss->online=s.online;
			ss->al_state=s.al_state;
			ss->operational=s.operational;
		}
	}
}
/*****************************************************************************/
void run(long shm)
{
	static unsigned counter=0;
	int sidx,i;
	int rt_downsample = 0; // only signal rt server every 3 cycles
	M3EcSlaveShm * s;
	RTIME tstart;
	RTIME t_ecat_wait_rx;
	RTIME t_ecat_rx;
	RTIME t_ecat_wait_shm;
	RTIME t_ecat_shm;
	RTIME t_ecat_wait_tx;
	RTIME t_ecat_tx;
	RTIME ts0;
	RTIME ts1;
	RTIME ts2;
	RTIME ts3;
	RTIME ts4;
	RTIME ts5;
	RTIME ts6;
	
	M3_INFO("EtherCAT kernel loop starting...\n");
#ifdef USE_DISTRIBUTED_CLOCKS
	struct timeval tv;
	unsigned int _ref_counter = 0;	
	count2timeval(nano2count(rt_get_real_time_ns()), &tv);
#endif
	sys.shm->counter=0;
	tstart=rt_get_time_ns();
	while (1) {
	    
		t_last_cycle = get_cycles();
	//Process Domain
		ts0=rt_get_time_ns();
		rt_sem_wait(&master_sem);
		ts1=rt_get_time_ns();
		ecrt_master_receive(sys.master); //Get Status data
		ecrt_domain_process(sys.domain[sys.domain_idx]);
		rt_sem_signal(&master_sem);
		ts2=rt_get_time_ns();
#ifdef USE_DISTRIBUTED_CLOCKS	
	// use tv for timestamping to match EC
		tv.tv_usec += RT_KMOD_TIMER_TICKS_NS/1000;
		if (tv.tv_usec >= 1000000)  {
			tv.tv_usec -= 1000000;
			tv.tv_sec++;
		}
#endif
	//Exchange data with shared memory
		rt_sem_wait(&shm_sem);
		ts3=rt_get_time_ns();
#ifdef USE_DISTRIBUTED_CLOCKS	
		sys.shm->timestamp_ns=EC_TIMEVAL2NANO(tv);
#else
		sys.shm->timestamp_ns=rt_get_time_ns()-tstart;
#endif
	
		for (sidx=0;sidx<sys.shm->slaves_responding;sidx++)
		{
			s=&(sys.shm->slave[sidx]);
			if (s->active && (sidx%sys.num_domain)==sys.domain_idx)
			{
				unsigned char * ps = sys.domain_pd[sidx%sys.num_domain]+sys.offset_status[sidx];
				unsigned char * pc = sys.domain_pd[sidx%sys.num_domain]+sys.offset_command[sidx];
				memcpy(s->status,ps, s->n_byte_status);
				memcpy(pc,s->cmd,s->n_byte_cmd);
			}
		}	
	
		rt_sem_signal(&shm_sem);
		if (sys.domain_idx%NUM_EC_DOMAIN==0)
		{		      
			sys.shm->counter++;
			rt_downsample++; 
			if (rt_downsample == NUM_EC_CYCLES_PER_RT)
			{
			  rt_sem_signal(&sync_sem);
			  rt_downsample = 0;
			}
		}
		ts4=rt_get_time_ns();
	//Send data out
#ifdef USE_DISTRIBUTED_CLOCKS	
	// Set Slave DC Ref Clks
		ecrt_master_application_time(sys.master, EC_TIMEVAL2NANO(tv));
		if (sync_ref_counter) {
			sync_ref_counter--;
		} else {
			sync_ref_counter = 9;
			ecrt_master_sync_reference_clock(sys.master);
		}
#endif
		rt_sem_wait(&master_sem);
		ts5=rt_get_time_ns();
		ecrt_domain_queue(sys.domain[sys.domain_idx]);
		ecrt_master_send(sys.master);
		rt_sem_signal(&master_sem);
		ts6=rt_get_time_ns();
		// calc timing for monitor:		
		t_ecat_wait_rx = ts1 - ts0;
		t_ecat_rx = ts2 - ts1;
		t_ecat_wait_shm = ts3 - ts2;
		t_ecat_shm = ts4 - ts3;
		t_ecat_wait_tx = ts5 - ts4;
		t_ecat_tx = ts6 - ts5;
	      
		sys.shm->monitor[sys.domain_idx].t_ecat_wait_rx = t_ecat_wait_rx;
		sys.shm->monitor[sys.domain_idx].t_ecat_rx = t_ecat_rx;
		sys.shm->monitor[sys.domain_idx].t_ecat_wait_shm = t_ecat_wait_shm;
		sys.shm->monitor[sys.domain_idx].t_ecat_shm = t_ecat_shm;
		sys.shm->monitor[sys.domain_idx].t_ecat_wait_tx = t_ecat_wait_tx;
		sys.shm->monitor[sys.domain_idx].t_ecat_tx = t_ecat_tx;
		
		sys.domain_idx=(sys.domain_idx+1)%sys.num_domain;
	
		if (counter) 
			counter--;
		else 
		{
			counter=RT_STATUS_FREQUENCY;
			check_domain_state();
			check_slave_state();
			if (!check_master_state())
				goto run_cleanup; 
		}
		rt_task_wait_period();
	}
run_cleanup:
	M3_INFO("Entering Run Cleanup...\n");
	//Set all cmds to 0. By convention, all M3 products safe with this cmd.
	for (sidx=0;sidx<sys.shm->slaves_responding;sidx++)
	{
		s=&(sys.shm->slave[sidx]);
		if (s->active)
		{
			unsigned char * pc = sys.domain_pd[sidx%sys.num_domain]+sys.offset_command[sidx];
			memset(pc,0,MAX_PDO_SIZE_BYTES);
			s->active=0;

		}
	}
	for (i=0;i<sys.num_domain;i++)
	{
		rt_sem_wait(&master_sem);
		ecrt_domain_queue(sys.domain[i]);
		ecrt_master_send(sys.master);
		rt_sem_signal(&master_sem);
	}
}

/*****************************************************************************/

void request_lock(void *shm)
{
    // too close to the next real time cycle: deny access...
	if (get_cycles() - t_last_cycle > t_critical) return;
    // allow access
	rt_sem_wait(&master_sem);	
}

/*****************************************************************************/

void release_lock(void *shm)
{
	rt_sem_signal(&master_sem);
}

/*****************************************************************************/
int m3sys_startup(void)
{
	 
	int sidx,i;
	int pcode;
	M3EcSlaveShm * s;
	
	if (!(sys.master = ecrt_request_master(0))) {
		M3_ERR("Requesting master 0 failed!\n");
		return 0;
	}
	ecrt_master_callbacks(sys.master, request_lock, release_lock, NULL);

	ecrt_master_state(sys.master, &sys.master_state);
	sys.shm->slaves_responding=sys.master_state.slaves_responding; 
	M3_INFO("Slaves Responding: %d\n",sys.shm->slaves_responding);

	sys.num_domain=MAX(1,MIN(sys.shm->slaves_responding,NUM_EC_DOMAIN));
	M3_INFO("Creating %d domains...\n",sys.num_domain);
	sys.domain_idx=0;
	for (i=0;i<sys.num_domain;i++)
	{
		if (!(sys.domain[i] = ecrt_master_create_domain(sys.master))) {
			M3_ERR("Domain creation failed!\n");
			goto out_release_master;
		}
	}
	
	M3_INFO("Registering PDOs...\n");
	//Search for an M3 product code for each slave
	sys.shm->slaves_active=0;
	for (sidx=0;sidx<sys.shm->slaves_responding;sidx++)
	{
		s=&(sys.shm->slave[sidx]);
		M3_INFO("Registering slave: %d\n",sidx);
		int found=0;
		for (pcode=M3_PRODUCT_CODE_START;pcode<=M3_PRODUCT_CODE_END;pcode++)
		{
			s->network_id=sidx;
			s->active=0;
			s->product_code=-1;
			s->serial_number=-1;

			sys.slave_config[sidx]=ecrt_master_slave_config(
					sys.master,
     0,
     sidx,
     MEKA_VENDOR_ID,
     pcode);
					
			if (sys.slave_config[sidx]!=NULL)
			{
				if (sys.slave_config[sidx]->slave) //Attached
				{
					found=1;
					M3_INFO("Attached Slave %d to Product Id %d\n",sidx,pcode);
					s->product_code=pcode;
					s->serial_number=sys.slave_config[sidx]->slave->sii.serial_number;
					sys.shm->slaves_active++;
					s->active=1;
					
					ec_pdo_t *pdo, *next_pdo;
					ec_pdo_entry_t * pe, *npe;
					s->n_byte_status=0;
					s->n_byte_cmd=0;
					//In M3 EEPROM standard, SYNCM0 is for Command, SYNCM1 is for Status
					//The status/command blocks are broken up into n entries of max 30 bytes each
					//Since 255 is max size of bit_length field
					list_for_each_entry_safe(pdo, next_pdo, &sys.slave_config[sidx]->slave->sii.pdos, list) 
					{
						list_for_each_entry_safe(pe, npe, &pdo->entries, list) 
						{
							if (pdo->sync_index==0)
								s->n_byte_cmd+=pe->bit_length/8;
							if (pdo->sync_index==1)
								s->n_byte_status+=pe->bit_length/8;
						}
					}
					M3_INFO("Slave %d PDO Byte Sizes: Command %d, Status %d\n",sidx,s->n_byte_cmd,s->n_byte_status);					
					break;
				}
				else
				{
					//This is ugly hack into master, hopefully newer rev will allow graceful failure on adding slaves
					//M3_INFO("Failed to attach Slave %d to Product Id %d\n",sidx,product_codes[pidx]);
					list_del(&(sys.slave_config[sidx]->list));
					ec_slave_config_clear(sys.slave_config[sidx]);
				}
			}
		}
		if (!found)
			M3_WARN("Slave %d was not matched to an M3 EtherCAT product\n", sidx);
	}
	//Assign the PDOs for the M3 Slaves
	for (sidx=0;sidx<sys.shm->slaves_responding;sidx++)
	{
		s=&(sys.shm->slave[sidx]);
		if (sys.slave_config[sidx]!=NULL && s->active)
		{
			sys.offset_status[sidx]=ecrt_slave_config_reg_pdo_entry( 
					sys.slave_config[sidx],
     M3EC_PDO_STATUS_INDEX,
     M3EC_PDO_STATUS_SUBINDEX,
     sys.domain[sidx%sys.num_domain],NULL );	
			if (sys.offset_status[sidx]<0)
			{
				M3_ERR("ecrt_slave_config_reg_pdo_entry STATUS failed for slave %d with %d\n", sidx,sys.offset_status[sidx]);
				goto out_release_master;
			}
			sys.offset_command[sidx]=ecrt_slave_config_reg_pdo_entry( 
					sys.slave_config[sidx],
     M3EC_PDO_CMD_INDEX,
     M3EC_PDO_CMD_SUBINDEX,
     sys.domain[sidx%sys.num_domain],NULL );
			if (sys.offset_command[sidx]<0)
			{
				M3_ERR("ecrt_slave_config_reg_pdo_entry CMD failed for slave %d with %d\n", sidx,sys.offset_command[sidx]);
				goto out_release_master;
			}
#ifdef USE_DISTRIBUTED_CLOCKS
			// configure SYNC signals for slaves
			M3_INFO("Setting up SYNC0 for Slave %d\n", sidx);
			ecrt_slave_config_dc(sys.slave_config[sidx], (uint16_t)0x0300, (uint32_t)1000000, (uint32_t)0, 0, 0);
#endif
		}
	}

	M3_INFO("Successful Setup of all slaves\n");

	M3_INFO("Activating master...\n");
	if (ecrt_master_activate(sys.master)) {
		M3_ERR("Failed to activate master!\n");
		goto out_release_master;
	}
	for (i=0;i<sys.num_domain;i++)
	{
		sys.domain_pd[i] = ecrt_domain_data(sys.domain[i]);
		int ps=ecrt_domain_size (sys.domain[i]);   	
		M3_INFO("Allocated Process Data of size %d for domain %d\n",ps,i);
	}
	return 1;
	
out_release_master:
		M3_ERR("Releasing master...\n");
	ecrt_release_master(sys.master);
	return 0;
	
}

int __init init_mod(void)
{
	RTIME tick_period, requested_ticks, now;

	rt_sem_init(&master_sem, 1);
	rt_sem_init(&shm_sem, 1);
	rt_sem_init(&sync_sem, 0); //Only signaled when ec update done
	rt_register(nam2num(SEMNAM_M3LSHM),&shm_sem,IS_SEM,0);
	rt_register(nam2num(SEMNAM_M3SYNC),&sync_sem,IS_SEM,0);
	M3_INFO("Starting...\n");
	sys.shm= rtai_kmalloc(nam2num(SHMNAM_M3MKMD), sizeof(M3EcSystemShm));
	memset(sys.shm,0,sizeof(M3EcSystemShm));
	M3_INFO("Allocated M3System shared memory of size: %u.\n",(int)sizeof(M3EcSystemShm));
	if (!m3sys_startup())
		goto out_return;
	t_critical = cpu_khz * 1000 / RT_KMOD_FREQUENCY - cpu_khz * RT_INHIBIT_TIME / 1000;
	M3_INFO("Starting cyclic sample thread...\n");
	requested_ticks = nano2count(RT_KMOD_TIMER_TICKS_NS); //
	tick_period = start_rt_timer(requested_ticks);
	M3_INFO("Rt timer started with %i/%i ticks.\n", (int) tick_period, (int) requested_ticks);
	if (rt_task_init(&task, run, 0, RT_STACK_SIZE, RT_TASK_PRIORITY+1, 1, NULL)) {
		M3_ERR("Failed to init RtAI task!\n");
		goto out_free_timer;
	}
	now = rt_get_time();
	if (rt_task_make_periodic(&task, now + tick_period, tick_period)) {
		M3_ERR("Failed to run RtAI task!\n");
		goto out_free_task;
	}
	M3_INFO("Initializion Done....\n");
	return 0;
 out_free_task:
		 rt_task_delete(&task);
 out_free_timer:
		 stop_rt_timer(); 
 out_return:
		 rt_sem_delete(&master_sem);
 rt_sem_delete(&shm_sem);
 rt_sem_delete(&sync_sem);
 rtai_kfree(nam2num(SHMNAM_M3MKMD));
 rt_drg_on_name(nam2num(SEMNAM_M3LSHM));
 rt_drg_on_name(nam2num(SEMNAM_M3SYNC));
 rt_drg_on_name(nam2num(SEMNAM_M3LEXT));
 M3_ERR("Failed to load. Aborting.\n");
 return -1;
}

/*****************************************************************************/

void __exit cleanup_mod(void)
{
	M3_INFO("Stopping...\n");
	rt_task_delete(&task);
	stop_rt_timer();
	ecrt_release_master(sys.master);
	rt_sem_delete(&master_sem);
	rt_sem_delete(&shm_sem);
	rt_sem_delete(&sync_sem);
	rtai_kfree(nam2num(SHMNAM_M3MKMD));
	rt_drg_on_name(nam2num(SEMNAM_M3LSHM));
	rt_drg_on_name(nam2num(SEMNAM_M3SYNC));
	M3_INFO("Unloading.\n");
}

/*****************************************************************************/

module_init(init_mod);
module_exit(cleanup_mod);

/*****************************************************************************/

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Aaron Edsinger edsinger@mekabot.com");	
MODULE_DESCRIPTION("Meka Robotics M3 EtherCAT driver");	

/*****************************************************************************/
