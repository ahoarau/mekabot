 /*************************************************************************
 * 
 * REDWOOD CONFIDENTIAL
 * Author: Aaron Edsinger
 * __________________
 * 
 *  [2012] - [+] Redwood Robotics Incorporated 
 *  All Rights Reserved.
 * 
 * All information contained herein is, and remains
 * the property of Redwood Robotics Incorporated and its suppliers,
 * if any.  The intellectual and technical concepts contained
 * herein are proprietary to Redwood Robotics Incorporated
 * and its suppliers and may be covered by U.S. and Foreign Patents,
 * patents in process, and are protected by trade secret or copyright law.
 * Dissemination of this information or reproduction of this material
 * is strictly forbidden unless prior written permission is obtained
 * from Redwood Robotics Incorporated.
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
#include "m3/hardware/led_matrix_ec_shm_sds.h"

// Needed for ROS
#include <ros/ros.h>
#include <shm_led_mouth/LEDMatrixCmd.h>


#define RT_TASK_FREQUENCY_MEKA_LED_SHM 100
#define RT_TIMER_TICKS_NS_MEKA_LED_SHM (1000000000 / RT_TASK_FREQUENCY_MEKA_LED_SHM)		//Period of rt-timer 
#define MEKA_LED_SHM "LSHMM"
#define MEKA_LED_CMD_SEM "LSHMC"
#define MEKA_LED_STATUS_SEM "LSHMS"


#define CYCLE_TIME_SEC 4


////////////////////////////////////////////////////////////////////////////////////
static int sys_thread_active = 0;
static int sys_thread_end=0;
static int end=0;
static int hst;
static M3LedMatrixEcShmSdsCommand cmd;
static M3LedMatrixEcShmSdsStatus status;
static int sds_status_size;
static int sds_cmd_size;
static long step_cnt = 0;
static void endme(int dummy) {  end=1; }

ros::Subscriber cmd_sub_g;
//tf::TransformBroadcaster odom_broadcaster;
////////////////////////////////////////////////////////////////////////////////////


///////  Periodic Control Loop:
void StepShm();
void commandCallback(const shm_led_mouth::LEDMatrixCmdConstPtr& msg);

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

void StepShm(int cntr)
{   
  
    SetTimestamp(GetTimestamp()); //Pass back timestamp as a heartbeat
   
    
     /*if (cntr % 100 == 0)
      {	
	if (1)
	{
	  printf("********************************\n");
	  printf("timestamp: %ld\n", status.timestamp);	  
	  {	    	    
	    printf("------------------------------\n");
	    printf("position: %f\n", status.position);
	    printf("velocity: %f\n", status.velocity);
	    printf("effort: %f\n", status.effort);	    
	     printf("------------------------------\n");
	    printf("\n");
	  }
	}
      }*/
    
    /*cmd.position = 600;
    cmd.velocity = 2000;
    cmd.stiffness = 1.0;
    cmd.control_mode = JOINT_MODE_ROS_THETA_GC;
    cmd.smoothing_mode = SMOOTHING_MODE_SLEW;*/
    
 

}

void commandCallback(const shm_led_mouth::LEDMatrixCmdConstPtr& msg)
{
  
  //printf("cmd!\n");
  
    cmd.enable = msg->enable;
    
  
    for (int i = 0; i < NUM_ROWS; i++)
    {	        
      for (int j = 0; j < NUM_COLS; j++)
      {	
	cmd.r[i][j] = msg->row[i].column[j].r;
	cmd.b[i][j] = msg->row[i].column[j].b;
	cmd.g[i][j] = msg->row[i].column[j].g;		
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
	printf("Starting real-time thread\n");
		
	
	sds_status_size = sizeof(M3LedMatrixEcShmSdsStatus);
	sds_cmd_size = sizeof(M3LedMatrixEcShmSdsCommand);
	
	memset(&cmd, 0, sds_cmd_size);
	
	task = rt_task_init_schmod(nam2num("LSHMP"), 0, 0, 0, SCHED_FIFO, 0xF);
	rt_allow_nonroot_hrt();
	if (task==NULL)
	{
		printf("Failed to create RT-TASK LSHMP\n");
		return 0;
	}
	status_sem=(SEM*)rt_get_adr(nam2num(MEKA_LED_STATUS_SEM));
	command_sem=(SEM*)rt_get_adr(nam2num(MEKA_LED_CMD_SEM));
	if (!status_sem)
	{
		printf("Unable to find the %s semaphore.\n",MEKA_LED_STATUS_SEM);
		rt_task_delete(task);
		return 0;
	}
	if (!command_sem)
	{
		printf("Unable to find the %s semaphore.\n",MEKA_LED_CMD_SEM);
		rt_task_delete(task);
		return 0;
	}
	
	
	RTIME tick_period = nano2count(RT_TIMER_TICKS_NS_MEKA_LED_SHM); 
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
		
		StepShm(cntr);		
		
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
			//rt_task_make_periodic(task, end + tick_period,tick_period);			
		}
		step_cnt++;
		if (cntr++ == CYCLE_TIME_SEC * 2 * RT_TIMER_TICKS_NS_MEKA_LED_SHM)
		  cntr = 0;
		rt_task_wait_period();
	}	
	printf("Exiting RealTime Thread...\n",0);
	rt_make_soft_real_time();
	rt_task_delete(task);
	sys_thread_active=0;
	return 0;
}



////////////////////////////////////////////////////////////////////////////////////
int main (int argc, char **argv)
{	
	RT_TASK *task;
	M3Sds * sys;
	int cntr=0;
	
	rt_allow_nonroot_hrt();
	
	/*ros::init(argc, argv, "base_controller"); // initialize ROS node
  	ros::AsyncSpinner spinner(1); // Use 1 thread - check if you actually need this for only publishing
  	spinner.start();
        ros::NodeHandle root_handle;*/
	
	ros::init(argc, argv, "base_controller", ros::init_options::NoSigintHandler); // initialize ROS node
	ros::AsyncSpinner spinner(1); // Use 1 thread - check if you actually need this for only publishing
	spinner.start();
        ros::NodeHandle root_handle;
	ros::NodeHandle p_nh("~");
	
	cmd_sub_g = root_handle.subscribe<shm_led_mouth::LEDMatrixCmd>("/led_matrix_command", 1, &commandCallback);
	
	
	signal(SIGINT, endme);

	if (sys = (M3Sds*)rt_shm_alloc(nam2num(MEKA_LED_SHM),sizeof(M3Sds),USE_VMALLOC))
		printf("Found shared memory starting shm_zlift_controller.");
	else
	{
		printf("Rtai_malloc failure for %s\n",MEKA_LED_SHM);
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
		rt_shm_free(nam2num(MEKA_LED_SHM));
		printf("Startup of thread failed.\n",0);
		return 0;
	}
	while(!end)
	{		
		usleep(250000);
		
	}
	printf("Removing RT thread...\n",0);
	sys_thread_end=1;
	//rt_thread_join(hst);
	usleep(1250000);
	if (sys_thread_active)printf("Real-time thread did not shutdown correctly\n");
	//rt_task_delete(task);
	rt_shm_free(nam2num(MEKA_LED_SHM));
	ros::shutdown();	
	return 0;
}

