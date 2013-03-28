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


#include "setup.h"
#include "current.h"
#include "control.h"
#include "pwm.h"
extern int test; //Test

extern int fsa_state[1];

extern unsigned int actual_pwm;	//pwm.c

//Assuming step is called at 2kHz
#define I_RMS_MOM_BUF_SZ 16
#define I_RMS_MOM_BUF_SHIFT 4
#define I_RMS_MOM_DS 248 //2000Hz to 8.06Hz, Buff of 16 gives ~2s window size.

//Assuming step is called at 2kHz
#define I_RMS_CONT_BUF_SZ 32
#define I_RMS_CONT_BUF_SHIFT 5
#define I_RMS_CONT_DS 624 //2000Hz to 3.2Hz. Buff of 32 gives ~10S window size.

volatile int i_mA;
volatile unsigned int i_zero_a, i_zero_b;
volatile int i_zero_cnt;
volatile long i_rms_mom_buf[I_RMS_MOM_BUF_SZ];
volatile long i_rms_cont_buf[I_RMS_CONT_BUF_SZ];
volatile unsigned int i_zero_sum_a, i_zero_sum_b;
volatile long i_rms_cont_sq, i_rms_mom_sq, i_fault_val; //squared value
volatile int i_rms_mom_ds,i_rms_cont_ds;
volatile long i_rms_mom_sum, i_rms_cont_sum;
volatile int i_rms_mom_idx,i_rms_cont_idx;
volatile int i_state,i_fault_cont,i_fault_mom;


int current_fault_mom_flag()
{
   if(i_fault_mom)
     	return M3ACT_FLAG_I_FAULT_MOM;
   else
     	return 0;
}

int current_fault_cont_flag()
{
     if (i_fault_cont)
     	return M3ACT_FLAG_I_FAULT_CONT;
   else
     	return 0;
}

int get_current_ma()
{
  	return i_mA;
}

long get_current_rms_mom_sq_ma()
{
  	return i_rms_mom_sq;
}
long get_current_rms_cont_sq_ma()
{
  	return i_rms_cont_sq;
}
int get_current_state()
{
  	return i_state;
}


//Calculates an average cycle current based on a single peak current measurement
unsigned int correct_mA(unsigned int pwm, unsigned int max_pwm, unsigned int current)
{
	unsigned long result = 0;
	unsigned int ratio = 0;
	
	if(pwm <= 0)
		return 0;
		
	ratio = (max_pwm << 3) / pwm;
	
	result = ((unsigned long)current * (unsigned long)ratio);
	result = result >> 3;
	
	return (unsigned int) result;
}


void reset_current_buf()
{
  	//i_mA=0;
 	i_rms_cont_idx=0;
 	i_rms_cont_sum=0;
 	i_rms_cont_sq=0;	
 	i_rms_cont_ds=0;
	i_rms_mom_idx=0;
	i_rms_mom_sum=0;
	i_rms_mom_sq=0;	
	i_rms_mom_ds=0;
  	memset((long *)i_rms_mom_buf,0,sizeof(long)*I_RMS_MOM_BUF_SZ);
  	memset((long *)i_rms_cont_buf,0,sizeof(long)*I_RMS_CONT_BUF_SZ);
}

void step_current()
{
#if 0
	unsigned int z = 0;
	static unsigned int zero_done = 0;
	
  	//Measure zero sensor reading at startup
	if (i_zero_cnt < 17 && i_zero_cnt > 0 && i_state == CURRENT_STARTUP)
  	{
      	i_zero_sum_a = i_zero_sum_a + get_avg_adc(ADC_CURRENT);
      	if (i_zero_cnt == 1)
      	{
			i_zero_a = (i_zero_sum_a >> 4);
		
			//ToDo: add flag
			//Make sure that the zero is valid
			if((i_zero_a > AMP_MIN) && (i_zero_a < AMP_MAX))
			{
				//Value in the range (+-10% for ACS, close to 0 for shunt), we use it
				i_state = CURRENT_READY;
				zero_done = 1;
			}
			else
			{
				//Try again...
				i_state = CURRENT_STARTUP;
				i_zero_sum_a = 0;
				i_zero_sum_b = 0;
				i_zero_cnt = 100;	
			}
      	}
  	}
  	i_zero_cnt=MAX(0,i_zero_cnt-1);
	
	//Avoid false readings at power-up (ex.: after e-stop is released)
	if((fsa_state[0] == CTRL_OFF || fsa_state[0] == CTRL_ABORT) && zero_done)
		i_state = CURRENT_HOLD;
	
	if (i_state == CURRENT_HOLD)
	{
		i_zero_a = get_avg_adc(ADC_CURRENT);
		
		//Make sure that the value is in the range (the 5VM bus is on)
		if((i_zero_a > AMP_MIN) && (i_zero_a < AMP_MAX))
		{
			//Value in the range (+-10% for ACS, close to 0 for shunt), we use it
			i_state = CURRENT_READY;
		}
		else
		{
			//Try again...
			i_state = CURRENT_HOLD;
		}
	}
	

	if (i_state == CURRENT_READY)
	{
		//Compute 'instantaneous' current
		
		//Hall effect sensor
	  	int x =(int)get_avg_adc(ADC_CURRENT) - (int)i_zero_a;
		i_mA = ((x * I_GAIN) >> I_SHIFT);
		
		//Return 0 if CTRL_OFF or CTRL_ABORT
		if(fsa_state[0] == CTRL_OFF || fsa_state[0] == CTRL_ABORT)
			i_mA = 0;

		//Now compute the momentary RMS value
		i_rms_mom_ds=INC_MOD(i_rms_mom_ds,I_RMS_MOM_DS);
		if (i_rms_mom_ds==0)
		{
			i_rms_mom_idx=INC_MOD(i_rms_mom_idx,I_RMS_MOM_BUF_SZ);	
			i_rms_mom_buf[i_rms_mom_idx]=(long)i_mA*(long)i_mA;		//RMS
					
			i_rms_mom_sum = 0;	
			for(z = 0; z < I_RMS_MOM_BUF_SZ; z++)
		    {
			   	//Sum all values
				i_rms_mom_sum += i_rms_mom_buf[z];
			}
		    i_rms_mom_sq=i_rms_mom_sum>>I_RMS_MOM_BUF_SHIFT;
		    
			if ((!i_fault_mom) && (i_rms_mom_sq > CURRENT_MAX_MOM_RMS_SQ))
			{
				if((i_mA > MAX_MOM_CURRENT) || (i_mA < -MAX_MOM_CURRENT))
				{
					//if (i_state==CURRENT_READY)
					//	i_fault_val=i_rms_mom_sq;
					i_state = CURRENT_FAULT_MOM;
					i_fault_mom = 1;
					reset_current_buf();//power turns off
				}
			}
		}


		//Now compute the continuous RMS value
		i_rms_cont_ds=INC_MOD(i_rms_cont_ds,I_RMS_CONT_DS);
		if (i_rms_cont_ds==0)
		{			
			i_rms_cont_idx=INC_MOD(i_rms_cont_idx,I_RMS_CONT_BUF_SZ);
			i_rms_cont_buf[i_rms_cont_idx]=(long)i_mA*(long)i_mA;
				
			i_rms_cont_sum = 0;	
			for(z = 0; z < I_RMS_CONT_BUF_SZ; z++)
	    	{
				i_rms_cont_sum += i_rms_cont_buf[z];
			}
	    	i_rms_cont_sq=i_rms_cont_sum >> I_RMS_CONT_BUF_SHIFT;
		}
 		if (!i_fault_cont && i_rms_cont_sq>CURRENT_MAX_CONT_RMS_SQ)
		{
		//	if (i_state==CURRENT_READY)
			//	i_fault_val=i_rms_cont_sq;
			i_state=CURRENT_FAULT_CONT;
			i_fault_cont=1;
			reset_current_buf();//power turns off
		}
	}
#endif
}

void setup_current()
{
  	i_mA=0;
  	i_zero_a=0;
  	i_zero_b=0;
  	i_zero_cnt=250;
  	i_zero_sum_a=0;
  	i_zero_sum_b=0;
 	i_rms_cont_idx=0;
 	i_rms_cont_sum=0;
 	i_rms_cont_sq=0;	
 	i_rms_cont_ds=0;
	i_fault_cont=0;
	i_fault_mom=0;
  	i_rms_mom_idx=0;
  	i_rms_mom_sum=0;
  	i_rms_mom_sq=0;	
  	i_rms_mom_ds=0;
  	i_state=CURRENT_STARTUP;
  	memset((long *)i_rms_mom_buf,0,sizeof(long)*I_RMS_MOM_BUF_SZ);
  	memset((long *)i_rms_cont_buf,0,sizeof(long)*I_RMS_CONT_BUF_SZ);
}

