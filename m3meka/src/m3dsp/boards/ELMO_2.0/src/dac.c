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
#ifdef USE_DAC
#include "dac.h"
#include "setup.h"



int dac_idx;
int dac_idx_fast;
int volatile dac_buffer[DAC_NUM_SMOOTH];

 
int get_dac_smooth()
{
	long v;
	int i;
	v=0;
	for (i=0;i<DAC_NUM_SMOOTH;i++)
		v=v+dac_buffer[i];
	return (int)(v>>DAC_SHIFT_SMOOTH);
}

int dac_cmd_val;
int dac_cmd(){return dac_cmd_val;}




unsigned int convert_val_to_unsigned_dac (int sval) 
{
	//converts signed dac command into unsigned command between 0 and DAC_MAX_VAL
	// If sval < -2048, then sval+2048 < 0, so sval is converted to 0
	// If sval > 2047, then sval+2048 > 4095, so sval is converted to 4095
	sval=sval+DAC_ZERO_VAL;
	sval=CLAMP(sval,0,DAC_MAX_VAL);
	sval=sval<<2; //first and last 2 bits are do-not-care
	return (unsigned int)sval;
}

void set_dac(int val)					
{
	int i,pm,dval;
	unsigned int sval;
	unsigned int x;



	volatile int tmp;
        
	dval=val;

	if (val!=0)
	{
		if (ec_cmd.command[0].pwm_db<0)
		{
			if (ABS(dval)<ABS(ec_cmd.command[0].pwm_db))
				dval=0;
		}
		else
		{
			if (dval>0)
				dval=dval+ec_cmd.command[0].pwm_db;
			else
				dval=dval-ec_cmd.command[0].pwm_db;
		}
		pm=MIN(DAC_MAX_DUTY,ec_cmd.command[0].pwm_max);
		if (dval>pm)
			dval=pm;
		if (dval<-pm)
			dval=-pm;
	}
        
	dac_buffer[dac_idx]=dval;
	dac_idx=INC_MOD(dac_idx,ADC_NUM_SMOOTH);
	dac_cmd_val=get_dac_smooth();//This goes back up as for current computations.
	sval=convert_val_to_unsigned_dac(dac_cmd_val);
        //sval=convert_val_to_unsigned_dac(dval);

        tmp++;

#ifdef M3_DEV
	BB_DAC_CS0=0; //DAC on CH0 of DevBoard
	BB_DAC_CS1=0;
#endif
	BB_DAC_SEL=0; //SCLK must fall > 50NS after this
	for (i=0;i<16;i++)
	{
		BB_DAC_CLK=1;
		x=0x8000&sval;
		if (x!=0)
			BB_DAC_DI=1;
		else
			BB_DAC_DI=0;
		sval=sval<<1;
		asm("nop"); //140ns min clock period
		asm("nop"); 
		BB_DAC_CLK=0;
		asm("nop");
		asm("nop");
		asm("nop");
		
	}
	BB_DAC_SEL=1;
}

void setup_dac(void)
{
	BB_DAC_CLK=0;
}



#endif
