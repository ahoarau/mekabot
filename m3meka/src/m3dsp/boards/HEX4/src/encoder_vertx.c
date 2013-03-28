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

#ifdef USE_ENCODER_VERTX

#include "setup.h"
#include "encoder_vertx.h"

unsigned int vertx_val[NUM_VERTX_CH];

int vertx_idx;
unsigned int  vertx_buffer[NUM_VERTX_CH][VERTX_NUM_SMOOTH];
unsigned int  vertx_error_buffer[NUM_VERTX_CH];

int vertx_error(int chid)
{
	return vertx_error_buffer[chid];
}
unsigned int get_avg_vertx(int ch)
{
	long v;
	int i;
	v=0;
	for (i=0;i<VERTX_NUM_SMOOTH;i++)
		v=v+vertx_buffer[ch][i];
	return (unsigned int)(v>>VERTX_SHIFT_SMOOTH);
}

//Unclear why, very occasionnal peg to max. Filter condition for now.
//Need to understand.
void vertx_error_filter(int ch,unsigned int val)
{
	/*if (val&&0x0003 !=0x0003)
	{
		vertx_error_buffer[ch]=val;
		return vertx_val[ch];
	}*/
	//Assume that encoder never gets near roll-over point
	//Max is 16384. Error code should be caught with first test
	//Second test is in case the error-code bits aren't set, which
	//has been observerd.
	//Assume chid=0 for all VERTX enabled systems

	int chid=0; //For dual encoder actuators, use only chid=0 for the config flag

	//	if ((val&(unsigned int)0x0003)!=1 || (val>>2)>16350 || (val>>2)>ec_cmd.command[chid].qei_max)
	//#else
	//ec_debug[chid]=(val>>2);
	if ((val&(unsigned int)0x0003)!=1 || (val>>2)>16350)
	//#endif
	{		
		if (ec_cmd.command[chid].config&M3ACT_CONFIG_VERTX_FILTER_OFF)
		{
			vertx_val[ch]=(val>>2);
			return;
		}
		else
		{
			vertx_error_buffer[ch]=vertx_error_buffer[ch]+1;
			
			return;
		}
	}
	vertx_val[ch]=(val>>2);
	return;
}


void setup_vertx(void)
{
	int i;
	BB_SPI_SS_SET
	for (i=0;i<NUM_VERTX_CH;i++)
	{
		vertx_val[i]=0;
		vertx_error_buffer[i]=0;
	}
}

unsigned int vertx_pos(int chid)
{
	return vertx_val[chid];
}

void step_vertx()
{
//Handle both channel in parallel
	unsigned int x,y;

	BB_SPI_SET_MOSI
	BB_SPI_SS_CLR
	BB_SPI_CLK_CLR
	DELAY_2300NS // WAIT 2.3us min between the SPI_SEL = lo and clock
	
	//Send 0xAA =b10101010. Takes ~20us
	BB_SPI_BIT_OUT(1,1)
	BB_SPI_BIT_OUT(0,0)
	BB_SPI_BIT_OUT(1,1)
	BB_SPI_BIT_OUT(0,0)
	BB_SPI_BIT_OUT(1,1)
	BB_SPI_BIT_OUT(0,0)
	BB_SPI_BIT_OUT(1,1)
	BB_SPI_BIT_OUT(0,0)

	us_delay(13); //Can be 12.5us-1.2=11.3us in theory

	//Send 0xFF =b11111111. Takes ~20us
	BB_SPI_BIT_OUT(1,1)
	BB_SPI_BIT_OUT(1,1)
	BB_SPI_BIT_OUT(1,1)
	BB_SPI_BIT_OUT(1,1)
	BB_SPI_BIT_OUT(1,1)
	BB_SPI_BIT_OUT(1,1)
	BB_SPI_BIT_OUT(1,1)
	BB_SPI_BIT_OUT(1,1)

	us_delay(8); //Can be 15-1.2us-13.8us in theory
	BB_SPI_SET_MISO
	us_delay(8);

	//Read in MSB Takes ~20us
	x=0;
	y=0;
	BB_SPI_BIT_IN(x,y);
	x=x<<1;
	y=y<<1;
	BB_SPI_BIT_IN(x,y);
	x=x<<1;
	y=y<<1;
	BB_SPI_BIT_IN(x,y);
	x=x<<1;
	y=y<<1;
	BB_SPI_BIT_IN(x,y);
	x=x<<1;
	y=y<<1;
	BB_SPI_BIT_IN(x,y);
	x=x<<1;
	y=y<<1;
	BB_SPI_BIT_IN(x,y);
	x=x<<1;
	y=y<<1;
	BB_SPI_BIT_IN(x,y);
	x=x<<1;
	y=y<<1;
	BB_SPI_BIT_IN(x,y);
	x=x<<1;
	y=y<<1;
	
	us_delay(13); //Can be 12.5us-1.2=11.3us in theory

	//Read in LSB .Takes ~20us
	BB_SPI_BIT_IN(x,y);
	x=x<<1;
	y=y<<1;
	BB_SPI_BIT_IN(x,y);
	x=x<<1;
	y=y<<1;
	BB_SPI_BIT_IN(x,y);
	x=x<<1;
	y=y<<1;
	BB_SPI_BIT_IN(x,y);
	x=x<<1;
	y=y<<1;
	BB_SPI_BIT_IN(x,y);
	x=x<<1;
	y=y<<1;
	BB_SPI_BIT_IN(x,y);
	x=x<<1;
	y=y<<1;
	BB_SPI_BIT_IN(x,y);
	x=x<<1;
	y=y<<1;
	BB_SPI_BIT_IN(x,y);

	DELAY_2300NS // WAIT 2.3us min between the SPI_CLK = lo and SS
	BB_SPI_SS_SET;
	DELAY_1000NS //Wait for bus to go high impedance
	BB_SPI_CLK_CLR;
	BB_SPI_SET_MOSI;
	//Total time: 2.3+20+13+20+15+20+13+20+2.3+1 ~=125us
	//Must wait 300us before try again. So max update rate is ~2.3KHz

	#if defined VERTX_BOTH
	vertx_error_filter(VERTX_CH_A,x); //Call before set vertx_val
	vertx_error_filter(VERTX_CH_B,y);//Call before set vertx_val
	vertx_buffer[VERTX_CH_A][vertx_idx]=vertx_val[VERTX_CH_A];
	vertx_buffer[VERTX_CH_B][vertx_idx]=vertx_val[VERTX_CH_B];
	#endif
	#if defined VERTX_CH_A_ONLY
	vertx_error_filter(VERTX_CH_A,x); //Call before set vertx_val
	vertx_buffer[VERTX_CH_A][vertx_idx]=vertx_val[VERTX_CH_A];
	#endif
	#if defined VERTX_CH_B_ONLY
	vertx_error_filter(VERTX_CH_B,x); //Call before set vertx_val
	vertx_buffer[VERTX_CH_B][vertx_idx]=vertx_val[VERTX_CH_B];
	#endif
	vertx_idx=INC_MOD(vertx_idx,VERTX_NUM_SMOOTH);
}

#endif
