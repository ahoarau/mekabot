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

#ifdef USE_TACTILE_PPS

#include "p33Fxxxx.h"
#include "setup.h"
#include "tactile_pps.h"


unsigned int taxels[NUM_PPS_CH][NUM_PPS_TAXEL];


//Called at 35Hz. Do bit-bang SPI here.

unsigned int get_taxel(int chid, int tid){return taxels[chid][tid];}


void step_pps(int chid)
{
	
	unsigned int x,y,j;

	//Initialisation of Clock and Chip Select
	PPS_SPI_CLK=1; 
	PPS_SPI_SS0=1;
	PPS_SPI_SS1=1;

	//Select the PPS sensor
	if (chid>=NUM_PPS_CH)
		return; //M3_GMB_G1R1 only supports ch0,1
	//ToggleHeartbeatLED();
	if (chid==0)
	{
		PPS_SPI_SS0=0;
	}
	if (chid==1)
	{
		PPS_SPI_SS1=0;
	}

	PPS_SPI_CLK=0;
	us_delay(4); // WAIT 4us min between the CS = lo and clock
	
	//Send READDATA = 0x14 =b00010100
	//PPS_SPI_BIT_OUT(0)
	//PPS_SPI_BIT_OUT(0)
	//PPS_SPI_BIT_OUT(0)
	//PPS_SPI_BIT_OUT(1)
	//PPS_SPI_BIT_OUT(0)
	//PPS_SPI_BIT_OUT(1)
	//PPS_SPI_BIT_OUT(0)
	//PPS_SPI_BIT_OUT(0)

	

	PPS_SPI_BIT_OUT(0)
	PPS_SPI_BIT_OUT(0)
	PPS_SPI_BIT_OUT(1)
	PPS_SPI_BIT_OUT(0)
	PPS_SPI_BIT_OUT(1)
	PPS_SPI_BIT_OUT(0)
	PPS_SPI_BIT_OUT(0)
	PPS_SPI_BIT_OUT(0)

	//Send SCANDATA = 0x12 =b00010010
	/*PPS_SPI_BIT_OUT(0)
	PPS_SPI_BIT_OUT(0)
	PPS_SPI_BIT_OUT(0)
	PPS_SPI_BIT_OUT(1)
	PPS_SPI_BIT_OUT(0)
	PPS_SPI_BIT_OUT(0)
	PPS_SPI_BIT_OUT(1)
	PPS_SPI_BIT_OUT(0)*/

	/*PPS_SPI_BIT_OUT(0)
	PPS_SPI_BIT_OUT(1)
	PPS_SPI_BIT_OUT(0)
	PPS_SPI_BIT_OUT(0)
	PPS_SPI_BIT_OUT(1)
	PPS_SPI_BIT_OUT(0)
	PPS_SPI_BIT_OUT(0)
	PPS_SPI_BIT_OUT(0)*/

	us_delay(36); //Wait 36us min between Command and first data clk pulse

	for(j=0;j<NUM_PPS_TAXEL;j++)
		{
		//Read in MSB
		x=0;
		y=0;
		PPS_SPI_BIT_IN(x);
		y=y|x<<7;
		PPS_SPI_BIT_IN(x);
		y=y|x<<6;
		PPS_SPI_BIT_IN(x);
		y=y|x<<5;
		PPS_SPI_BIT_IN(x);
		y=y|x<<4;
		PPS_SPI_BIT_IN(x);
		y=y|x<<3;
		PPS_SPI_BIT_IN(x);
		y=y|x<<2;
		PPS_SPI_BIT_IN(x);
		y=y|x<<1;
		PPS_SPI_BIT_IN(x);
		y=y|x;
	
		us_delay(4); //WAIT 4us min between MSB and LSB reception
		
		//y=0;
		//Read in LSB .Takes ~20us
		PPS_SPI_BIT_IN(x);
		y=y|x<<15;
		PPS_SPI_BIT_IN(x);
		y=y|x<<14;
		PPS_SPI_BIT_IN(x);
		y=y|x<<13;
		PPS_SPI_BIT_IN(x);
		y=y|x<<12;
		PPS_SPI_BIT_IN(x);
		y=y|x<<11;
		PPS_SPI_BIT_IN(x);
		y=y|x<<10;
		PPS_SPI_BIT_IN(x);
		y=y|x<<9;
		PPS_SPI_BIT_IN(x);
		y=y|x<<8;
		us_delay(4); //WAIT 4us min between 2 Data reception

		taxels[chid][j]=y;
	}
	//Unselect the PPS sensor and initialisation of CLK
	PPS_SPI_SS0=1;
	PPS_SPI_SS1=1;
	PPS_SPI_CLK=1;

}

void setup_pps(void) {
	int i,j;
	for (i=0;i<NUM_PPS_CH;i++)
		for(j=0;j<NUM_PPS_TAXEL;j++)
			taxels[i][j]=0;
	PPS_SPI_SS0=1;
	PPS_SPI_SS1=1;
	PPS_SPI_CLK=1;
}

#endif
