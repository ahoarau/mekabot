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
#ifdef USE_ADC_SPI

#include "setup.h"
#include "adc_spi.h"

unsigned int adc_spi_val[ADC_SPI_NUM_CH];
int adc_spi_idx;
unsigned int volatile adc_spi_buffer[ADC_SPI_NUM_CH][ADC_SPI_NUM_SMOOTH];


unsigned int get_avg_adc_spi(int ch)
{
	long v;
	int i;
	v=0;
	for (i=0;i<ADC_SPI_NUM_SMOOTH;i++)
		v=v+adc_spi_buffer[ch][i];
	return (unsigned int)(v>>ADC_SPI_SHIFT_SMOOTH);
}


void setup_adc_spi(void)
{
	adc_spi_idx=0;
	BB_ASPI_SS_SET	//Initialization of CS. CS should go back to 1 after each "step_adc_spi()"
	
	int i,j;
	for (i=0;i<ADC_SPI_NUM_CH;i++)
	{
		adc_spi_val[i]=0;
		for (j=0;j<ADC_SPI_NUM_SMOOTH;j++)
			adc_spi_buffer[i][j]=0;
		//adc_spi_error_buffer[i]=0;
	}
}

unsigned int get_adc_spi(int chid)
{
	return adc_spi_val[chid];
}


void step_adc_spi()
{
	//Handle bit-bang SPI
	unsigned int x;

	BB_ASPI_SS_CLR
	BB_ASPI_CLK_CLR

	DELAY_100NS//tscus

	//Read in Takes ~???

	BB_ASPI_BIT_IN(x); //First CLK pulse returns a HI-Z value
	BB_ASPI_BIT_IN(x); //Second CLK pulse returns a NULL BIT

	x=0; //  Initialize x before getting sensor Data

	BB_ASPI_BIT_IN(x); //bit #11 (MSB)
	x=x<<1;
	BB_ASPI_BIT_IN(x); 
	x=x<<1;
	BB_ASPI_BIT_IN(x);
	x=x<<1;
	BB_ASPI_BIT_IN(x);
	x=x<<1;
	BB_ASPI_BIT_IN(x);
	x=x<<1;
	BB_ASPI_BIT_IN(x);
	x=x<<1;
	BB_ASPI_BIT_IN(x);
	x=x<<1;
	BB_ASPI_BIT_IN(x);
	x=x<<1;
	BB_ASPI_BIT_IN(x);
	x=x<<1;
	BB_ASPI_BIT_IN(x);
	x=x<<1;
	BB_ASPI_BIT_IN(x);
	x=x<<1;
	BB_ASPI_BIT_IN(x);

	BB_ASPI_CLK_SET	//Not sure if needed, might be removed once the protocol works to test
	DELAY_700NS		//Not sure if needed, might be removed once the protocol works to test
	BB_ASPI_CLK_CLR	//Not sure if needed, might be removed once the protocol works to test

	BB_ASPI_SS_SET
	DELAY_700NS		//tcsh (could be 625ns)

	
	adc_spi_val[0]=x; //
	adc_spi_buffer[0][adc_spi_idx]=x;//vertx_val[0];
	adc_spi_idx=INC_MOD(adc_spi_idx,ADC_SPI_NUM_SMOOTH);

}
#endif
