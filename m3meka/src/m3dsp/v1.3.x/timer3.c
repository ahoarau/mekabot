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

#ifdef USE_TIMER3

#include "timer3.h"
#include "setup.h"

int tmp_cnt;
int irq_cnt;

#ifdef USE_TACTILE_PPS
int pps_trigger;
#endif

void setup_timer3(void)
{
	// ToDo: Setup timer3 for Ethercat distributed clock jitter calculation
	// The ECAT_TIMER (16 bit timer) runs from 0 to 0xFFFF round and round, 
	// the ECAT_CAPTURE_REG captures the ECAT_TIMER_REG when the ESC interrupt goes active
	irq_cnt=0;
	T3CON = 0;
	TMR3 = 0x0000;
	T3CONbits.TCKPS=T3_TCKPS;					
	PR3 = (unsigned int)T3_PR3;					
	_T3IF = 0;									//Clear interrupt
	T3CONbits.TON = 1;							//Start Timer 3
	_T3IE = 1;									//Enable T3 ints
	return;
}

//Every X us
void __attribute__((__interrupt__, no_auto_psv)) _T3Interrupt(void) {
	_T3IF = 0;


//Latch encoder timestamp on Rising edge.
#ifdef USE_TIMESTAMP_DC
	SetTimestampLatch;
	ClrTimestampLatch;
#endif

//ToggleHeartbeatLED();
#if defined USE_ENCODER_VERTX
#if defined M3_MAX2 || defined M3_BMW_A2R1 || defined M3_BMW_A2R2  || defined M3_HEX4_S2R1  || defined M3_BMW_A2R3
	step_vertx();
#endif
#endif

#ifdef USE_ADC_SPI
	step_adc_spi();
#endif

#if defined USE_ENCODER_QEI
#if defined M3_ELMO_Z1R1  || defined M3_ELMO_B1R1 || defined M3_MAX2_BDC_ARMH || defined  M3_MAX2_BDC_ARMH2 || defined M3_MAX2_BLDC_A2R3_QEI
	step_qei();
#endif
#endif

#ifdef USE_UART
#ifdef M3_FB_DEV_0_0
	//Insert UART protocol here ! /////////////////////////////////

#endif
#endif

#if defined USE_LED_RGB
	step_led_rgb();
#endif

#if defined USE_LED_MDRV
	//if (load_led_mdrv())
	//	step_led_mdrv();
	//load_led_mdrv();
	load_led_mdrv();
	irq_cnt++;
	if (irq_cnt==LEDMDRV_IRQ_PER_TRIGGER)
	{
		irq_cnt=0;
		step_led_mdrv();
	}
	//ToggleHeartbeatLED();
#endif

#ifdef USE_AS5510
	step_as5510();
#endif

#if defined M3_WMA_0_1 || defined M3_BMA_A1R1 || defined M3_DAC_0_1 || defined M3_ELMO_RNA_R0 || defined M3_HMB_H1R1 \
	||defined M3_HB2_H2R1_J0J1 || defined M3_HB2_H2R1_J2J3J4  ||defined M3_HB2_H2R2_J0J1 || \
	defined M3_HB2_H2R2_J2J3J4 || defined M3_GMB_G1R1 || defined M3_PWR_0_2 || defined M3_PWR_0_3 || \
	defined M3_PWR_0_4 || defined M3_DEV || defined M3_MAX2 || defined M3_HEX2_S1R1 || defined M3_BMW_A2R1 || \
	defined M3_BMW_A2R2 || defined M3_ELMO_B1R1 || defined M3_PWR_0_5 || defined M3_ELMO_Z1R1  || defined M3_HEX4_S2R1 \
	 || defined M3_BMW_A2R3 || defined M3_HB2_H2R3_J0J1 || defined M3_HB2_H2R3_J2J3J4
#ifdef USE_CONTROL	
	step_control();
#endif
	irq_cnt++;
#ifdef USE_TACTILE_PPS
	if (irq_cnt==PPS_IRQ_PER_TRIGGER)
	{
		irq_cnt=0;
		pps_trigger=1;
	}
#endif
#endif
}

#endif
