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

#ifdef USE_TIMER1

#include "timer1.h"
#include "setup.h"

/*
For current sensing the ADC needs to sync to the start of the PWM cycle
Because we are using >1 ADC channel, we can't just use the SEVTCMP functionality
Instead, we initialize Timer1 to trigger after X us from the PWM rising edge
and then every PWM_TIMEBASE_CYC/4 us after that
On every Timer1 interrupt we then manually trigger the sample/hold of the next
ADC channel in the queue (of the NCH)
Timer1 is turned on in SetupPwm at the same time as the PWM counter, so they are synced.
*/

int t1_irq_idx;

void setup_timer1(void)
{
	T1CON = 0;
	TMR1 = 0x0000;
	T1CONbits.TCKPS=0; // 1:1 Prescaler, same as PWM
	PR1 = PWM_TIMEBASE_CYC; //PR1 = 40Mhz/1/25Khz=1600-1
	_T1IF = 0;
	_T1IE = 1;
	t1_irq_idx=0;
}


void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt(void) 
{
	_T1IF = 0;
	if (t1_irq_idx==0) //Set current sense conversion point
	{
		Nop();
		//ToDo: get ride of this whole function?
	}
	else
	{
		TMR1=PWM_TIMEBASE_CYC-(PWM_TIMEBASE_CYC>>3);
	}
	t1_irq_idx++;
}

#endif


