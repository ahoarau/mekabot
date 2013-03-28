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

#include "p33Fxxxx.h"
#include "setup.h"
#include "bootloader.h"
#include "dio.h"

_FWDT(FWDTEN_OFF);
//Note: the firmware loaded by the bootloader inherits these settings.
//, so need a bootloader version for each set of settings
//#if defined M3_BLD_INVPWM
//_FPOR(FPWRT_PWR1 & HPOL_OFF);/* Turn off the power-up timers. Invert PWM polarity*
//#else
_FPOR(FPWRT_PWR1);					/* Turn off the power-up timers. */
//#endif
_FOSCSEL(FNOSC_PRIPLL & IESO_ON);	// Auto switch to the EC+PLL clock
//Clock switching+monitor disabled, OSC2 is clock O/P, External clock, allow lock/unlock of peripheral-pin-select
_FOSC(FCKSM_CSDCMD & OSCIOFNC_OFF  & POSCMD_EC & IOL1WAY_OFF); 
_FGS(GCP_OFF);            							// Disable Code Protection

int main(void)
{
	long cntr=0;
	//Setup oscillator/ports/pins first
	setup_oscillator();
	setup_ports();
	setup_peripheral_pin_select();
	setup_interrupt_priorities();
	while (!eeprom_loaded());		//Wait until ESC is ready			
	setup_bootloader();
	while(1){
		step_bootloader();
		if (cntr++==150000)
		{
			cntr=0;
			ToggleHeartbeatLED();
		}
	}
}

