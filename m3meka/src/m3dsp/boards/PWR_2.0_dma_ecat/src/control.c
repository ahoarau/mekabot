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

#ifdef USE_CONTROL

extern unsigned int watchdog_expired;

#include "setup.h"
#include "control.h"
#if defined PWR_0_3 || defined PWR_0_4 || defined PWR_0_5
#include "timer3.h"
int pwr_status_cnt;
#endif


#if defined PWR_0_2 || defined PWR_0_3 || defined PWR_0_4 || defined PWR_0_5
void setup_control() 
{
#if defined PWR_0_3 || defined PWR_0_4 || defined PWR_0_5
	pwr_status_cnt=0;
#endif 
}
void step_control()
{

	#ifdef USE_WATCHDOG	
	if (ec_cmd.enable_motor && !watchdog_expired)
		SetEnableMotor;
	else
		ClrEnableMotor;
	#else
	if (ec_cmd.enable_motor)
		SetEnableMotor;
	else
		ClrEnableMotor;
	#endif
	
#if defined PWR_0_3 || defined PWR_0_4 || defined PWR_0_5

	if (PinMotorEnabled)
		SetPwrStatusLED;
	else
		if (pwr_status_cnt++>=60)
		{
			pwr_status_cnt=0;
			TogglePwrStatusLED();
		}
#endif 

#if defined PWR_0_5 && defined USE_BUZZER
	if (ec_cmd.config&M3PWR_CONFIG_BUZZER)
			BUZZER_SET
	else
			BUZZER_CLR
#endif
}

#endif //M3_PWR
#endif //USE_CONTROL
