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

#ifdef USE_DIO

#include "setup.h"
#include "dio.h"


#if defined PWR_0_3 || defined PWR_0_4 || defined PWR_0_5


#define MOTOR_ENABLED_BUF_SZ 20
int motor_enabled_buf[MOTOR_ENABLED_BUF_SZ];
int midx;
int GetMotorEnabledFiltered()
{
  int i,cnt=0;
  motor_enabled_buf[midx]=PinMotorEnabled;
  for (i=0;i<MOTOR_ENABLED_BUF_SZ;i++)
		cnt+=motor_enabled_buf[i];
  midx=INC_MOD(midx,MOTOR_ENABLED_BUF_SZ);
  if (cnt>=MOTOR_ENABLED_BUF_SZ/2)
   	return 1;
  else
    return 0;
}

inline void TogglePwrStatusLED(void)
{
	if (PinPwrStatusLED==1)
		ClrPwrStatusLED;
	else
		SetPwrStatusLED;
}
#endif





void setup_dio(void) 
{
#if defined PWR_0_2 || defined PWR_0_3 || defined PWR_0_4 || defined PWR_0_5
	ClrEnableMotor;
#if defined PWR_0_3 || defined PWR_0_4 || defined PWR_0_5
	midx=0;
#endif

#if defined PWR_0_5 && defined USE_BUZZER
#define BUZZER_CLR;
#endif

#endif
}


inline void ToggleHeartbeatLED(void)
{
	if (PinHeartbeatLED==1)
		ClrHeartbeatLED;
	else
		SetHeartbeatLED;
}
#endif
