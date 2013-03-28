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


#if defined M3_PWR_0_3 || defined M3_PWR_0_4 || defined M3_PWR_0_5


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

#if defined M3_ELMO_B1R1 || defined M3_ELMO_Z1R1 || defined M3_DAC_0_1
int limit_switch_neg_flag()
{
	if (!PinLimitSwitch) //inverted 
		return (int)M3ACT_FLAG_NEG_LIMITSWITCH;
	else
		return 0;
}
int limit_switch_pos_flag()
{
	return 0; //neg only for this board
}
#endif

//ZTB Z AXIS TRIGGER BOARD
#if defined M3_ELMO_Z1R1
int get_aux_switch()
{
	return PinZTB;
}
int aux_switch_flag()
{
	if (PinZTB) 
	{	SetHeartbeatLED;
		return (int)M3ACT_FLAG_AUX_SWITCH;
	}
	else
		return 0;
}
#endif



void setup_dio(void) 
{
#if defined M3_PWR_0_2 || defined M3_PWR_0_3 || defined M3_PWR_0_4 || defined M3_PWR_0_5
	ClrEnableMotor;
#if defined M3_PWR_0_3 || defined M3_PWR_0_4 || defined M3_PWR_0_5
	midx=0;
#endif

#if defined M3_PWR_0_5 && defined USE_BUZZER
#define BUZZER_CLR;
#endif

#endif
#if defined M3_DAC_0_1 || defined M3_ELMO_RNA_R0 || defined M3_ELMO_B1R1 || defined M3_ELMO_Z1R1
	ClrEnableAmp;
#endif
#if defined M3_ELMO_Z1R1
	AD1PCFGLbits.PCFG7=1;// Use AN7 as a digital input for M3_ZTB board (Z-Axis trigger board)
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
