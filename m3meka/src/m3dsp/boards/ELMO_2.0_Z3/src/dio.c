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

void setup_dio(void) 
{

    ClrEnableAmp;


}

inline void ToggleHeartbeatLED(void)
{
	/* Legacy:
	if (PinHeartbeatLED==1)
		ClrHeartbeatLED;
	else
		SetHeartbeatLED;
	*/
	PinHeartbeatLED ^= 1;
}

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
