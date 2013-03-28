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


#ifndef _TIMER3_H
#define _TIMER3_H



#define T3_TCKPS					1 //8:1 scalar, so 1 tick = 200ns, 5 ticks/1us
#define T3_US_PER_IRQ				1000 //1000Hz
#define T3_PR3						T3_US_PER_IRQ*5 //Ticks to interrupt at.
#define LEDMDRV_IRQ_PER_TRIGGER		10//18 //100Hz


void setup_timer3(void);

extern int step_led_flag;


#endif
