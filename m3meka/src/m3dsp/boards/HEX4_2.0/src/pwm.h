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

#ifndef __PWM_H__
#define __PWM_H__ 


//TPWM=TCY(PTPER+1)*(PTMR Prescale)=(1/40M)*(1599+1)*1=40us
//Resolution=log(2*TPWM/TCY)/log(2)=log(2*400)/log(2)=9.6 bits

//#define PWM_TIMEBASE_CYC  399  // TPWM=TCY(PTPER+1)*PTMR PRESCALE = (1/40MHZ)*400 = 1/100KHZ 
//#define PWM_TIMEBASE_CYC  799  // TPWM=TCY(PTPER+1)*PTMR PRESCALE = (1/40MHZ)*800 = 1/50KHZ 
//#define PWM_TIMEBASE_CYC  1599  // TPWM=TCY(PTPER+1)*PTMR PRESCALE =(1/40MHZ)*1600 = 1/25KHZ 
//#define PWM_TIMEBASE_CYC  1999  // TPWM=TCY(PTPER+1)*PTMR PRESCALE = (1/40MHZ)*2000 = 1/20KHZ
//#define PWM_TIMEBASE_CYC  2665  // TPWM=TCY(PTPER+1)*PTMR PRESCALE = (1/40MHZ)*2666 = 1/15KHZ

void setup_pwm();
void set_pwm(int chid, int val);
int pwm_cmd(int chid);

#define PWM_TIMEBASE_CYC		1599
#define PWM_HALF_PERIOD_CYC		PWM_TIMEBASE_CYC
#define PWM_FULL_PERIOD_CYC		2*PWM_TIMEBASE_CYC
#define PWM_MIN_DUTY			0	//BMW has 100NS min pulse width. Each tick is 25ns, so min of 4
#define PWM_MAX_DUTY			PWM_FULL_PERIOD_CYC-2*PWM_MIN_DUTY	//Never turn on 100%
#define PWM_ADC_SYNC_TICKS		250		//Trigger ADC conversion this many ticks after start of PWM period
#define NUM_PWM_CH				2

#define SetPwmDir1				LATBbits.LATB12 = 1
#define ClrPwmDir1				LATBbits.LATB12 = 0
#define PinPwmDir1				PORTBbits.RB12

#define SetPwmDir2				LATBbits.LATB15 = 1
#define ClrPwmDir2				LATBbits.LATB15 = 0
#define PinPwmDir2				PORTBbits.RB15

#endif

