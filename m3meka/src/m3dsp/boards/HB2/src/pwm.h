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

#ifdef USE_PWM

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

#if defined HB2_H2R1_J0J1

#define NUM_PWM_CH 2

#define SetEnableAmpA LATBbits.LATB9=1
#define ClrEnableAmpA LATBbits.LATB9=0
#define SetEnableAmpB LATBbits.LATB4=1
#define ClrEnableAmpB LATBbits.LATB4=0

#define ClrPwmDir1	LATBbits.LATB15=0
#define SetPwmDir1	LATBbits.LATB15=1
#define ClrPwmDir2	LATBbits.LATB13=0
#define SetPwmDir2	LATBbits.LATB13=1
#define PinPwm1H1	PORTBbits.RB14  //Pwm1
#define PinPwm1L1	PORTBbits.RB15  //Dir1
#define PinPwm1H2	PORTBbits.RB12  //Pwm0
#define PinPwm1L2	PORTBbits.RB13  //Dir0

#define PWM_TIMEBASE_CYC  1599  // TPWM=TCY(PTPER+1)*PTMR PRESCALE = (1/40MHZ)*800 = 1/50KHZ 
#define PWM_HALF_PERIOD_CYC	PWM_TIMEBASE_CYC
#define PWM_FULL_PERIOD_CYC	2*PWM_TIMEBASE_CYC
#define PWM_MIN_DUTY 	0	
#define PWM_MAX_DUTY   PWM_FULL_PERIOD_CYC	
#define PWM_ADC_SYNC_TICKS	250		//Trigger ADC conversion this many ticks after start of PWM period
#endif

#if defined HB2_H2R2_J0J1 || defined HB2_0_2_H2R3_J0J1 
#define NUM_PWM_CH 2
#define ClrPwmDirJ0	LATBbits.LATB13=0
#define SetPwmDirJ0	LATBbits.LATB13=1
#define ClrPwmDirJ1	LATBbits.LATB15=0
#define SetPwmDirJ1	LATBbits.LATB15=1
#define PinPwm1H1	PORTBbits.RB14  //Pwm P1DC1 = J1
#define PinPwm1H2	PORTBbits.RB12  //Pwm P1DC2 = J0
#define PWM_TIMEBASE_CYC  1599  // TPWM=TCY(PTPER+1)*PTMR PRESCALE = (1/40MHZ)*800 = 1/50KHZ 
#define PWM_HALF_PERIOD_CYC	PWM_TIMEBASE_CYC
#define PWM_FULL_PERIOD_CYC	2*PWM_TIMEBASE_CYC
#define PWM_MIN_DUTY 	0	
#define PWM_MAX_DUTY   PWM_FULL_PERIOD_CYC	
#define PWM_ADC_SYNC_TICKS	250		//Trigger ADC conversion this many ticks after start of PWM period
#endif


#if defined HB2_H2R1_J2J3J4 
#define NUM_PWM_CH 3

#define SetEnableAmpA LATBbits.LATB4=1
#define ClrEnableAmpA LATBbits.LATB4=0
#define SetEnableAmpB LATBbits.LATB9=1
#define ClrEnableAmpB LATBbits.LATB9=0
#define SetEnableAmpC LATCbits.LATC2=1
#define ClrEnableAmpC LATCbits.LATC2=0

#define ClrPwmDir1	LATBbits.LATB15=0
#define SetPwmDir1	LATBbits.LATB15=1
#define ClrPwmDir2	LATBbits.LATB13=0
#define SetPwmDir2	LATBbits.LATB13=1
#define ClrPwmDir3	LATBbits.LATB11=0
#define SetPwmDir3	LATBbits.LATB11=1
#define PinPwm1H1	PORTBbits.RB14  //Pwm
#define PinPwm1L1	PORTBbits.RB15  //Dir
#define PinPwm1H2	PORTBbits.RB12  //Pwm
#define PinPwm1L2	PORTBbits.RB13  //Dir
#define PinPwm1H3	PORTBbits.RB10  //Pwm
#define PinPwm1L3	PORTBbits.RB11  //Dir
#define PWM_TIMEBASE_CYC  1599  // TPWM=TCY(PTPER+1)*PTMR PRESCALE = (1/40MHZ)*800 = 1/50KHZ 
#define PWM_HALF_PERIOD_CYC	PWM_TIMEBASE_CYC
#define PWM_FULL_PERIOD_CYC	2*PWM_TIMEBASE_CYC
#define PWM_MIN_DUTY 	0	
#define PWM_MAX_DUTY   PWM_FULL_PERIOD_CYC	
#define PWM_ADC_SYNC_TICKS	250		//Trigger ADC conversion this many ticks after start of PWM period
#endif

#if defined HB2_H2R2_J2J3J4  || defined HB2_0_2_H2R3_J2J3J4
#define NUM_PWM_CH 3
#define ClrPwmDirJ4	LATBbits.LATB11=0
#define SetPwmDirJ4	LATBbits.LATB11=1
#define ClrPwmDirJ3	LATBbits.LATB13=0
#define SetPwmDirJ3	LATBbits.LATB13=1
#define ClrPwmDirJ2	LATBbits.LATB15=0
#define SetPwmDirJ2	LATBbits.LATB15=1
#define PinPwm1H1	PORTBbits.RB14  //Pwm P1DC1 = J2
#define PinPwm1H2	PORTBbits.RB12  //Pwm P1DC2 = J3
#define PinPwm1H3	PORTBbits.RB10  //Pwm P1DC3 = J4
#define PWM_TIMEBASE_CYC  1599  // TPWM=TCY(PTPER+1)*PTMR PRESCALE = (1/40MHZ)*800 = 1/50KHZ 
#define PWM_HALF_PERIOD_CYC	PWM_TIMEBASE_CYC
#define PWM_FULL_PERIOD_CYC	2*PWM_TIMEBASE_CYC
#define PWM_MIN_DUTY 	0	
#define PWM_MAX_DUTY   PWM_FULL_PERIOD_CYC	
#define PWM_ADC_SYNC_TICKS	250		//Trigger ADC conversion this many ticks after start of PWM period
#endif

#endif
#endif
