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


#include "pwm.h"
#include "setup.h"

int pwm_cmd_buf[NUM_PWM_CH];
int pwm_duty_buf[NUM_PWM_CH];
int pwm_cmd(int chid){return pwm_cmd_buf[chid];}

int pwm_deadband(int chid, int abs_val)
{
	if (abs_val==0)
		return 0;
	if (ec_cmd.command[chid].pwm_db<0)
	{
		if (abs_val<ABS(ec_cmd.command[chid].pwm_db))
			abs_val=0;
	}
	else
		abs_val=abs_val+ec_cmd.command[chid].pwm_db;
	return abs_val;
}

//In: 0<=PWM_MIN_DUTY<=val<=ec_cmd.command[chid].pwm_max<=PWM_MAX_DUTY
//Out: Inverted clamped version
int invert_and_clamp_pwm(int chid, int abs_val)
{
	int pm=MAX(PWM_MIN_DUTY,MIN(PWM_MAX_DUTY,ec_cmd.command[chid].pwm_max));
	return CLAMP(PWM_MAX_DUTY-abs_val,PWM_MAX_DUTY-pm, PWM_MAX_DUTY-PWM_MIN_DUTY);
}

int clamp_pwm(int chid, int abs_val)
{
	int pm=MAX(PWM_MIN_DUTY,MIN(PWM_MAX_DUTY,ec_cmd.command[chid].pwm_max));
	return CLAMP(abs_val,PWM_MIN_DUTY,pm);
}


void set_pwm(int chid, int val)
{

	pwm_cmd_buf[chid]=SIGN(val)*CLAMP(ABS(val),0,ec_cmd.command[chid].pwm_max); //Send back commanded value before gets inverted, deadband, etc


	int sign=SIGN(val);
	val=pwm_deadband(chid,ABS(val));
	pwm_duty_buf[chid]=clamp_pwm( chid, val);
	if (chid==0)
	{
		if (sign <= 0)
			SetPwmDir1;
		else
			ClrPwmDir1;
		P1DC2 =pwm_duty_buf[chid];
		//ec_debug[chid]=P1DC2;
	}
	if (chid==1)
	{
		if (sign <= 0)
			SetPwmDir2;
		else
			ClrPwmDir2;
		P1DC1 =pwm_duty_buf[chid];
		//ec_debug[chid]=P1DC2;
	}
	//Utilize the non pwm-db value, as QS mode for amp
	//requires a db of +800
	//pwm_cmd_buf[chid]=sign*pwm_duty_buf[chid];


}

void setup_pwm(void) {
	int i;
	for (i=0;i<NUM_PWM_CH;i++)
	{
		pwm_cmd_buf[i]=0;
		pwm_duty_buf[i]=0;
	}

	PTCONbits.PTEN = 0;  // PWM disable
	//Set time base
	PTCON  = 0;
	PTCONbits.PTOPS=0;			// PWM 1:1 postcale
	PTCONbits.PTCKPS = 0;		// PWM timebase input clock prescale 1:1 (Tcy)
	PTPER = PWM_TIMEBASE_CYC;	// set timebase period
	PTCONbits.PTMOD = 0;		//Free-Running Mode
	PTMR = 0;					//Reset counter
	PWMCON1 = 0;				// reset condition specified in CONFIG BITS FPOR

//	tmp_val=300;//********************************************************************************************************************************************
	PWMCON1 = 0;
	//RP13 is PWM1L2 which is PWM1 (P1DC2)
	//RP12 is PWM1H2 which is DIR1
	//RP14 is PWM1H1 which is PWM2 (P1DC1)
	//RP15 is PWM1L1 which is DIR2
	PWMCON1bits.PMOD2	= 1;	//PWM1H2,PWM1L2 is independent pair
	PWMCON1bits.PMOD1	= 1;	//PWM1H1,PWM1L1 is independent pair
	PWMCON1bits.PEN2L	= 1;	//Enable PWM1L2 (RP13)
	PWMCON1bits.PEN1H	= 1;	//Enable PWM1H1 (RP14)
	PDC2 = 0; // default PWM period
	PDC1 = 0; // default PWM period
	//Synchronize ADC to PWM
	PWMCON2 = 0;
	PWMCON2bits.SEVOPS = 0;						// Special event 1:1 post scale
	P1SECMPbits.SEVTCMP = PWM_ADC_SYNC_TICKS;	//ADC trigger - specifiy the phase realtive to PWM


	//Disable faults
	FLTACON = 0;
	_PWM1IF = 0;
	PTCONbits.PTEN = 1;  // PWM enable
}
