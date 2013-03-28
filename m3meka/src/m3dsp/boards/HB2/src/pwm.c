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

#ifdef USE_PWM

#include "pwm.h"
#include "setup.h"


int pwm_cmd_buf[NUM_PWM_CH];
int pwm_duty_buf[NUM_PWM_CH];
int pwm_cmd(int chid){return pwm_cmd_buf[chid];}

#if defined M3_HMB_H1R1 || defined M3_HEX2_S1R1 || defined HB2_H2R2_J2J3J4 || defined HB2_H2R2_J0J1 || \
	defined HB2_H2R1_J0J1 || defined HB2_H2R1_J2J3J4 || defined HB2_0_2_H2R3_J0J1 || defined HB2_0_2_H2R3_J2J3J4
int pwm_enc_limit(int chid, int val)
{
#ifdef USE_ENCODER_MA3 
	int pwm_fwd=ec_cmd.command[chid].config&M3ACT_CONFIG_PWM_FWD_SIGN;
	if (ma3_at_lower_bound(chid))
		if ((val<0 && pwm_fwd) ||(val>0&&!pwm_fwd))
			val=0;
	if (ma3_at_upper_bound(chid))
		if ((val>0 && pwm_fwd) ||(val<0&&!pwm_fwd))
			val=0;
#endif
	return val;
}
#endif
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
//int tmp_val;//***********************************************************************************************************************************
void set_pwm(int chid, int val)
{
//	if(chid==0) tmp_val=tmp_val*-1;//***********************************************************************************************************************************
//	val=tmp_val;//***********************************************************************************************************************************
	
	pwm_cmd_buf[chid]=SIGN(val)*CLAMP(ABS(val),0,ec_cmd.command[chid].pwm_max); //Send back commanded value before gets inverted, deadband, etc
	
#if defined HB2_H2R1_J0J1 || defined HB2_H2R1_J2J3J4 
	//Safety limits
	/*int pwm_fwd=ec_cmd.command[chid].config&M3ACT_CONFIG_PWM_FWD_SIGN;
	//ec_debug[chid]=ma3_at_lower_bound(chid);
	if (ma3_at_lower_bound(chid))
		if ((val<0 && pwm_fwd) ||(val>0&&!pwm_fwd))
			val=0;
	if (ma3_at_upper_bound(chid))
		if ((val>0 && pwm_fwd) ||(val<0&&!pwm_fwd))
			val=0;*/
	int sign=SIGN(val);
	val=pwm_enc_limit(chid,val);
	val=pwm_deadband(chid, ABS(val));
	//Invert signal
	//pwm_cmd_buf[chid]=val;
	
	/*if (val<0)
		val-=ec_cmd.command[chid].pwm_db;
	if (val>0)
		val+= ec_cmd.command[chid].pwm_db;

	int pm=MIN(PWM_MAX_DUTY,ABS(ec_cmd.command[chid].pwm_max));*/
	pwm_duty_buf[chid]=invert_and_clamp_pwm(chid,val);
	//CLAMP(PWM_MAX_DUTY-ABS(val), PWM_MAX_DUTY-pm, PWM_MAX_DUTY);
	//pwm_duty_buf[chid]=CLAMP(ABS(val), PWM_MIN_DUTY,pm);
	pwm_cmd_buf[chid]=sign*CLAMP(val,0,ec_cmd.command[chid].pwm_max); 
#if defined HB2_H2R1_J0J1 
	if (chid==0)
	{
		if (sign<=0) SetPwmDir2; else ClrPwmDir2;
		//if (val==0) ClrEnableAmpA; else SetEnableAmpA;
		P1DC2 = pwm_duty_buf[chid];
	}
	if (chid==1)
	{
		if (sign<=0) SetPwmDir1; else ClrPwmDir1;
		//if (val==0) ClrEnableAmpB; else SetEnableAmpB;
		P1DC1 = pwm_duty_buf[chid];
	}
#endif
#if defined HB2_H2R1_J2J3J4 
	if (chid==0)
	{
		if (sign<=0) SetPwmDir1; else ClrPwmDir1;
		//if (val==0) ClrEnableAmpA; else SetEnableAmpA;
		P1DC1 = pwm_duty_buf[chid];
	}
	if (chid==1)
	{
		if (sign<=0) SetPwmDir2; else ClrPwmDir2;
		//if (val==0) ClrEnableAmpB; else SetEnableAmpB;
		P1DC2 = pwm_duty_buf[chid];
	}
	if (chid==2)
	{
		if (sign<=0) SetPwmDir3; else ClrPwmDir3;
		//if (val==0) ClrEnableAmpC; else SetEnableAmpC;
		P1DC3 = pwm_duty_buf[chid];
	}
#endif
#endif

#if defined HB2_H2R2_J0J1  || defined HB2_0_2_H2R3_J0J1
	//J0: L6235 Brushless Amp ; J1: L6205 Brushed Amplifier
	int sign=SIGN(val);
	val= pwm_enc_limit(chid, val);
	val=pwm_deadband(chid,ABS(val));
	if (chid==0)
	{
		//Invert signal
		pwm_duty_buf[chid]=clamp_pwm(chid,val);
		if (sign<=0) SetPwmDirJ0; else ClrPwmDirJ0;
		P1DC2 = pwm_duty_buf[chid];
	}
	if (chid==1)
	{
		if (sign<0) //Invert polarity for this direction
			pwm_duty_buf[chid]=invert_and_clamp_pwm(chid,val);
		else
			pwm_duty_buf[chid]=clamp_pwm(chid,val);
			if (sign<=0) SetPwmDirJ1; else ClrPwmDirJ1;
			P1DC1 = pwm_duty_buf[chid];
	}
	pwm_cmd_buf[chid]=sign*CLAMP(val,0,ec_cmd.command[chid].pwm_max);
#endif

#if defined HB2_H2R2_J2J3J4  || defined HB2_0_2_H2R3_J2J3J4
	//3CH L6205 Amplifier
	int sign=SIGN(val);
	val= pwm_enc_limit(chid, val);
	val=pwm_deadband(chid,ABS(val));
	if (sign<0) //Invert polarity for this direction
		pwm_duty_buf[chid]=invert_and_clamp_pwm(chid,val);
	else
		pwm_duty_buf[chid]=clamp_pwm(chid,val);
	if (chid==0)
	{
		if (sign<=0) SetPwmDirJ2; else ClrPwmDirJ2;
		P1DC1 = pwm_duty_buf[chid];
	}
	if (chid==1)
	{
		if (sign<=0) SetPwmDirJ3; else ClrPwmDirJ3;
		P1DC2 = pwm_duty_buf[chid];
	}
	if (chid==2)
	{
		if (sign<=0) SetPwmDirJ4; else ClrPwmDirJ4;
		P1DC3 = pwm_duty_buf[chid];
	}
	pwm_cmd_buf[chid]=sign*CLAMP(val,0,ec_cmd.command[chid].pwm_max);
#endif
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

#if defined HB2_H2R1_J0J1 || defined HB2_H2R2_J0J1 || defined HB2_0_2_H2R3_J0J1 
	
	PWMCON1bits.PMOD2=1;	//PWM1H2,PWM1L2 is independent pair
	PWMCON1bits.PEN2H = 1;	//Enable PWM1H2 (RP12)
	PWMCON1bits.PMOD1=1;	//PWM1H1,PWM1L1 is independent pair
	PWMCON1bits.PEN1H = 1;	//Enable PWM1H1 (RP14)

	PDC1 = 0; // default PWM period
	PDC2 = 0; // default PWM period
	//Synchronize ADC to PWM
	PWMCON2 = 0;
	PWMCON2bits.SEVOPS = 0;						// Special event 1:1 post scale
	P1SECMPbits.SEVTCMP = PWM_ADC_SYNC_TICKS;	//ADC trigger - specifiy the phase realtive to PWM
#endif


#if defined HB2_H2R1_J2J3J4 || defined HB2_H2R2_J2J3J4 || defined HB2_0_2_H2R3_J2J3J4
	
	PWMCON1bits.PMOD1=1;	//PWM1H1,PWM1L1 is independent pair
	PWMCON1bits.PEN1H = 1;	//Enable PWM1H1
	
	PWMCON1bits.PMOD2=1;	//PWM1H2,PWM1L2 is independent pair
	PWMCON1bits.PEN2H = 1;	//Enable PWM1H2 

	PWMCON1bits.PMOD3=1;	//PWM1H3,PWM1L3 is independent pair
	PWMCON1bits.PEN3H = 1;	//Enable PWM1H3 

	PDC1 = 0; // default PWM period
	PDC2 = 0; // default PWM period
	PDC3 = 0; // default PWM period
	//Synchronize ADC to PWM
	PWMCON2 = 0;
	PWMCON2bits.SEVOPS = 0;						// Special event 1:1 post scale
	P1SECMPbits.SEVTCMP = PWM_ADC_SYNC_TICKS;	//ADC trigger - specifiy the phase realtive to PWM
#endif
	//Disable faults
	FLTACON = 0;
	_PWM1IF=0;
	PTCONbits.PTEN = 1;  // PWM enable
}
#endif 
