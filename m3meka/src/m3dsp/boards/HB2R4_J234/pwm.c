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

#if defined M3_HMB_H1R1 || defined M3_HEX2_S1R1 || defined M3_HB2_H2R2_J2J3J4 || defined M3_HB2_H2R2_J0J1 || \
	defined M3_HB2_H2R1_J0J1 || defined M3_HB2_H2R1_J2J3J4 || defined M3_HB2_H2R3_J0J1 || defined M3_HB2_H2R3_J2J3J4
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
	
#if defined M3_BMA_A1R1 
	//chid must be zero
	int pm=MIN(PWM_MAX_DUTY,ABS(ec_cmd.command[chid].pwm_max));
	if (val <= 0) 
	{
		pwm_duty_buf[0]=-val;
		SetPwmDir;
	}
	else
	{
		pwm_duty_buf[0]=val;
		ClrPwmDir;
	}
	if (pwm_duty_buf[0]!=0)
	{
		if (pwm_duty_buf[0]>pm)
			pwm_duty_buf[0]=pm;
		if (pwm_duty_buf[0]<PWM_MIN_DUTY)
			pwm_duty_buf[0]=PWM_MIN_DUTY;
	}
	P1DC2 = pwm_duty_buf[0];
#endif

#if defined M3_BMW_A2R2  || defined M3_BMW_A2R3
	int sign=SIGN(val);
	val=pwm_deadband(chid,ABS(val));
	pwm_duty_buf[chid]=clamp_pwm( chid, val);
	if (sign <= 0) 
		SetPwmDir;
	else
		ClrPwmDir;
	P1DC2 =pwm_duty_buf[chid];
	pwm_cmd_buf[chid]=sign*val;
	//ec_debug[chid]=P1DC2;
#endif

#if defined M3_HEX4_S2R1 
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

#endif

#if defined M3_BMW_A2R1 
	//Invert signal
	//chid must be zero
	int pm=MIN(PWM_MAX_DUTY,ABS(ec_cmd.command[chid].pwm_max));
	if (val <= 0) 
		SetPwmDir;
	else
		ClrPwmDir;
	if (val!=0)
		val=ABS(val)+ec_cmd.command[chid].pwm_db;
	
	pwm_duty_buf[0]=CLAMP(PWM_MAX_DUTY-val, PWM_MAX_DUTY-pm, PWM_MAX_DUTY);
	//pwm_duty_buf[0]=CLAMP(val, PWM_MIN_DUTY, PWM_MAX_DUTY);
	P1DC1 = pwm_duty_buf[0];
#endif

#if defined M3_HB2_H2R1_J0J1 || defined M3_HB2_H2R1_J2J3J4 
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
#if defined M3_HB2_H2R1_J0J1 
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
#if defined M3_HB2_H2R1_J2J3J4 
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

#if defined M3_HB2_H2R2_J0J1  || defined M3_HB2_H2R3_J0J1
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

#if defined M3_HB2_H2R2_J2J3J4  || defined M3_HB2_H2R3_J2J3J4
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

#if defined M3_HMB_H1R1 || defined M3_HEX2_S1R1 
	//Safety limits
	val= pwm_enc_limit(chid, val);
	int sign=SIGN(val);
	int pmax=MIN(PWM_MAX_DUTY,ABS(ec_cmd.command[chid].pwm_max));
	if (sign<0) //Invert polarity for this direction
	{
		pwm_duty_buf[chid]=CLAMP(PWM_MAX_DUTY+val, PWM_MAX_DUTY-pmax, PWM_MAX_DUTY);

		if (chid==0)
		{
			P1DC2 = pwm_duty_buf[chid];
			SetPwmDir2;
		}
		else
		{
			SetPwmDir1;
			P1DC1 = pwm_duty_buf[chid];
		}
	}
	else
	{
		pwm_duty_buf[chid]=CLAMP(val,PWM_MIN_DUTY,pmax);
		if (chid==0)
		{
			P1DC2 = pwm_duty_buf[chid];
			ClrPwmDir2;
		}
		else
		{
			ClrPwmDir1;
			P1DC1 = pwm_duty_buf[chid];
		}
	}		
	
#endif

#ifdef M3_GMB_G1R1
	int sign=SIGN(val);
	int pmax=MIN(PWM_MAX_DUTY,ABS(ec_cmd.command[chid].pwm_max));
	if (sign<0) //Invert polarity for this direction
	{
		pwm_duty_buf[chid]=CLAMP(PWM_MAX_DUTY+val, PWM_MAX_DUTY-pmax, PWM_MAX_DUTY);
		if (chid==0)
		{
			P1DC2 = pwm_duty_buf[chid];
			SetPwmDir2;
		}
		else
		{
			SetPwmDir1;
			P1DC1 = pwm_duty_buf[chid];
		}
	}
	else
	{
		pwm_duty_buf[chid]=CLAMP(val,PWM_MIN_DUTY,pmax);
		if (chid==0)
		{
			P1DC2 = pwm_duty_buf[chid];
			ClrPwmDir2;
		}
		else
		{
			ClrPwmDir1;
			P1DC1 = pwm_duty_buf[chid];
		}
	}	
#endif

#ifdef M3_WMA_0_1
//chid must be zero
	int pm=MIN(PWM_MAX_DUTY,ec_cmd.command[chid].pwm_max);
	if (val <= 0) 
		pwm_duty_buf[0]=-val;
	else
		pwm_duty_buf[0]=val;
	if (pwm_duty_buf[0]!=0)
	{
		if (pwm_duty_buf[0]>pm)
			pwm_duty_buf[0]=pm;
		if (pwm_duty_buf[0]<PWM_MIN_DUTY)
			pwm_duty_buf[0]=PWM_MIN_DUTY;
	}
	if (val <= 0) 
	{
			P1DC1 = pwm_duty_buf[0];
			P1DC2 = 0;
	} else 
	{
			P1DC1 = 0;
			P1DC2 = pwm_duty_buf[0];
	}
	
#endif
#ifdef M3_MAX2
//chid must be zero
	int sign=SIGN(val);
	val=pwm_deadband(chid,ABS(val));
	pwm_cmd_buf[chid]=sign*clamp_pwm(chid,val);

	pwm_cmd_buf[chid]=SIGN(val)*CLAMP(ABS(val),0,ec_cmd.command[chid].pwm_max); //Send back commanded value before gets inverted, deadband, etc
	//ec_debug[chid]=pwm_cmd_buf[chid];

#if  defined  M3_MAX2_BDC_ARMH //||M3_MAX2_BDC_T2R1
	pwm_duty_buf[chid]=val;//clamp_pwm(chid,val);
	if (sign <= 0) 
	{
			P1DC1 = pwm_duty_buf[chid];
			P1DC2 = 0;
	} else 
	{
			P1DC1 = 0;
			P1DC2 = pwm_duty_buf[chid];
	}
#endif//BDC

#if defined M3_MAX2_BDC_A2R2 || defined M3_MAX2_BDC_T2R2 || defined  M3_MAX2_BDC_S1R1 \
	 || defined M3_MAX2_BDC_T2R3 || defined M3_MAX2_BDC_A2R1 || defined M3_MAX2_BDC_T2R1 || defined M3_MAX2_BDC_A2R3
#ifdef PWM_4Q
	/*
	X X POVD3H POVD3L   POVD2H POVD2L POVD1H POVD1L   X X POUT3H POUT3L   POUT2H POUT2L POUT1H POUT1L 
	X X   Q5     Q6       Q3     Q4      Q1    Q2     X X   Q5     Q6       Q3    Q4      Q1    Q2
	POVDXX=1: PWM
	POVDXX=0: POUT
	Forward: Q1=On, Q4=PWM	 =	0000 1100 0000 0010 = 0x0C02
	Reverse: Q3=On, Q2=PWM	 =	0000 0011 0000 1000 = 0x0308
	*/
	pwm_duty_buf[chid]=invert_and_clamp_pwm(chid,val); //Invert as switching on low-leg
	if (sign >= 0) 
	{
			P1DC1 = pwm_duty_buf[0];
			//P1DC2 = 0;
			OVDCON=0x0308;
	} else 
	{
			//P1DC1 = 0;
			OVDCON=0x0C02;
			P1DC2 = pwm_duty_buf[0];
	}
#endif//4Q
#endif//BDC
//Run 
#if defined M3_MAX2_BLDC_A2R2 || defined M3_MAX2_BLDC_A2R1 || defined M3_MAX2_BLDC_T2R1 || defined M3_MAX2_BLDC_T2R2 || defined M3_MAX2_BLDC_A2R3 || defined M3_MAX2_BLDC_T2R3
#if defined PWM_4Q 
		pwm_duty_buf[chid]=invert_and_clamp_pwm(chid,val); //Invert as switching on low-leg
		P1DC1=pwm_duty_buf[chid];
		P1DC2=pwm_duty_buf[chid];
		P1DC3=pwm_duty_buf[chid];
#endif
#if defined PWM_2Q
		pwm_duty_buf[chid]=clamp_pwm(chid,val); //Invert as switching on low-leg
		P1DC1=pwm_duty_buf[chid];
		P1DC2=pwm_duty_buf[chid];
		P1DC3=pwm_duty_buf[chid];
#endif
#endif//BLDC
#ifdef USE_BLDC
		if (sign<=0) set_bldc_dir(0); else set_bldc_dir(1);
#endif

#endif//MAX2
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

#ifdef M3_MAX2
#ifdef PWM_4Q
	PWMCON1bits.PMOD1=0;	//PWM1H1,PWM1L1 is complimentary pair
	PWMCON1bits.PMOD2=0;	//PWM1H2,PWM1L2 is complimentary pair
	PWMCON1bits.PMOD3=0;	//PWM1H3,PWM1L3 is complimentary pair
	PWMCON1bits.PEN1H = 1;	//Enable PWM1H1
	PWMCON1bits.PEN1L = 1;	//Enable PWM1L1
	PWMCON1bits.PEN2H = 1;	//Enable PWM1H2
	PWMCON1bits.PEN2L = 1;	//Enable PWM1L2
#endif
#ifdef PWM_2Q
	PWMCON1bits.PMOD1=1;	//PWM1H1,PWM1L1 is independent pair
	PWMCON1bits.PMOD2=1;	//PWM1H2,PWM1L2 is independent pair
	PWMCON1bits.PMOD3=1;	//PWM1H3,PWM1L3 is independent pair
	PWMCON1bits.PEN1H = 1;	//Enable PWM1H1
	PWMCON1bits.PEN1L = 1;	//Enable PWM1L1
	PWMCON1bits.PEN2H = 1;	//Enable PWM1H2
	PWMCON1bits.PEN2L = 1;	//Enable PWM1L2
#endif

	//Synchronize ADC to PWM
	PWMCON2 = 0;			
	PWMCON2bits.SEVOPS = 0;	// Special event 1:1 post scale
	PWMCON2bits.IUE=1;		//Immediate update of duty cycle
	//PWMCON2bits.IUE=0;		//Update of duty cycle sync to PWM time base
	P1SECMPbits.SEVTCMP = PWM_ADC_SYNC_TICKS;	//ADC trigger - specifiy the phase realtive to PWM
	//Set PWM dead time
	DTCON1 = 0;					//Deadtime clock base is 1:1 TCY for Unit A, Unit B
	DTCON1bits.DTA = PWM_DEAD_CYC_A;
	DTCON1bits.DTB = PWM_DEAD_CYC_B;
	DTCON2 = 0;
	DTCON2bits.DTS1A = 0;		//Deadtime for PWM1H1/PWM1L1 going active from Unit A
	DTCON2bits.DTS1I = 1;		//Deadtime for PWM1H1/PWM1L1 going inactive from Unit B
	DTCON2bits.DTS2A = 0;		//Deadtime for PWM1H2/PWM1L2 going active from Unit A
	DTCON2bits.DTS2I = 1;		//Deadtime for PWM1H2/PWM1L2 going inactive from Unit B
	PDC1 = 0;
	PDC2 = 0;
#if defined M3_MAX2_BLDC_A2R2 || defined M3_MAX2_BLDC_A2R1 || defined M3_MAX2_BLDC_T2R1 || defined M3_MAX2_BLDC_T2R2 || defined M3_MAX2_BLDC_A2R3 || defined M3_MAX2_BLDC_T2R3
#ifdef PWM_4Q
	PWMCON1bits.PMOD3=0;	//PWM1H3,PWM1L3 is complimentary pair
	PWMCON1bits.PEN3H = 1;	//Enable PWM1H3
	PWMCON1bits.PEN3L = 1;	//Enable PWM1L3
#endif
#ifdef PWM_2Q
	PWMCON1bits.PMOD3=1;	//PWM1H3,PWM1L3 is independent pair
	PWMCON1bits.PEN3H = 1;	//Enable PWM1H3
	PWMCON1bits.PEN3L = 1;	//Enable PWM1L3
#endif
	DTCON2bits.DTS3A = 0;	//Deadtime for PWM1H3/PWM1L3 going active from Unit A
	DTCON2bits.DTS3I = 1;	//Deadtime for PWM1H3/PWM1L3 going inactive from Unit B
	PDC3 = 0;
	//OVDCON=0;				//Allow control PWM1H/L1-3 using OVD. Set in bldc.c instead...
	PWMCON2bits.OSYNC=0;	//OVDCON overrides are on next Tcy boundary
	//PWMCON2bits.OSYNC=1;	//OVDCON overrides synchronized to PWM time base
#endif
#endif

#ifdef M3_BMA_A1R1
	//RP13 is PWM1L2 which goes to amp pwm
	PWMCON1bits.PMOD2=1;	//PWM1H2,PWM1L2 is independent pair
	PWMCON1bits.PEN2L = 1;	//Enable PWM1L2 (RP13)
	PDC2 = 0; // default PWM period
	//Synchronize ADC to PWM
	PWMCON2 = 0;
	PWMCON2bits.SEVOPS = 0;						// Special event 1:1 post scale
	P1SECMPbits.SEVTCMP = PWM_ADC_SYNC_TICKS;	//ADC trigger - specifiy the phase realtive to PWM
#endif

#if defined M3_BMW_A2R2 || defined M3_BMW_A2R3
	//RP13 is PWM1L2 which goes to amp pwm
	//RP12 is DIR
	PWMCON1bits.PMOD2=1;	//PWM1H2,PWM1L2 is independent pair
	PWMCON1bits.PEN2L = 1;	//Enable PWM1L2 (RP13)
	PDC2 = 0; // default PWM period
	//Synchronize ADC to PWM
	PWMCON2 = 0;
	PWMCON2bits.SEVOPS = 0;						// Special event 1:1 post scale
	P1SECMPbits.SEVTCMP = PWM_ADC_SYNC_TICKS;	//ADC trigger - specifiy the phase realtive to PWM
#endif

#ifdef M3_HEX4_S2R1
//	tmp_val=300;//********************************************************************************************************************************************
	PWMCON1 = 0;
	//RP13 is PWM1L2 which is PWM1 (P1DC2)
	//RP12 is PWM1H2 which is DIR1
	//RP14 is PWM1H1 which is PWM2 (P1DC1)
	//RP15 is PWM1L1 which is DIR2 
	PWMCON1bits.PMOD2=1;	//PWM1H2,PWM1L2 is independent pair
	PWMCON1bits.PMOD1=1;	//PWM1H1,PWM1L1 is independent pair
	PWMCON1bits.PEN2L = 1;	//Enable PWM1L2 (RP13)
	PWMCON1bits.PEN1H = 1;	//Enable PWM1H1 (RP14)
	PDC2 = 0; // default PWM period
	PDC1 = 0; // default PWM period
	//Synchronize ADC to PWM
	PWMCON2=0;
	PWMCON2bits.SEVOPS = 0;						// Special event 1:1 post scale
	P1SECMPbits.SEVTCMP = PWM_ADC_SYNC_TICKS;	//ADC trigger - specifiy the phase realtive to PWM
#endif

#ifdef M3_BMW_A2R1
	//RP14 is PWM1H1 which goes to amp enable
	//RP13 is DIR
	//RP12 is PWM1H2 which goes low (to make vref high) (v0.1)
	PWMCON1bits.PMOD2=1;	//PWM1H2,PWM1L2 is independent pair
	PWMCON1bits.PMOD1=1;	//PWM1H1,PWM1L1 is independent pair
	PWMCON1bits.PEN1H = 1;	//Enable PWM1H1 (RP14)
	PDC1 = 0; // default PWM period
	SetBWMVref;
	//Synchronize ADC to PWM
	PWMCON2 = 0;
	PWMCON2bits.SEVOPS = 0;						// Special event 1:1 post scale
	P1SECMPbits.SEVTCMP = PWM_ADC_SYNC_TICKS;	//ADC trigger - specifiy the phase realtive to PWM
#endif

#if defined M3_HB2_H2R1_J0J1 || defined M3_HB2_H2R2_J0J1 || defined M3_HB2_H2R3_J0J1 
	
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


#if defined M3_HB2_H2R1_J2J3J4 || defined M3_HB2_H2R2_J2J3J4 || defined M3_HB2_H2R3_J2J3J4
	
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

#ifdef M3_WMA_0_1
	PWMCON1bits.PMOD1=0;	//PWM1H1,PWM1L1 is complimentary pair
	PWMCON1bits.PMOD2=0;	//PWM1H2,PWM1L2 is complimentary pair
	PWMCON1bits.PEN1H = 1;	//Enable PWM1H1
	PWMCON1bits.PEN1L = 1;	//Enable PWM1L1
	PWMCON1bits.PEN2H = 1;	//Enable PWM1H2
	PWMCON1bits.PEN2L = 1;	//Enable PWM1L2
	//Synchronize ADC to PWM
	PWMCON2 = 0;
	PWMCON2bits.SEVOPS = 0;				// Special event 1:1 post scale
	P1SECMPbits.SEVTCMP = PWM_ADC_SYNC_TICKS;	//ADC trigger - specifiy the phase realtive to PWM
	//Set PWM dead time
	DTCON1 = 0;					//Deadtime clock base is 1:1 TCY for Unit A, Unit B
	DTCON1bits.DTA = PWM_DEAD_CYC_A;
	DTCON1bits.DTB = PWM_DEAD_CYC_B;
	DTCON2 = 0;
	DTCON2bits.DTS1A = 0;		//Deadtime for PWM1H1/PWM1L1 going active from Unit A
	DTCON2bits.DTS1I = 1;		//Deadtime for PWM1H1/PWM1L1 going inactive from Unit B
	DTCON2bits.DTS2A = 0;		//Deadtime for PWM1H2/PWM1L2 going active from Unit A
	DTCON2bits.DTS2I = 1;		//Deadtime for PWM1H2/PWM1L2 going inactive from Unit B
	// put in some default PWM periods
	PDC1 = 0;
	PDC2 = 0;
#endif

#if defined M3_HMB_H1R1 || defined M3_GMB_G1R1 || defined M3_HEX2_S1R1
	PWMCON1bits.PMOD1=1;	//PWM1H1,PWM1L1 is independent pair
	PWMCON1bits.PMOD2=1;	//PWM1H2,PWM1L2 is independent pair
	PWMCON1bits.PEN1H = 1;	//Enable PWM1H1
	PWMCON1bits.PEN1L = 0;	//Disable PWM1L1
	PWMCON1bits.PEN2H = 1;	//Enable PWM1H2
	PWMCON1bits.PEN2L = 0;	//Disable PWM1L2
	PDC1 = 0; // default PWM period
	PDC2 = 0; // default PWM period
	//Synchronize ADC to PWM
	PWMCON2 = 0;
	PWMCON2bits.SEVOPS = 0;						// Special event 1:1 post scale
	PWMCON2bits.IUE=1;							//Immediate update of duty cycle
	P1SECMPbits.SEVTCMP = PWM_ADC_SYNC_TICKS;	//ADC trigger - specifiy the phase realtive to PWM
#endif
#if defined M3_GMB_G1R1
	PWMCON1bits.PMOD1=1;	//PWM1H1,PWM1L1 is independent pair
	PWMCON1bits.PMOD2=1;	//PWM1H2,PWM1L2 is independent pair
	PWMCON1bits.PEN1H = 1;	//Enable PWM1H1  PWM ChA
	PWMCON1bits.PEN1L = 0;	//Disable PWM1L1 DIR ChB
	PWMCON1bits.PEN2H = 1;	//Enable PWM1H2  PWM ChB
	PWMCON1bits.PEN2L = 0;	//Disable PWM1L2 DIR ChA
	PDC1 = 0; // default PWM period
	PDC2 = 0; // default PWM period
	//Synchronize ADC to PWM
	PWMCON2 = 0;
	PWMCON2bits.SEVOPS = 0;						// Special event 1:1 post scale
	PWMCON2bits.IUE=1;							//Immediate update of duty cycle
	P1SECMPbits.SEVTCMP = PWM_ADC_SYNC_TICKS;	//ADC trigger - specifiy the phase realtive to PWM
#endif
	//Disable faults
	FLTACON = 0;
	_PWM1IF=0;
	PTCONbits.PTEN = 1;  // PWM enable
}
#endif 
