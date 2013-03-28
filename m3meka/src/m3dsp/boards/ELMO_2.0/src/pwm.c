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

unsigned int actual_pwm = 0;
int pwm_cmd_buf[NUM_PWM_CH];
int pwm_duty_buf[NUM_PWM_CH];
int get_pwm_cmd(int chid){return pwm_cmd_buf[chid];}

static int pwm_desired;
static int pwm_current_desired;

void update_pwm()
{
        switch (get_dsp_state()) {
        case DSP_PWM:
            set_pwm(0,pwm_desired);
            break;
        case DSP_CURRENT:
            set_pwm(0,pwm_current_desired);
            break;
        default:
            set_pwm(0,0);
            break;
    }
}

void set_pwm_desired(int pwm)
{
    pwm_desired = pwm;
}

void set_pwm_current_desired(int pwm)
{
    pwm_current_desired = pwm;
}
int pwm_deadband(int chid, int abs_val)
{
/*	if (abs_val==0)
		return 0;
	if (ec_cmd.command[chid].pwm_db<0)
	{
		if (abs_val<ABS(ec_cmd.command[chid].pwm_db))
			abs_val=0;
	}
	else
		abs_val=abs_val+ec_cmd.command[chid].pwm_db; */
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
	//pwm_cmd_buf[chid]=val;
	#if defined USE_CURRENT
	     if (get_current_state()!=CURRENT_READY)
			val=0;
	#endif
	//ec_cmd.command[chid].pwm_max = PWM_MAX_DUTY;
	int sign=SIGN(val);
        // pwm_cmd_buf[chid]=sign*CLAMP(ABS(val),0,ec_cmd.command[chid].pwm_max); //Send back commanded value before gets inverted, deadband, etc



        //pwm_cmd_buf[chid]=sign*CLAMP(ABS(val),0,PWM_MAX_DUTY); //Send back commanded value before gets inverted, deadband, etc
//	val = pwm_deadband(chid,ABS(val));
	
	//Used by correct_ma
	#ifdef USE_CURRENT
	actual_pwm = val;
	#endif
	
	//ADC trigger at the middle of a PWM pulse
	//P1SECMPbits.SEVTCMP = MAX(pwm_cmd_buf[chid] >> 2, 50); //MAX(val - 50, 20);	//Fixed value

	#ifdef USE_BLDC
	#if defined PWM_4Q 
//		pwm_duty_buf[chid]=invert_and_clamp_pwm(chid,val); //Invert as switching on low-leg

                pwm_duty_buf[chid] = clamp_pwm(chid,ABS(val));
                pwm_cmd_buf[chid]=pwm_duty_buf[chid];
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


        if (sign<0) {
            set_bldc_dir(0);
        } else
            set_bldc_dir(1);

	#else	//BLDC or not
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
	#endif //#ifdef USE_BLDC
}

void setup_pwm(void) {
	int i;
	for (i=0;i<NUM_PWM_CH;i++)
	{
		pwm_cmd_buf[i]=0;
		pwm_duty_buf[i]=0;
	}

	PTCONbits.PTEN = 0;  		// PWM disable
	//Set time base
	PTCON  = 0;
	PTCONbits.PTOPS=0;			// PWM 1:1 postcale
	PTCONbits.PTCKPS = 0;		// PWM timebase input clock prescale 1:1 (Tcy)
	PTCONbits.PTMOD = 2;		// up down counting mode "center aligned pwm"

        PTPER = PWM_TIMEBASE_CYC;	// set timebase period
	PTMR = 0;					// Reset counter

	PWMCON1 = 0;				// reset condition specified in CONFIG BITS FPOR
	PWMCON1bits.PMOD1=0;		//PWM1H1,PWM1L1 is complimentary pair
	PWMCON1bits.PMOD2=0;		//PWM1H2,PWM1L2 is complimentary pair
	PWMCON1bits.PMOD3=0;		//PWM1H3,PWM1L3 is complimentary pair
	PWMCON1bits.PEN1H = 1;		//Enable PWM1H1
	PWMCON1bits.PEN1L = 1;		//Enable PWM1L1
	PWMCON1bits.PEN2H = 1;		//Enable PWM1H2
	PWMCON1bits.PEN2L = 1;		//Enable PWM1L2
        PWMCON1bits.PEN3H = 1;          //Enable PWM1H3
	PWMCON1bits.PEN3L = 1;          //Enable PWM1L3


	//Synchronize ADC to PWM
	PWMCON2 = 0;			
	PWMCON2bits.SEVOPS = 0;	// Special event 1:1 post scale	
	PWMCON2bits.IUE=0;	//Update of duty cycle sync to PWM time base (default)
	//PWMCON2bits.IUE=1;		//Immediate update of duty cycle
	PWMCON2bits.OSYNC=0;		//OVDCON overrides are on next Tcy boundary
	//PWMCON2bits.OSYNC=1;		//OVDCON overrides synchronized to PWM time base

        P1SECMPbits.SEVTDIR = 1;        // trigger on counting down
        P1SECMPbits.SEVTCMP = PWM_TIMEBASE_CYC;        // trigger in center of on state
	//P1SECMPbits.SEVTCMP = PWM_ADC_SYNC_TICKS;	//ADC trigger - specifiy the phase realtive to PWM
	//Note: see current.c, this value is changing at runtime
	
	//Set PWM dead time
	DTCON1 = 0;					//Deadtime clock base is 1:1 TCY for Unit A, Unit B
	DTCON1bits.DTA = PWM_DEAD_CYC_A;
	DTCON1bits.DTB = PWM_DEAD_CYC_B;
	DTCON2 = 0;
	DTCON2bits.DTS1A = 0;		//Deadtime for PWM1H1/PWM1L1 going active from Unit A
	DTCON2bits.DTS1I = 1;		//Deadtime for PWM1H1/PWM1L1 going inactive from Unit B
	DTCON2bits.DTS2A = 0;		//Deadtime for PWM1H2/PWM1L2 going active from Unit A
	DTCON2bits.DTS2I = 1;		//Deadtime for PWM1H2/PWM1L2 going inactive from Unit B
	DTCON2bits.DTS3A = 0;           //Deadtime for PWM1H3/PWM1L3 going active from Unit A
	DTCON2bits.DTS3I = 1;           //Deadtime for PWM1H3/PWM1L3 going inactive from Unit B
	
	
	#ifdef USE_TIMER1
	T1CONbits.TON = 1;   	 	//Start Timer 1 for ADC Sync functionality
	#endif

        PDC1 = 0;
	PDC2 = 0;
        PDC3 = 0;
	
	//Disable faults
	FLTACON = 0;
	_PWM1IF = 0;
	_PWM1IE = 0;				//Disable interrupts	
	PTCONbits.PTEN = 1;  		// PWM enable
}
#endif 


//PWM1 Interrupt - Trig on Match
/*
void __attribute__ ((interrupt, no_auto_psv)) _MPWM1Interrupt(void)
{
	_PWM1IF = 0;	//Clear flag
	//Interrupt disabled, should never be called
}
*/
