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

#ifdef USE_ENCODER_QEI

#include "p33Fxxxx.h"
#include "setup.h"
//#include "encoder_ma3.h"
#include "ethercat.h"
#include "timer3.h"
#include "dio.h"


long qei_pos;
long qei_roll;
int qei_err;
int qei_calibrate_flag_last;
int qei_calibration_req;
int qei_limitswitch_last;
int qei_zero_last;
int qei_calibrated;
int qei_index_cnt;

static int last_poscnt = 0;

static uint16_t	last_t;
static int32_t	last_per;

static int32_t	qei_per;

static uint32_t sum_ec;
static uint32_t last_sum;

static uint16_t last_tmr2;

static uint16_t samp_flag;			// indicates that the last time there was a captured edge

static uint32_t last_qei;

static uint16_t	qei_tmp;
static uint16_t	last_qei_tmp;


static int32_t t1;
static int32_t t2;

static int qei_vel;

int16_t idx_offset;
static int16_t index_flag;


#if defined M3_MAX2_BDC_ARMH || defined  M3_MAX2_BDC_ARMH2
int qei_zero_cnt=0;
//Do some debounce before zero-ing encoder
void step_qei_zero()
{
#if defined M3_MAX2_BDC_ARMH
	if (PinZero)
		qei_zero_cnt++;
	else
		qei_zero_cnt=0;
	if (qei_zero_cnt>=100)
		qei_zero();
#endif
#if defined M3_MAX2_BDC_ARMH2
	qei_zero_cnt=0;
	if (qei_zero_cnt>=100)
		qei_zero();
#endif

}
#endif

void set_idx_offset(int16_t i)
{
	idx_offset = i;
}


int16_t get_period_16()
{
	if (qei_per > 32767)
		return (int16_t)32767;
	else if (qei_per < -32767)
		return (int16_t)-32767;
	else
		return (int16_t)qei_per;
}

#if 0
void step_period()
{
	static int32_t	per;
	static uint16_t snap_tmr2;
	static uint32_t tmp;

	qei_tmp = 0;

	snap_tmr2 = TMR2;


	if (IC1CONbits.ICBNE == 0)													// No edges since last time...
	{
		if (samp_flag == 1)
		{
			if (last_t > snap_tmr2)
				sum_ec = (uint32_t)(PR2 - last_t) + snap_tmr2;
			else
			{
				sum_ec = (uint32_t)snap_tmr2 - (uint32_t)last_t;
			}

			samp_flag = 0;
		}
		else
		{
			if (last_tmr2 > snap_tmr2)
			{
				tmp = (uint32_t)(PR2 - last_tmr2) + snap_tmr2;
			}
			else
			{
				tmp = (uint32_t)snap_tmr2 - (uint32_t)last_tmr2;
			}

			if (0x7FFFFFFF - sum_ec < tmp)										// Check if adding tmp will overload sum_ec
				sum_ec = 0x7FFFFFFF;
			else
				sum_ec += tmp;
		}


		if (ABS(last_per) > sum_ec)												// Don't use the sum until it's larger than the last measured period...
		{
			per = last_per;
		}
		else
		{
			// if the calculated period (before setting the direction) is greater than the last period...
			if (sum_ec > (last_sum+60000 ) )
			{
				// ignore the calculated value...
				sum_ec = last_sum;

			}

			last_sum = sum_ec;


			if (QEICONbits.UPDN == 0)	// negative
				per = -sum_ec;
			else
				per = sum_ec;
		}

		if (per == 0)
		{
			per = last_per;
		}

	}
	else																		// At least one edge came in...
	{
		t1	= IC1BUF;

		if (IC1CONbits.ICBNE == 1)												// More edges...
		{
			t2 = IC1BUF;
			while(IC1CONbits.ICBNE == 1) last_t = IC1BUF;						// Loop until the capture buffer is empty

			if( t2 > t1 )
			{
				per	= (t2 - t1);
			}
			else if (t2 == t1)	// this is bad
			{
				// pretend it didn't happen...
				per = last_per;

			}
			else
			{
				per	= (PR2 - t1) + t2;
			}

			last_t	= t2;

		}
		else
		{
			if (samp_flag == 1)
			{
				if (t1 > last_t)
					per = t1 - last_t;
				else
					per = (PR2 - last_t) + t1;
			}
			else
			{
				if (t1 > last_tmr2)
					per = sum_ec + t1 - last_tmr2;
				else
					per = sum_ec + (PR2 - last_tmr2) + t1;
			}

			last_t	= t1;
		}

		if (per <= 1)
		{
			per = last_per;
		}
		else
		{

			// if the calculated period (before setting the direction) is greater than the last period...
			if (per > (ABS(last_per)+60000 ) )
			{
				// ignore the calculated value...
				per = ABS(last_per);

			}

			// set the direction...
			if (QEICONbits.UPDN == 0)
			{
				per = -per;
			}
			last_per = per;
		}



		last_qei	= qei_pos;

		samp_flag	= 1;

		sum_ec		= 0;

	}


	last_tmr2 = snap_tmr2;



	qei_per = per;


} // end step_per
#endif

void set_index_flag(int16_t x)
{
	index_flag = x;
}


int32_t get_period()
{
	return qei_per;
}

int  get_period_LW()
{
	return (int)(qei_per & 0X0000FFFF);
}
int  get_period_HW()
{
	return (int)((qei_per>>16)& 0X0000FFFF);
}

int qei_error()
{
	return qei_err;
}

//return lower word

int  qei_freeze_LW()
{
	return (int)(get_qei_freeze() & 0X0000FFFF);
}

int  qei_freeze_HW()
{
	return (int)((get_qei_freeze()>>16)& 0X0000FFFF);
}

int  qei_position_LW()
{
	return (int)(qei_pos & 0X0000FFFF);
}
int  qei_position_HW()
{
	return (int)((qei_pos>>16)& 0X0000FFFF);
}
long qei_position()
{
	return qei_pos;
}
int qei_calibrated_flag()
{
  return qei_calibrated;
}

void qei_zero()
{
	POSCNT=0;
	qei_pos=0;
	qei_roll=0;
}

void step_bb()
{
    	//Signal for calibration on transition from 0 to 1 on the flag from the master
	if (!qei_calibration_req)
		qei_calibration_req=(ec_cmd.command[0].config & M3ACT_CONFIG_CALIB_QEI_LIMITSWITCH_NEG) && !qei_calibrate_flag_last;

	 if ( qei_calibration_req)
	 {
			  if (limit_switch_neg_flag() && ! qei_limitswitch_last) //require off-to-on transition
			  {
				  POSCNT=0;
				  qei_pos=0;
				  qei_roll=0;
				  qei_calibrated=M3ACT_FLAG_QEI_CALIBRATED;
				  qei_calibration_req=0;
			  }
	 }
      qei_limitswitch_last=limit_switch_neg_flag();
	  qei_calibrate_flag_last=ec_cmd.command[0].config & M3ACT_CONFIG_CALIB_QEI_LIMITSWITCH_NEG;
}

void step_qei()
{
#if 0
#if defined M3_ELMO_B1R1 || defined M3_ELMO_Z1R1
	//Signal for calibration on transition from 0 to 1 on the flag from the master
	if (!qei_calibration_req)
		qei_calibration_req=(ec_cmd.command[0].config & M3ACT_CONFIG_CALIB_QEI_LIMITSWITCH_NEG) && !qei_calibrate_flag_last;

	 if ( qei_calibration_req)
	 {
			  if (limit_switch_neg_flag() && ! qei_limitswitch_last) //require off-to-on transition
			  {
				  POSCNT=0;
				  qei_pos=0;
				  qei_roll=0;
				  qei_calibrated=M3ACT_FLAG_QEI_CALIBRATED;
				  qei_calibration_req=0;
			  }
	 }
      qei_limitswitch_last=limit_switch_neg_flag();
	  qei_calibrate_flag_last=ec_cmd.command[0].config & M3ACT_CONFIG_CALIB_QEI_LIMITSWITCH_NEG;
#endif
#endif
#if defined M3_MAX2_BLDC_A2R3_QEI
		if (ec_cmd.command[0].config & M3ACT_CONFIG_CALIB_QEI_MANUAL && ! qei_zero_last)
		{
				POSCNT=0;
				qei_pos=0;
				qei_roll=0;
				qei_calibrated=M3ACT_FLAG_QEI_CALIBRATED;
		}
		qei_zero_last=(ec_cmd.command[0].config & M3ACT_CONFIG_CALIB_QEI_MANUAL);
#endif
	  //if (ABS(qei_roll)<QEI_32B_LIMIT)
		qei_pos = qei_roll+POSCNT;

#if defined M3_MAX2_BDC_ARMH || defined  M3_MAX2_BDC_ARMH2
	step_qei_zero();
#endif

}

#if 1
void __attribute__((__interrupt__, no_auto_psv)) _QEI1Interrupt(void)
{	

#if defined QEI_USE_INDEX
	if (_INDX)
	{
		//if (ABS(qei_roll)<QEI_32B_LIMIT)
		//{
			if (_UPDN) //forward
			{
				qei_roll+=(QEI_MAXCNT+1);
				ec_debug[0]++;
			}
			else
			{
				qei_roll-=(QEI_MAXCNT+1);
				ec_debug[0]--;
			}
			qei_pos = qei_roll+POSCNT;
	//	}
	}
#else
//	if (ABS(qei_roll)<QEI_32B_LIMIT)
	//{
			if (_UPDN) //forward
					qei_roll+=(QEI_MAXCNT+1);
				else
					qei_roll-=(QEI_MAXCNT+1);
			qei_pos = qei_roll+POSCNT;
//Latch encoder timestamp on Rising edge.
#ifdef USE_TIMESTAMP_DC
	SetTimestampLatch;
	ClrTimestampLatch;
#endif
	//}
	//ec_debug[0]=qei_roll;
#endif
	if (_CNTERR )
	{
		qei_err++;
		_CNTERR=0;
	}
	
	_QEI1IF = 0;		//Clear the flag

}
#endif

#if 0
// Capture 2 Interrupt
void __attribute__((__interrupt__, no_auto_psv)) _IC2Interrupt(void)
{
	IEC0bits.IC2IE = 0;


	set_idx_offset(POS1CNT);
	set_index_flag(1);
	qei_zero();

	IFS0bits.IC2IF = 0;
}


// Timer 2 interrupt
void __attribute__((__interrupt__, no_auto_psv)) _T2Interrupt(void) {
	_T2IF = 0;
}
#endif


void setup_qei(void) {
qei_pos=0;
	qei_roll=0;
	qei_err=0;
	qei_limitswitch_last=1; //avoids startup at switch condition
	qei_calibrate_flag_last=0;//calibration will be enabled when master passes down flag for first time
	qei_calibration_req=0;//not req until signaled from master
	qei_zero_last=0;

  //Setup QEI
	MAXCNT = QEI_MAXCNT;
	QEICON = 0;
	POSCNT=0;
	QEICONbits.CNTERR=0; //Clear errors

#if defined QEI_USE_INDEX
	QEICONbits.POSRES = 1; //Enable index pulse reset
	QEICONbits.QEIM = 6;   //x4 mode with Index Pulse Reset
#else
	QEICONbits.POSRES = 0; //Disable index pulse reset
	QEICONbits.QEIM = 7;   //x4 mode with MAXCNT match reset
#endif
	//QEICONbits.SWPAB=0;		//Phase A/B Input not swapped
	QEICONbits.SWPAB=1;		//Phase A/B Input swapped

  //Setup Digital Filter
  //Pin must be high for 3 clock cycles (Tqeck) to be counted
  //For 1:2 divide, Tqeck=50ns, So pin must be high for 150ns.
  //For 2Mhz encoder, Tqei=500ns, pin is high for 250ns min.
  DFLTCON = 0;
  //DFLTCONbits.CEID = 0; //enable count error ints
  //DFLTCONbits.CEID = 1; //disable count error ints
  //DFLTCONbits.QECK = 4;//0x1;	// Digital filter clock  1:2 divide
  //DFLTCONbits.QEOUT = 0;  // Enable filter
  //DFLTCONbits.IMV0=0;	//Set Phase A for Index
  //DFLTCONbits.IMV1=0;	//Set Phase B for Index
  _QEI1IE=1;				//Enable interrupt

}

#endif
