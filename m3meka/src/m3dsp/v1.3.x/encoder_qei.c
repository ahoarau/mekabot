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
#include "encoder_ma3.h"
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

int qei_error()
{
	return qei_err;
}

//return lower word
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

void step_qei()
{
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

void __attribute__((__interrupt__, no_auto_psv)) _QEIInterrupt(void)
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
	
	_QEIIF = 0;		//Clear the flag

}

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
  _QEIIE=1;				//Enable interrupt

}

#endif
