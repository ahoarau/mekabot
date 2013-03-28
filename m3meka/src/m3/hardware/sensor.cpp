/* 
M3 -- Meka Robotics Robot Components
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

#include "sensor.h"

namespace m3{
	
using namespace std;

///////////////////////////////////////////////////////
#define MA3_10BIT_MAX_PULSE_US 1024
#define MA3_10BIT_PERIOD_US  MA3_10BIT_MAX_PULSE_US+1
#define MA3_12BIT_MAX_PULSE_US 4096
#define MA3_12BIT_PERIOD_US  MA3_12BIT_MAX_PULSE_US+1
#define VERTX_14BIT_MAX  16383 


void M3AngleSensor::ReadConfig(const YAML::Node & doc)
{
	string t;
	doc["type"] >> t;
	if (t.compare("vertx_14bit")==0){type=VERTX_14_BIT;}
	if (t.compare("ma3_12bit")==0){type=MA3_12_BIT;}
	if (t.compare("ma3_10bit")==0){type=MA3_10_BIT;}
	if (t.compare("ma3_12bit_poly")==0){type=MA3_12_BIT_POLY;}
	if (t.compare("qei")==0){type=QEI;}
	if (type==VERTX_14_BIT || type==MA3_12_BIT || type==MA3_10_BIT)
	{
		doc["cb_scale"]>>cb_scale;
		doc["cb_bias"]>>cb_bias;
	}
	if (type==MA3_12_BIT_POLY) //Older format v0.0-v0.5
		doc["cb_theta"]>>cb_theta;
		doc["cb_scale"]>>cb_scale;
		doc["cb_bias"]>>cb_bias;
	if (type==QEI)
	{
	    doc["cb_scale"]>>cb_scale;
	    doc["cb_bias"]>>cb_bias;
	    doc["cb_ticks_per_rev"]>>cb_ticks_per_rev;
	}
	
}

mReal M3AngleSensor::GetThetaDeg()
{    
    return val;

}

void M3AngleSensor::Step(int qei_on, int qei_period, int qei_rollover)
{	
	if (type==QEI)
	{
		unsigned int low=(unsigned short)qei_on;
		unsigned int high=((unsigned short)qei_rollover<<16);
		int ticks = high|low;		
		val = 360.0*(mReal(ticks)/cb_ticks_per_rev);
		val= val*cb_scale+cb_bias;	
		//mReal secs = qei_period*1.0/(40000000.0/8.0);
		//velocity = (360.0/8192.0)/secs; // TODO: make these params in yaml
		velocity = (360.0/8192.0) * ((mReal)qei_period) * 1000.0;
		//if (ABS(velocity) < 7)
		  //velocity = 0.0;
		return;
	}
	if (type==VERTX_14_BIT )
	{
		val = 360.0*(mReal(qei_on))/(mReal)VERTX_14BIT_MAX;//deg
		val= val*cb_scale+cb_bias;
		return;
	}
	
	if (type==MA3_12_BIT )
	{
		if (qei_period==0) 
		{
			val = 0.0;
			return;
		}
		val = ((mReal(qei_on)*MA3_12BIT_PERIOD_US)/mReal(qei_period))-1; //0-4095
		val= 360.0*(val/MA3_12BIT_MAX_PULSE_US);
		val= val*cb_scale+cb_bias;
		return;
	}
	if (type==MA3_10_BIT)
	{
		if (qei_period==0)
		{
			val = 0.0;
			return;
		}
		val = ((mReal(qei_on)*MA3_10BIT_PERIOD_US)/mReal(qei_period))-1; //0-1023
		val= 360.0*(val/MA3_10BIT_MAX_PULSE_US);
		val= val*cb_scale+cb_bias;
		return;
	}
	if (type==MA3_12_BIT_POLY)
	{
		if (qei_period==0) 
		{
			val = 0.0;
			return;
		}
		val = ((mReal(qei_on)*MA3_12BIT_PERIOD_US)/mReal(qei_period))-1; //0-4095
		
		// ToDo: see below
		val=EvalCalibrationPoly(cb_theta,val);
		val= val*cb_scale+cb_bias;
		return;
	}
}

void M3AngleSensor::StepError(int ec_err)
{	
	
	if (type==VERTX_14_BIT )
	{
	   err = ec_err;
	
	} else {
	  err = 0; 
	}
	
	
}


///////////////////////////////////////////////////////



void M3CurrentSensor::ReadConfig(const YAML::Node & doc)
{
	string t;
	doc["type"] >> t;
	if (t.compare("none")==0){type=NONE; return;}
	if (t.compare("adc_poly")==0){type=ADC_POLY;}
	if (t.compare("adc_poly_single")==0){type=ADC_POLY_SINGLE;}
	if (t.compare("adc_linear_5V")==0){type=ADC_LINEAR_5V;}
	if (t.compare("adc_linear_5V_ns")==0){type=ADC_LINEAR_5V_NS;}
	if (t.compare("linear_amp_via_dac")==0){type=LINEAR_AMP_VIA_DAC;}
	if (t.compare("dsp_calib")==0){type=DSP_CALIB;}
	if (t.compare("dsp_ticks")==0){type=DSP_TICKS;}
	doc["cb_scale"]>>cb_scale;
	doc["cb_bias"]>>cb_bias;
	if (type==ADC_POLY)
	{
		doc["cb_current_a"]>>cb_current_a;
		doc["cb_current_b"]>>cb_current_b;
	}
	if (type==ADC_POLY_SINGLE)
	{
		doc["cb_current_a"]>>cb_current_a;
	}
	if (type==ADC_LINEAR_5V ||type==ADC_LINEAR_5V_NS)
	{
		doc["cb_mV_per_A"]>>cb_mV_per_A;
		doc["cb_ticks_at_zero_a"]>>cb_ticks_at_zero_a;
		doc["cb_ticks_at_zero_b"]>>cb_ticks_at_zero_b;
	}
	if (type==LINEAR_AMP_VIA_DAC)
	{
		doc["cb_dac_mV_per_tick"]>>cb_dac_mV_per_tick;
		doc["cb_amp_mA_per_mV"]>>cb_amp_mA_per_mV;
	}
}

int M3CurrentSensor::mAtoTicks(mReal milliamps)
{
    if (type==LINEAR_AMP_VIA_DAC)
    {
	  return int((milliamps - cb_bias) * (1.0/cb_scale) * (1.0/(cb_amp_mA_per_mV * cb_dac_mV_per_tick)));
    } 
    if (type==DSP_TICKS)
    {
      return int((milliamps - cb_bias) * (1.0/cb_scale));
    } 
    return 0;
}

void M3CurrentSensor::Step(mReal ticks_a, mReal ticks_b, mReal current_ma)
{
	if (type==ADC_POLY)
	{
		mReal i_a = EvalCalibrationPoly(cb_current_a,ticks_a);
		mReal i_b = EvalCalibrationPoly(cb_current_b,ticks_b);
		mReal s=MAX(ABS(i_a), ABS(i_b));
		val= MAX(0,s*cb_scale+cb_bias);
	}
	if (type==ADC_POLY_SINGLE)
	{
		mReal i_a = EvalCalibrationPoly(cb_current_a,ticks_a);		
		val= MAX(0,i_a*cb_scale+cb_bias);		
	}
	if (type==LINEAR_AMP_VIA_DAC)
	{
		val= (ticks_a*cb_dac_mV_per_tick*cb_amp_mA_per_mV)*cb_scale+cb_bias;
	}
	if (type==ADC_LINEAR_5V)
	{
		mReal mV_a = 5000.0*(ticks_a-cb_ticks_at_zero_a)/M3EC_ADC_TICKS_MAX;
		mReal mV_b = 5000.0*(ticks_b-cb_ticks_at_zero_b)/M3EC_ADC_TICKS_MAX;
		mReal i_a = 1000.0*mV_a/cb_mV_per_A;
		mReal i_b = 1000.0*mV_b/cb_mV_per_A;
		mReal s=MAX(ABS(i_a), ABS(i_b));
		val= MAX(0,s*cb_scale+cb_bias);
	}
	if (type==ADC_LINEAR_5V_NS)
	{
		mReal mV_a = 3300.0*(ticks_a-cb_ticks_at_zero_a)/M3EC_ADC_TICKS_MAX;
		mReal mV_b = 3300.0*(ticks_b-cb_ticks_at_zero_b)/M3EC_ADC_TICKS_MAX;
		mReal i_a = 1000.0*mV_a/cb_mV_per_A;
		mReal i_b = 1000.0*mV_b/cb_mV_per_A;
		mReal s=MAX(ABS(i_a), ABS(i_b));
		val= MAX(0,s*cb_scale+cb_bias);
	}
	if (type==DSP_CALIB)
	{
		val= current_ma*cb_scale+cb_bias;
	}
	if (type==DSP_TICKS)
	{
		val= ticks_a*cb_scale+cb_bias;
	}
	if (type==NONE) val=0.0;
}
///////////////////////////////////////////////////////

void M3VoltageSensor::ReadConfig(const YAML::Node & doc)
{
	string t;
	doc["type"] >> t;
	if (t.compare("none")==0){type=NONE; return;}
	if (t.compare("adc_poly")==0){type=ADC_POLY;}
	doc["cb_scale"]>>cb_scale;
	doc["cb_bias"]>>cb_bias;
	if (type==ADC_POLY)
	{
		doc["cb_voltage"]>>cb_voltage;
		doc["cb_inv_voltage"]>>cb_inv_voltage;
	}
}

void M3VoltageSensor::Step(mReal ticks)
{
	if (type==ADC_POLY)
	{
		mReal v = EvalCalibrationPoly(cb_voltage,ticks);
		val=v*cb_scale+cb_bias;
	}
}
///////////////////////////////////////////////////////

void M3TorqueSensor::ReadConfig(const YAML::Node & doc)
{
	string t;
	doc["type"] >> t;
	if (t.compare("none")==0){type=NONE; return;}
	if (t.compare("adc_poly")==0){type=ADC_POLY;}
	if (t.compare("sea_vertx_14bit")==0){type=SEA_VERTX_14_BIT;}
	if (t.compare("ff_current_ctrl")==0){type=FF_CURRENT_CTRL;}
	doc["cb_scale"]>>cb_scale;
	doc["cb_bias"]>>cb_bias;
	if (type==ADC_POLY || type==SEA_VERTX_14_BIT)
	{
		doc["cb_torque"]>>cb_torque;
		doc["cb_inv_torque"]>>cb_inv_torque;
	}
	if (type==FF_CURRENT_CTRL)
	{
		doc["cb_mNm_per_mA"]>>cb_mNm_per_mA;
		doc["cb_mA_per_tick"]>>cb_mA_per_tick;
	}
}

void M3TorqueSensor::Step(mReal ticks)
{
	if (type==NONE)
	  return;
	if (type== ADC_POLY|| type==SEA_VERTX_14_BIT)
	{
		
		val =EvalCalibrationPoly(cb_torque,ticks);
		
	}
	if (type==FF_CURRENT_CTRL)
	{   // NOTE: this value is off because we are not including current bias
		val = ticks*cb_mA_per_tick*cb_mNm_per_mA;
	}
	val= val*cb_scale+cb_bias;
}

void M3TorqueSensor::StepError(int ec_err)
{	
	
	if (type==SEA_VERTX_14_BIT )
	{
	   err = ec_err;
	
	} else {
	  err = 0; 
	}
	
	
}

int  M3TorqueSensor::mNmToTicks(mReal mNm, M3CurrentSensor * current)
{
    if (type==FF_CURRENT_CTRL) 
    {  
      return current->mAtoTicks((mNm - cb_bias) * (1.0/cb_scale) * (1.0/cb_mNm_per_mA));
    }
    
    return 0.0;
}

int M3TorqueSensor::mNmToTicks(mReal x)
{
	if (type==NONE)
	  return 0;
	
	if (type==ADC_POLY)
		return CLAMP((int)EvalCalibrationPoly(cb_inv_torque,(x-cb_bias)/cb_scale),0,M3EC_ADC_TICKS_MAX);
	if (type==SEA_VERTX_14_BIT)
		return CLAMP((int)EvalCalibrationPoly(cb_inv_torque,(x-cb_bias)/cb_scale),0,VERTX_14BIT_MAX);
	/*if (type==FF_CURRENT_CTRL) {
	    //M3_INFO("%f, %f, %f\n",cb_scale,cb_mNm_per_mA,cb_mA_per_tick);
	    return (int)(((x-cb_bias)/cb_scale)/cb_mNm_per_mA)/cb_mA_per_tick;
	}*/
	return 0;
}
///////////////////////////////////////////////////////

void M3WrenchSensor::ReadConfig(const YAML::Node & doc)
{
	string t;
	doc["type"] >> t;
	if (t.compare("none")==0){type=NONE; return;}
	if (t.compare("6x6_linear")==0){type=W_6X6_LINEAR;}
	if (type==W_6X6_LINEAR)
	{
		doc["cb_adc_bias"]>>cb_adc_bias;
		doc["cb_scale"]>>cb_scale;
		doc["cb_bias"]>>cb_bias;
		doc["cb_fx"]>>cb_fx;
		doc["cb_fy"]>>cb_fy;
		doc["cb_fz"]>>cb_fz;
		doc["cb_tx"]>>cb_tx;
		doc["cb_ty"]>>cb_ty;
		doc["cb_tz"]>>cb_tz;
		/*for (int i=0;i<6;i++)
		{
			C(0,i)=cb_fx[i];
			C(1,i)=cb_fy[i];
			C(2,i)=cb_fz[i];
			C(3,i)=cb_tx[i];
			C(4,i)=cb_ty[i];
			C(5,i)=cb_tz[i];
			z(i)=cb_adc_bias[i];
			b(i)=cb_bias[i];
		}*/
		//Temp workaround. Eigen broken...
		int i;
		for (i=0;i<6;i++) C[i]=	  cb_fx[i];
		for (i=0;i<6;i++) C[i+6]= cb_fy[i];
		for (i=0;i<6;i++) C[i+12]=cb_fz[i];
		for (i=0;i<6;i++) C[i+18]=cb_tx[i];
		for (i=0;i<6;i++) C[i+24]=cb_ty[i];
		for (i=0;i<6;i++) C[i+30]=cb_tz[i];
	}
}

void M3WrenchSensor::Step(mReal ticks_0,mReal ticks_1,mReal ticks_2,
			  mReal ticks_3,mReal ticks_4,mReal ticks_5)
{
	if (type==W_6X6_LINEAR)
	{
		
		temp[0]=ticks_0-cb_adc_bias[0];
		temp[1]=ticks_1-cb_adc_bias[1];
		temp[2]=ticks_2-cb_adc_bias[2];
		temp[3]=ticks_3-cb_adc_bias[3];
		temp[4]=ticks_4-cb_adc_bias[4];
		temp[5]=ticks_5-cb_adc_bias[5];
		
		//Temp workaround. Eigen broken...
		
		for (int i=0;i<6;i++)
		{
			wrench[i]=0;
			for (int j=0;j<6;j++)
			{
				wrench[i]+=C[i*6+j]*temp[j];
			}
			wrench[i]=wrench[i]*cb_scale+cb_bias[i];
		}
		
		/*temp=C*(ticks-z);
		temp= temp*cb_scale+b;
		wrench.force[0]=temp(0);
		wrench.force[1]=temp(1);
		wrench.force[2]=temp(2);
		wrench.torque[0]=temp(3);
		wrench.torque[1]=temp(4);
		wrench.torque[2]=temp(5);*/
	}
}

///////////////////////////////////////////////////////


void M3TempSensor::ReadConfig(const YAML::Node & doc)
{
	string t;
	doc["type"] >> t;
	if (t.compare("none")==0){type=NONE; return;}
	if (t.compare("temp_25C")==0){type=TEMP_25C; return;}
	if (t.compare("adc_poly")==0){type=ADC_POLY;}
	if (t.compare("adc_linear_3V3")==0){type=ADC_LINEAR_3V3;}
	if (t.compare("adc_linear_5V")==0){type=ADC_LINEAR_5V;}
	if (t.compare("dsp_calib")==0){type=DSP_CALIB;}
	doc["cb_scale"]>>cb_scale;
	doc["cb_bias"]>>cb_bias;
	if (type==ADC_POLY) //Older format v0.0-v0.5
		doc["cb_temp"]>>cb_temp;
		//cb_temp=YamlReadVectorM(doc["cb_temp"]);
	if (type==ADC_LINEAR_3V3 || type == ADC_LINEAR_5V)
	{
		doc["cb_mV_at_25C"]>>cb_mV_at_25C;
		doc["cb_mV_per_C"] >>cb_mV_per_C;
	}
}


void M3TempSensor::Step(mReal ticks)
{
	if (type==TEMP_25C)
	{
		val=25.0;
		return;
	}
	if (type==ADC_POLY)
	{
		val=EvalCalibrationPoly(cb_temp,ticks);
		val= val*cb_scale+cb_bias;
		return;
	}

	if (type==ADC_LINEAR_3V3 )
	{
		mReal mV= ticks/(M3EC_ADC_TICKS_MAX/3300.0);
		mReal bias = 25.0-(cb_mV_at_25C/cb_mV_per_C); //C
		val = mV/cb_mV_per_C + bias;
		val= val*cb_scale+cb_bias;
		return;
	}
	
	if (type == ADC_LINEAR_5V)
	{
		mReal mV= ticks/(M3EC_ADC_TICKS_MAX/5000.0);
		mReal bias = 25.0-(cb_mV_at_25C/cb_mV_per_C); //C
		val = mV/cb_mV_per_C + bias;
		val= val*cb_scale+cb_bias;
	}
	if (type == DSP_CALIB)
	{
		val = ticks*cb_scale + cb_bias;
	}
}

} //namespace
///////////////////////////////////////////////////////
