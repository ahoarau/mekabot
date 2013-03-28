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

#ifndef M3_SENSOR_H
#define M3_SENSOR_H

#include "../toolbox/toolbox.h"
#include <m3rt/base/m3ec_def.h>
#include <kdl/frames.hpp>
#include <Eigen/Core>

USING_PART_OF_NAMESPACE_EIGEN

namespace m3
{
using namespace std;
using namespace KDL;
	
class M3AngleSensor 
{
	public:
		M3AngleSensor():tmp_cnt(0),val(0){}
		virtual void Step(int qei_on, int qei_period, int qei_rollover);
		virtual mReal GetThetaDeg();
		virtual mReal GetThetaDotDeg(){return velocity;}
		virtual mReal GetThetaRad(){return DEG2RAD(GetThetaDeg());}
		virtual mReal GetThetaDotRad(){return DEG2RAD(GetThetaDotDeg());}
		virtual bool IsTypeQEI(){return (type==QEI);}
		virtual void StepError(int ec_error);
		virtual int GetError(){return err;}
		virtual void ReadConfig(const YAML::Node & doc);
	protected:
		enum { NONE, VERTX_14_BIT, MA3_12_BIT, MA3_10_BIT, MA3_12_BIT_POLY, QEI };
		mReal cb_scale;
		mReal cb_bias;
		mReal cb_ticks_per_rev;
		mReal cb_ticks_per_rollover;
		vector<mReal> cb_theta;
		mReal val;
		mReal velocity;
		int type;
		int tmp_cnt;		
		int err;
};

/*
Temp sensor output is (generally) independent of supply voltage (3V3-5V)
Some inputs to the ADC are scaled by 3.3/5.0
USE adc_linear_3V3 for a signal that is fed directly to the ADC
USE adc_linear_5V  for a signal that is scaled by 3.3/5.0 before being fed to the ADC 
USE adc_poly for ad-hoc calibration
USE TMP_25C to provide a default temp of 25C in case no sensor present
*/
class M3TempSensor
{
	public:
		M3TempSensor():val(0){}
		virtual void Step(mReal ticks);
		virtual mReal GetTempC(){return val;}
		virtual mReal GetTempF(){return C2F(GetTempC());} 
		virtual void ReadConfig(const YAML::Node & doc);
	protected:
		enum {NONE, ADC_POLY, ADC_LINEAR_3V3, ADC_LINEAR_5V, TEMP_25C, DSP_CALIB};
		vector<mReal> cb_temp;
		mReal cb_mV_at_25C;
		mReal cb_mV_per_C;
		mReal cb_scale;
		mReal cb_bias;
		mReal val;
		int type;
};

/*
	USE adc_poly for ad-hoc calibration
	USE adc_linear_5V for a 0-5V range sensor converted to 0-3.3V
	USE adc_linear_5V_NS for 0-5V range sensor not scaled to 0-3.3V
	USE dsp_calib if current_ma from DSP is valid
*/
class M3CurrentSensor
{
	public:
		M3CurrentSensor():val(0){}
		virtual void Step(mReal ticks_a, mReal ticks_b, mReal current_ma);
		virtual mReal GetCurrent_mA(){return val;}
		mReal GetCurrent_A() const {return val/1000.0;}
		virtual void ReadConfig(const YAML::Node& doc);
		virtual int mAtoTicks(mReal milliamps);
		virtual void SetZero(){val=0.0;}
		
	protected:
		enum {NONE,  ADC_POLY, ADC_LINEAR_5V,ADC_LINEAR_5V_NS,LINEAR_AMP_VIA_DAC,
			ADC_POLY_SINGLE,DSP_CALIB, DSP_TICKS, ADC_PHASE_MA};
		mReal val;
		vector<mReal> cb_current_a, cb_current_b;
		int type;
		mReal cb_scale;
		mReal cb_bias;
		mReal cb_mV_per_A;
		mReal cb_ticks_at_zero_a;
		mReal cb_ticks_at_zero_b;
		mReal cb_dac_mV_per_tick;
		mReal cb_amp_mA_per_mV;
		int tmp_cnt;

};

class M3TorqueSensor 
{
	public:
		M3TorqueSensor():val(0),tmp_cnt(0){}
		virtual void Step(mReal ticks);
		virtual mReal GetTorque_mNm(){return val;}
		int mNmToTicks(mReal x, M3CurrentSensor * current);
		int mNmToTicks(mReal x);
		virtual void ReadConfig(const YAML::Node& doc);
		bool IsFFCurrentCtrl(){return (type == FF_CURRENT_CTRL);}
		virtual void StepError(int ec_error);
		virtual int GetError(){return err;}
	protected:
		enum {NONE, ADC_POLY, SEA_VERTX_14_BIT, FF_CURRENT_CTRL};
		vector<mReal> cb_torque, cb_inv_torque;
		mReal val;
		mReal cb_scale;
		mReal cb_bias;
		mReal cb_mNm_per_mA;
		mReal cb_mA_per_tick;
		int type;
		int tmp_cnt;
		int err;
};

class M3WrenchSensor 
{
	public:
		//M3WrenchSensor() : temp(6),ticks(6), z(6), C(6,6), b(6){}
		M3WrenchSensor() :temp(6),C(36){}
		virtual void Step(mReal ticks_0,mReal ticks_1,mReal ticks_2,
				  mReal ticks_3,mReal ticks_4,mReal ticks_5);
		Wrench * GetWrench(){return & wrench;}
		virtual void ReadConfig(const YAML::Node& doc);
	protected:
		enum{NONE, W_6X6_LINEAR};
		Wrench wrench;
		//VectorXf ticks;
		//VectorXf z;
		//MatrixXf C;
		//VectorXf b;
		//VectorXf temp;
		
		vector<mReal> C;
		vector<mReal> temp;
		
		vector<mReal> cb_fx;
		vector<mReal> cb_fy;
		vector<mReal> cb_fz;
		vector<mReal> cb_tx;
		vector<mReal> cb_ty;
		vector<mReal> cb_tz;
		vector<mReal> cb_adc_bias;
		mReal cb_scale;
		vector<mReal> cb_bias;
		int type;
};



class M3VoltageSensor
{
	public:
		M3VoltageSensor():val(0){}
		virtual void Step(mReal ticks);
		virtual mReal GetVoltage_V(){return val;}
		virtual void ReadConfig(const YAML::Node& doc);
	protected:
		enum {NONE,  ADC_POLY};
		mReal val;
		int type;
		mReal cb_scale;
		mReal cb_bias;
		vector<mReal> cb_voltage;
		vector<mReal> cb_inv_voltage;
};

}

#endif


