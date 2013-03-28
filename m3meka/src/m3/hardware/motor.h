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

#ifndef M3_MOTOR_H
#define M3_MOTOR_H

#include "m3/toolbox/toolbox.h"
#include "m3rt/base/m3ec_def.h"


USING_PART_OF_NAMESPACE_EIGEN

namespace m3
{
using namespace std;


class M3MotorModel 
{
	public:
	      //EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	      M3MotorModel():i_scale(1.0),safe_thermal_pct(1.0),safe_pwm_pct(0.9),starting_current(0),power_elec(0),power_heat(0),power_mech(0),winding_temp(0),case_temp(0),ambient_temp(0),i_rms(0),i_cont(0),v_cemf(0),v_pwm(0),safe_pwm_duty(0),first_wt_step(5),model_type(NONE){}
		//i: current sensor if present (unfiltered, mA)
		//pwm: current duty cycle to motor (-pwm_max to pwm_max)
		//rpm: motor velocity
		//temp: temp sensor if present (case or ambient, unfilterd, C)
		//config files specify which models to used depending on avaialble sensors
		void Step(mReal i, mReal pwm, mReal rpm, mReal tmp);
		mReal GetWindingTemp(){return winding_temp;}
		mReal GetCaseTemp(){return case_temp;}
		mReal GetAmbientTemp(){return ambient_temp;}
		mReal GetMaxWindingTemp(){return max_winding_temp;}
		mReal GetCurrentRMS(){return i_rms;}
		mReal GetCurrentContinuous(){return i_cont;}
		mReal GetPowerHeat(){return power_heat;}
		mReal GetPowerMech(){return power_mech;}
		mReal GetPowerElec(){return power_elec;}
		mReal GetMaxCurrent(){return nominal_current*1000.0;} //Continuous, mA
		mReal GetSafePwmDuty(){return safe_pwm_duty;}
		mReal mNmToPwm(mReal mNm);
		virtual void ReadConfig(const YAML::Node & doc, string config_filename);		
		void ThermalShutdown(string config_filename, mReal ambient_temp);
	protected:
		void ThermalInit(string config_filename);	
		void StepModelV0(mReal i, mReal pwm, mReal rpm, mReal tmp);
		void StepModelV1(mReal i, mReal pwm, mReal rpm, mReal tmp);
		void StepModelV2(mReal i, mReal pwm, mReal rpm, mReal tmp);
		void StepVoltageLimit();
		string name;
		//MODEL_V0 is for V0 actuator config files. Full motor parameters not present in config file. (RBL)
		//MODEL_V1 is for V1 actuator config files. Full motor parameters present
		//MODEL_V2 is for V2 actautor config files. Current sensor and ambient temp sensor present.
		enum {NONE, MODEL_V0, MODEL_V1, MODEL_V2}; //model typels
		enum {CURRENT_NONE, CURRENT_MEASURED, CURRENT_CONTROLLED};//current sensor types
		enum {TEMP_NONE, TEMP_CASE, TEMP_AMBIENT};//temp sensor types
		mReal nominal_voltage;//V
		mReal no_load_speed;//rpm
		mReal no_load_current;//mA
		mReal nominal_speed;//rpm
		mReal nominal_torque;//mNm
		mReal nominal_current;//A
		mReal stall_torque;//mNm
		mReal starting_current;//A
		mReal max_efficiency;//%
		mReal winding_resistance;//Ohm
		mReal winding_inductance;//mH
		mReal torque_constant;//mNm/A
		mReal speed_constant;//rpm/V
		mReal speed_torque_gradient;//rpm/mNm
		mReal mechanical_time_constant;//ms
		mReal rotor_inertia;//gcm^2
		mReal thermal_resistance_housing_ambient;//K/W
		mReal thermal_resistance_rotor_housing;//K/W
		mReal thermal_time_constant_winding;//s (Seconds to achieve 63% of steady-state temp)
		mReal thermal_time_constant_motor;//s
		mReal max_winding_temp;//C
		mReal gear_ratio;//N:1
		mReal amplifier_resistance;//Ohm
		mReal max_pwm_duty;
		mReal alpha_cu;
		mReal winding_temp;
		mReal case_temp;
		mReal ambient_temp;
		mReal power_heat;
		mReal power_mech;
		mReal power_elec;
		mReal i_rms;
		mReal i_cont;
		mReal v_cemf;
		mReal v_pwm;
		mReal safe_pwm_duty;
		mReal safe_thermal_pct;
		mReal safe_pwm_pct;
		mReal i_scale;
		int temp_sensor_type;
		int current_sensor_type;
		int model_type;
		M3ExponentialAvg winding_temp_avg;
		M3ExponentialAvg case_temp_avg;
		M3TimeSlew safe_pwm_slew;
		M3TimeAvg tmp_avg;
		M3TimeAvg curr_avg_rms;
		M3TimeAvg curr_avg_cont;
		int first_wt_step;
		int tmp_cnt;
		mReal previous_winding_temp;
		mReal previous_case_temp;
		string previous_temp_timestamp;
		MatrixXf eA1;
		VectorXf eA2, Tprev;
		bool use_old_temp_values;
};

}

#endif


