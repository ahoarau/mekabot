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

#ifndef M3_PWR_H
#define M3_PWR_H

#include "m3rt/base/component.h"
#include "m3/toolbox/toolbox.h"
#include "m3/hardware/pwr.pb.h"
#include "m3/hardware/pwr_ec.pb.h"
#include "m3/hardware/pwr_ec.h"
#include "m3/hardware/sensor.h"
#include <google/protobuf/message.h>

namespace m3
{
	using namespace std;
	
class M3Pwr : public m3rt::M3Component
{
	public:
		M3Pwr(): m3rt::M3Component(CALIB_PRIORITY),ecc(NULL),startup_cnt(0),ignore_bounds(false),warn_charge_voltage(false)
		{
			RegisterVersion("default",DEFAULT);	//RBL
			RegisterVersion("iss",ISS);		//ISS. No change from DEFAULT
			RegisterVersion("iss_base",ISS_BASE);	//ISS Base. Added Bus Current Sensing
		}
		google::protobuf::Message * GetCommand(){return &command;}
		google::protobuf::Message * GetStatus(){return &status;}
		google::protobuf::Message * GetParam(){return &param;}
	public:
		//Command
		void SetMotorEnable(bool on);
		//Status
		bool IsMotorPowerOn();
		mReal GetCurrentDigital();
		mReal GetBusCurrent();
		mReal GetBusVoltage();
		mReal GetTimestamp();
	protected:
		enum {DEFAULT,ISS,ISS_BASE};
		bool ReadConfig(const char * filename);
		void Startup();
		void Shutdown();
		void StepStatus();
		void StepCommand();
		bool LinkDependentComponents();
		M3PwrStatus status;
		M3PwrCommand command;
		M3PwrParam param;
		M3BaseStatus * GetBaseStatus(){return status.mutable_base();}
	private:
		string ecc_name;
		M3PwrEc * ecc; 	//Corresponding EtherCAT component 
		M3CurrentSensor i_sense;
		M3CurrentSensor bus_i_sense;
		M3VoltageSensor v_sense;
		M3TimeAvg voltage_avg;
		M3TimeAvg current_avg;
		M3TimeAvg bus_current_avg;
		int startup_cnt;
		int tmp_cnt;
		int warn_cnt;
		bool ignore_bounds,warn_charge_voltage;
};


}

#endif


