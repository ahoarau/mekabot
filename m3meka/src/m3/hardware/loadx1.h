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

#ifndef M3_LOADX1_H
#define M3_LOADX1_H

#include "m3rt/base/component.h"
#include "m3/hardware/loadx1.pb.h"
#include <m3/hardware/loadx1_ec.pb.h>
#include "m3/hardware/loadx1_ec.h"
#include "m3/hardware/sensor.h"
#include "m3/toolbox/toolbox.h"
#include "m3/toolbox/dfilter.h"
#include <google/protobuf/message.h>

namespace m3
{
	using namespace std;

class M3LoadX1 : public m3rt::M3Component
{
	public:
		M3LoadX1(): m3rt::M3Component(CALIB_PRIORITY),ecc(NULL)
		{
			RegisterVersion("default",DEFAULT);	//RBL
			RegisterVersion("iss",ISS);		//ISS. No change from DEFAULT
		}
		google::protobuf::Message * GetCommand(){return &command;}
		google::protobuf::Message * GetStatus(){return &status;}
		google::protobuf::Message * GetParam(){return &param;}
	public:
		int64_t GetTimestamp(){return status.base().timestamp();}
		mReal GetTorque(){return status.torque();}
		mReal GetTorqueDot(){return status.torquedot();}
	protected:
		enum {DEFAULT,ISS};
		M3TorqueSensor tq_sense;
		M3DFilter torquedot_df;
		
		bool ReadConfig(const char * filename);
		void Startup();
		void Shutdown();
		void StepStatus();
		void StepCommand();
		M3LoadX1Status status;
		M3LoadX1Command command;
		M3LoadX1Param param;
		M3BaseStatus * GetBaseStatus(){return status.mutable_base();}
		bool LinkDependentComponents();
		string ecc_name;
		M3LoadX1Ec * ecc; 	//Corresponding EtherCAT component 
		int tmp_cnt;
};


}

#endif


