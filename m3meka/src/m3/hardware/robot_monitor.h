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

#ifndef M3_ROBOT_MONITOR_H
#define M3_ROBOT_MONITOR_H

#include "m3rt/base/component.h"
#include "m3/toolbox/toolbox.h"
#include "m3/toolbox/dfilter.h"
#include "m3/hardware/pwr.h"
#include "m3/hardware/robot_monitor.pb.h"
#include "m3/hardware/actuator.h"
#include <google/protobuf/message.h>

namespace m3
{
	using namespace std;
	
	
class M3RobotMonitor : public m3rt::M3Component
{
	public:
		M3RobotMonitor(): m3rt::M3Component(ROBOT_CTRL_PRIORITY),startup_cnt(0)
		{
			RegisterVersion("default",DEFAULT);	//RBL
			RegisterVersion("iss",ISS);		//ISS. No change from DEFAULT
		}
		google::protobuf::Message * GetCommand(){return &command;}
		google::protobuf::Message * GetStatus(){return &status;}
		google::protobuf::Message * GetParam(){return &param;}	
		
	protected:
		enum {DEFAULT,ISS};
		bool ReadConfig(const char * filename);
		void Startup();
		void Shutdown();
		void StepStatus();
		void StepCommand();
		bool LinkDependentComponents();
		M3RobotMonitorStatus status;
		M3RobotMonitorCommand command;
		M3RobotMonitorParam param;
		M3BaseStatus * GetBaseStatus(){return status.mutable_base();}
	private:
		// NOTE: For now hardcode pwr and actuator components
		// since those are the only types that we need.
		// In future if needed use switch statement with
		// GetCompType to support different component types that have similar sensors
		vector<M3Pwr *> volt_comps;
		vector<M3Actuator *> temp_comps;
		vector<string> volt_names;
		vector<string> temp_names;
		int startup_cnt;
		
};


} // m3 namespace

#endif


