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

#ifndef MONITOR_H
#define MONITOR_H

#include "m3rt/base/component.h"
#include "m3rt/base/component_base.pb.h" 

namespace m3
{
using namespace std;

class M3Monitor : public m3rt::M3Component
{
	public:
		M3Monitor(): m3rt::M3Component()
		{
			RegisterVersion("default",DEFAULT);	//RBL
			RegisterVersion("iss",ISS);		//ISS. Same as DEFAULT
		}
		google::protobuf::Message * GetCommand(){return &command;}
		//Use M3RtSystem member M3MonitorStatus. It updates this msg, not M3Monitor
		google::protobuf::Message * GetStatus();
		google::protobuf::Message * GetParam(){return &param;}
	protected:
		enum {DEFAULT, ISS};
		bool ReadConfig(const char * filename);
		void Startup();
		void Shutdown();
		void StepStatus();
		void StepCommand();
	protected:
		M3MonitorCommand command;
		M3MonitorParam   param;
		M3BaseStatus * GetBaseStatus();
};
}
#endif


