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

#include "m3/toolbox/monitor.h"
#include "m3rt/base/component_factory.h"
#include "m3rt/base/toolbox.h"

namespace m3{
using namespace std;
using namespace m3rt;


void M3Monitor::Startup(){SetStateSafeOp();}
void M3Monitor::Shutdown(){}

google::protobuf::Message * M3Monitor::GetStatus(){return factory->GetMonitorStatus();}
M3BaseStatus * M3Monitor::GetBaseStatus(){return factory->GetMonitorStatus()->mutable_base();}
		
bool M3Monitor::ReadConfig(const char * filename)
{
	YAML::Node doc;
	GetYamlDoc(filename, doc);
	
	if (!M3Component::ReadConfig(filename))
		return false;

	return true;
}

void M3Monitor::StepStatus()
{
	//Don't do anything as RtSystem will update data.
}

void M3Monitor::StepCommand(){}

}