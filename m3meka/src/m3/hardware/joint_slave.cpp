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

#include "m3/hardware/joint_slave.h"


namespace m3{
	
using namespace m3rt;
using namespace std;


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////



bool M3JointSlave::LinkDependentComponents()
{
	
	cpj=(M3Joint*) factory->GetComponent(cpj_name);
	if (cpj==NULL)
	{
		M3_INFO("M3Joint component %s not found for component %s\n",cpj_name.c_str(),GetName().c_str());
		return false;
	}
	return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void M3JointSlave::Startup()
{
	
	SetStateSafeOp();
	
}

void M3JointSlave::Shutdown()
{
	
}

bool M3JointSlave::ReadConfig(const char * filename)
{
	YAML::Node doc;
	if (!M3Component::ReadConfig(filename))
		return false;
	GetYamlDoc(filename, doc);
	doc["cpj_component"] >> cpj_name;
	doc["calib"]["cb_ms_ratio"]>>cb_ms_ratio; //N:1 gearing
	return true;
}

void M3JointSlave::StepStatus()
{	
	if (IsStateError())
		return;
	status.set_amp_temp(0);
	status.set_motor_temp(0);
	status.set_current(0);
	status.set_torque(0);
	status.set_torquedot(0);
	status.set_theta(cpj->GetThetaDeg()/cb_ms_ratio);
	status.set_thetadot(cpj->GetThetaDotDeg()/cb_ms_ratio);
	status.set_thetadotdot(cpj->GetThetaDotDotDeg()/cb_ms_ratio);
}


void M3JointSlave::StepCommand()
{	

}

}