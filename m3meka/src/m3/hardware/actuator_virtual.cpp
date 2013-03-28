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

#include "m3/hardware/actuator_virtual.h"
#include "m3rt/base/m3rt_def.h"
#include "m3rt/base/component_factory.h"


namespace m3{
	
using namespace m3rt;
using namespace std;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void M3ActuatorVirtual::Startup()
{
	SetStateSafeOp();
}

/*void M3ActuatorVirtual::Shutdown()
{

}*/

bool M3ActuatorVirtual::ReadConfig(const char * filename)
{
	int val;
	YAML::Node doc;
	if (!M3Actuator::ReadConfig(filename))
		return false;
	GetYamlDoc(filename, doc);
	doc["joint_component"] >> jnt_name;	
	return true;
}

bool M3ActuatorVirtual::LinkDependentComponents()
{
	joint=(M3Joint*) factory->GetComponent(jnt_name);
	if (joint==NULL)
	{
		M3_INFO("M3Joint component %s not found for component %s. Proceeding without it.\n",jnt_name.c_str(),GetName().c_str());
		return false;
	}
	return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void M3ActuatorVirtual::StepStatus()
{
	pnt_cnt++;
	
	if (IsStateError())
		return;
	status.set_torque(command.tq_desired());
	status.set_torquedot(torquedot_df.Step(GetTorque()));
	status.set_flags(ACTUATOR_EC_FLAG_QEI_CALIBRATED);


	if (joint != NULL)
	{
		M3Transmission * t=joint->GetTransmission();
		if (t!=NULL)
		{

			angle_df.Step(t->GetThetaDesJointDeg(),0); //Note: should be GetThetaDesSensorDeg, not working. this OK so long as all angle sensors are collocated 1:1
			status.set_theta(angle_df.GetTheta());
			//status.set_theta(160);
			status.set_thetadot(angle_df.GetThetaDot());
			status.set_thetadotdot(angle_df.GetThetaDotDot());
		}
		else
			M3_INFO("No transmission found for %s\n",joint->GetName().c_str());
	}
	//status.set_amp_temp(25.0);
	//status.set_current(0.0);
	//status.set_motor_temp(25.0);
}

void M3ActuatorVirtual::StepCommand()
{

}

}
