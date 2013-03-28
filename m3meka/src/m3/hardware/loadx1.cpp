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

#include "m3/hardware/loadx1.h"
#include "m3rt/base/m3rt_def.h"
#include "m3rt/base/component_factory.h"


namespace m3{
	
using namespace m3rt;
using namespace std;

///////////////////////////////////////////////////////

void M3LoadX1::Startup()
{
	if (ecc!=NULL)
		SetStateSafeOp();
	else
		SetStateError();
}

void M3LoadX1::Shutdown()
{

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool M3LoadX1::ReadConfig(const char * filename)
{
	int val;
	YAML::Node doc;
	if (!M3Component::ReadConfig(filename))
		return false;
	GetYamlDoc(filename, doc);
	doc["ec_component"] >> ecc_name;
	tq_sense.ReadConfig(doc["calib"]["torque"]);
	torquedot_df.ReadConfig(doc["calib"]["torquedot_df"]);
	return true;
}

bool M3LoadX1::LinkDependentComponents()
{
	ecc=(M3LoadX1Ec*) factory->GetComponent(ecc_name);
	if (ecc==NULL)
	{
		M3_INFO("M3LoadX1Ec component %s not found for component %s\n",ecc_name.c_str(),GetName().c_str());
		return false;
	}
	return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void M3LoadX1::StepStatus()
{
	if (IsStateError())
		return;
	M3LoadX1EcStatus * ecs = (M3LoadX1EcStatus * )(ecc->GetStatus());
	tq_sense.Step(ecs->adc_torque());
	status.set_torque(tq_sense.GetTorque_mNm());
	status.set_torquedot(torquedot_df.Step(GetTorque()));
}


void M3LoadX1::StepCommand(){}

}