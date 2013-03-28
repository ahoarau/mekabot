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

#include "m3/hardware/loadx6.h"
#include "m3rt/base/m3rt_def.h"
#include "m3rt/base/component_factory.h"


namespace m3{
	
using namespace m3rt;
using namespace std;


		
///////////////////////////////////////////////////////
void M3LoadX6::Startup()
{
	
	for (int i=0;i<6;i++)
		status.add_wrench(0);
	//if (ecc!=NULL)
		SetStateSafeOp();
	//else
	//	SetStateError();
}

void M3LoadX6::Shutdown(){}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			  
bool M3LoadX6::ReadConfig(const char * filename)
{
	if (!M3Component::ReadConfig(filename))
		return false;
	YAML::Node doc;
	GetYamlDoc(filename, doc);
	doc["ec_component"] >> ecc_name;
	w_sense.ReadConfig(doc["calib"]["wrench"]);
	for (int i=0;i<6;i++)
		wrench_df[i].ReadConfig(doc["calib"]["wrench_df"]);
	return true;
}

bool M3LoadX6::LinkDependentComponents()
{
	ecc=(M3LoadX6Ec*) factory->GetComponent(ecc_name);
	if (ecc==NULL)
	{
		M3_INFO("M3LoadX6Ec component %s not found for component %s.\n",ecc_name.c_str(),GetName().c_str());
		return false;
	}
	return true;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void M3LoadX6::StepStatus()
{
	if (IsStateError())
		return;
	if (ecc!=NULL)
	{
		M3LoadX6EcStatus * ec_status=(M3LoadX6EcStatus *)ecc->GetStatus();
		status.set_adc_ext_0(ec_status->adc_ext_0());
		status.set_adc_ext_1(ec_status->adc_ext_1());
		status.set_adc_ext_2(ec_status->adc_ext_2());
		status.set_dig_ext_0(ec_status->dig_ext_0());
		
		mReal a0=wrench_df[0].Step((mReal)ec_status->adc_load_0());
		mReal a1=wrench_df[1].Step((mReal)ec_status->adc_load_1());
		mReal a2=wrench_df[2].Step((mReal)ec_status->adc_load_2());
		mReal a3=wrench_df[3].Step((mReal)ec_status->adc_load_3());
		mReal a4=wrench_df[4].Step((mReal)ec_status->adc_load_4());
		mReal a5=wrench_df[5].Step((mReal)ec_status->adc_load_5());

		w_sense.Step(a0,a1,a2,a3,a4,a5);
		Wrench * w=w_sense.GetWrench();
		for (int i=0;i<6;i++)
			status.set_wrench(i,(*w)[i]);
	}
	else
	{
		for (int i=0;i<6;i++)
			status.set_wrench(i,0);
	}
}
mReal M3LoadX6::GetWrench(int idx)
{
  
  if (idx<0 ||idx>=6)
    return 0.0;
  return status.wrench(idx);
}

void M3LoadX6::StepCommand(){}



}