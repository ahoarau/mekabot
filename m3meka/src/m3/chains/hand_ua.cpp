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

#include "m3/chains/hand_ua.h"

namespace m3
{
	
using namespace m3rt;
using namespace std;


void M3HandUA::StepStatus()
{
	if (IsStateError())
		return;
	for (int i=0;i<ndof_finger;i++)
	{
		status.set_theta_index(i, hand->GetThetaDeg(2) * param.flex_factor_index(i));
		status.set_theta_ring(i, hand->GetThetaDeg(3) * param.flex_factor_ring(i));
		status.set_theta_pinky(i, hand->GetThetaDeg(4) * param.flex_factor_pinky(i));
	}
	status.set_theta_thumb(0, hand->GetThetaDeg(0));
	for (int i=1;i<ndof_finger;i++)
	{
		status.set_theta_thumb(i, hand->GetThetaDeg(1) * param.flex_factor_thumb(i-1));
	}
	
}

void M3HandUA::StepCommand()
{	
	if (IsStateSafeOp() || IsStateError())
		return;
	
}
	
bool M3HandUA::LinkDependentComponents()
{	
	hand = (M3Hand*)factory->GetComponent(hand_name);
	if (hand == NULL)
	{
	  M3_ERR("M3Hand %s not found for %s. \n", hand_name.c_str(),GetName().c_str()); 
	  return false;
	}
	  
	
	return true;
}

void M3HandUA::Startup()
{		
	

	for (int i=0;i<ndof_finger;i++)
	{
		
		status.add_theta_thumb(0.0);
		status.add_theta_index(0.0);
		status.add_theta_ring(0.0);
		status.add_theta_pinky(0.0);
	}

	SetStateSafeOp();

}


bool M3HandUA::ReadConfig(const char * filename)
{
	if (!M3Component::ReadConfig(filename))
		return false;
	YAML::Node doc;
	GetYamlDoc(filename, doc);
	
	for(int i = 0; i < ndof_finger; i++)
	{
	  mReal value;
	  doc["param"]["flex_factor"]["index"][i] >> value;
	  param.add_flex_factor_index(value);
	  doc["param"]["flex_factor"]["ring"][i] >> value;
	  param.add_flex_factor_ring(value);
	  doc["param"]["flex_factor"]["pinky"][i] >> value;
	  param.add_flex_factor_pinky(value);
	}
	
	for(int i = 0; i < 2; i++) // thumb is kind of a special case..
	{
	  mReal value;
	  doc["param"]["flex_factor"]["thumb"][i] >> value;
	  param.add_flex_factor_thumb(value);	  
	}
	
	doc["hand_component"] >> hand_name;
	
	return true;
}


void M3HandUA::Shutdown()
{

}


}