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

#include "m3/hardware/loadx6_virtual.h"
#include "m3rt/base/m3rt_def.h"
#include "m3rt/base/component_factory.h"


namespace m3{
	
using namespace m3rt;
using namespace std;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////


bool M3LoadX6Virtual::ReadConfig(const char * filename)
{

	return M3LoadX6::ReadConfig(filename);
}

bool M3LoadX6Virtual::LinkDependentComponents()
{
	
	return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void M3LoadX6Virtual::StepStatus()
{	
	status.set_adc_ext_0(0.1);
	status.set_adc_ext_1(0.2);
	status.set_adc_ext_2(0.3);
	status.set_dig_ext_0(0.4);
	
	for (int i=0;i<6;i++)
	  status.set_wrench(i,i);
}


}