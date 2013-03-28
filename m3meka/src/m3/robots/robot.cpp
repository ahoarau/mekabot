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

#include "m3/robots/robot.h"


namespace m3
{
	
using namespace m3rt;
using namespace std;
using namespace KDL;

bool M3Robot::LinkDependentComponents()
{
	if (pwr_name.size()!=0)
	{		
		pwr=(M3Pwr*)factory->GetComponent(pwr_name);
		if (pwr==NULL)
		{
			M3_ERR("M3Pwr component %s could not be linked for M3Robot\n",
					pwr_name.c_str());
			return false;
		}
	}
	return true;
}

bool M3Robot::ReadConfig(const char * filename)
{	
	if (!M3Component::ReadConfig(filename))
		return false;	
	
	YAML::Node doc;
	GetYamlDoc(filename, doc);
	
	doc["pwr_component"] >> pwr_name;

	return true;	

	return true;
}

void M3Robot::Startup()
{
	grav_world = Vector(0,0,1.)*GRAV;
}

}