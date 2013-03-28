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

#include <m3/hardware/tactile_pps22_ec.h>
#include "m3rt/base/component_factory.h"



namespace m3{
	
using namespace m3rt;
using namespace std;


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
M3BaseStatus * M3TactilePPS22Ec::GetBaseStatus()
{
		return status.mutable_base();
}

void M3TactilePPS22Ec::SetStatusFromPdo(unsigned char * data)
{
	if (IsPdoVersion(TACTX2_PDO_V1))
	{
		M3TactX2PdoV1Status * ec = (M3TactX2PdoV1Status *) data;
		M3TactilePPS22V1Status * pdo;
		pdo=&(ec->tactile[chid]);
		status.set_timestamp(ec->timestamp);
		for (int i=0;i<22;i++)
		{
			status.set_taxels(i, pdo->taxel[i]);
		}
	}
}

void M3TactilePPS22Ec::Startup()
{
	for (int i=0;i<22;i++)
		status.add_taxels(0);
	M3ComponentEc::Startup();
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void M3TactilePPS22Ec::SetPdoFromCommand(unsigned char * data)
{

}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool M3TactilePPS22Ec::LinkDependentComponents()
{
	return true;
}

bool M3TactilePPS22Ec::ReadConfig(const char * filename)
{
	if (!M3ComponentEc::ReadConfig(filename))
		return false;
	YAML::Node doc;
	GetYamlDoc(filename, doc);
	int val;
	doc["chid"] >> val;
	chid = val;
	return false;
}

}
 