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

#include <m3/hardware/loadx1_ec.h>
#include "m3rt/base/component_factory.h"
#include "m3rt/base/m3ec_def.h"
#include "m3/hardware/m3ec_pdo_v1_def.h"
namespace m3{
	
using namespace m3rt;
using namespace std;


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
M3BaseStatus * M3LoadX1Ec::GetBaseStatus()
{
	return status.mutable_base();
}

//If piggybacks on ActuatorEc, then do nothing
void M3LoadX1Ec::ResetCommandPdo(unsigned char * pdo)
{
	if (IsPdoVersion(ACTX1_PDO_V1)) return;
}


size_t M3LoadX1Ec::GetStatusPdoSize()
{
	if (IsPdoVersion(ACTX1_PDO_V1)) return sizeof(M3ActX1PdoV1Status);
	return 0;
}

size_t M3LoadX1Ec::GetCommandPdoSize()
{
	if (IsPdoVersion(ACTX1_PDO_V1)) return sizeof(M3ActX1PdoV1Cmd);
	return 0;
}

void M3LoadX1Ec::SetStatusFromPdo(unsigned char * data)
{
	if (IsPdoVersion(ACTX1_PDO_V1))
	{
		M3ActX1PdoV1Status * ec = (M3ActX1PdoV1Status *) data;
		status.set_timestamp(ec->timestamp);
		status.set_adc_torque(ec->status[chid].adc_ext_a);
		status.set_flags(ec->status[chid].flags);
	} 

}

void M3LoadX1Ec::SetPdoFromCommand(unsigned char * data){}


bool M3LoadX1Ec::LinkDependentComponents()
{
	return true;
}

bool M3LoadX1Ec::ReadConfig(const char * filename)
{
	YAML::Node doc;
	if (!M3ComponentEc::ReadConfig(filename))
		return false;
	if (IsPdoVersion(ACTX1_PDO_V1))
	{
		chid=0;
		return true;
	}
	return false;
}

}
   
