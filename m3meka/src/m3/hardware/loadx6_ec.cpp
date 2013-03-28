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

#include <m3/hardware/loadx6_ec.h>


namespace m3{
	
using namespace m3rt;
using namespace std;

///////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void M3LoadX6Ec::SetStatusFromPdo(unsigned char * data)
{
	if (IsPdoVersion(LOADX6_PDO_V1))
	{
		M3LoadX6PdoV1Status * ec = (M3LoadX6PdoV1Status *) data;
		status.set_timestamp(ec->timestamp);
		status.set_adc_ext_0(ec->adc_ext_0);
		status.set_adc_ext_1(ec->adc_ext_1);
		status.set_adc_ext_2(ec->adc_ext_2);
		status.set_adc_load_0(ec->adc_load_0);
		status.set_adc_load_1(ec->adc_load_1);
		status.set_adc_load_2(ec->adc_load_2);
		status.set_adc_load_3(ec->adc_load_3);
		status.set_adc_load_4(ec->adc_load_4);
		status.set_adc_load_5(ec->adc_load_5);
		status.set_dig_ext_0(ec->dig_ext_0);
		status.set_flags(ec->flags);
	}
	if (IsPdoVersion(LOADX6_PDO_V0))
	{
		M3LoadX6PdoV0Status * ec = (M3LoadX6PdoV0Status *) data;
		status.set_timestamp(GetBaseStatus()->timestamp());//Not provided by DSP for this version
		status.set_adc_ext_0(ec->adc_ext_0);
		status.set_adc_ext_1(ec->adc_ext_1);
		status.set_adc_ext_2(ec->adc_ext_2);
		status.set_adc_load_0(ec->adc_load_0);
		status.set_adc_load_1(ec->adc_load_1);
		status.set_adc_load_2(ec->adc_load_2);
		status.set_adc_load_3(ec->adc_load_3);
		status.set_adc_load_4(ec->adc_load_4);
		status.set_adc_load_5(ec->adc_load_5);
		status.set_dig_ext_0(ec->dig_ext_0);
		status.set_flags(ec->flags);
	}
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void M3LoadX6Ec::SetPdoFromCommand(unsigned char * data)
{
	if (IsPdoVersion(LOADX6_PDO_V1))
	{
		M3LoadX6PdoV1Cmd * ec = (M3LoadX6PdoV1Cmd *) data;
		ec->config=param.config();
	}
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool M3LoadX6Ec::ReadConfig(const char * filename)
{
	if (!M3ComponentEc::ReadConfig(filename))
		return false;
	YAML::Node doc;
	GetYamlDoc(filename, doc);
	int val;
	doc["param"]["config"] >> val;
	param.set_config(val);
	return true;
}

}
   
