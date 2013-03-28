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

#include <m3/hardware/ledx2_ec.h>

namespace m3{
	
using namespace m3rt;
using namespace std;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void M3LedX2Ec::SetStatusFromPdo(unsigned char * data)
{
	if (IsPdoVersion(LEDX2_PDO_V1))
	{
		M3LedX2PdoV1Status * ec = (M3LedX2PdoV1Status *) data;
		status.set_timestamp(ec->timestamp);
		status.set_adc_ext_a(ec->adc_ext_a);
		status.set_adc_ext_b(ec->adc_ext_b);
		status.set_adc_ext_c(ec->adc_ext_c);
		status.set_adc_ext_d(ec->adc_ext_d);
		status.set_flags(ec->flags);
	}
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void M3LedX2Ec::SetPdoFromCommand(unsigned char * data)
{
	if (IsPdoVersion(LEDX2_PDO_V1))
	{
		M3LedX2PdoV1Cmd * ec = (M3LedX2PdoV1Cmd *) data;
		ec->config=param.config();
		ec->enable_a=command.enable_a();
		ec->enable_b=command.enable_b();
		
		ec->branch_a.board_a.r=(int16_t)command.branch_a().board_a().r();
		ec->branch_a.board_a.g=(int16_t)command.branch_a().board_a().g();
		ec->branch_a.board_a.b=(int16_t)command.branch_a().board_a().b();
		
		ec->branch_a.board_b.r=(int16_t)command.branch_a().board_b().r();
		ec->branch_a.board_b.g=(int16_t)command.branch_a().board_b().g();
		ec->branch_a.board_b.b=(int16_t)command.branch_a().board_b().b();
		
		ec->branch_b.board_a.r=(int16_t)command.branch_b().board_a().r();
		ec->branch_b.board_a.g=(int16_t)command.branch_b().board_a().g();
		ec->branch_b.board_a.b=(int16_t)command.branch_b().board_a().b();
		
		ec->branch_b.board_b.r=(int16_t)command.branch_b().board_b().r();
		ec->branch_b.board_b.g=(int16_t)command.branch_b().board_b().g();
		ec->branch_b.board_b.b=(int16_t)command.branch_b().board_b().b();
	}
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool M3LedX2Ec::ReadConfig(const char * filename)
{
	YAML::Node doc;
	GetYamlDoc(filename, doc);
	if (!M3ComponentEc::ReadConfig(filename))
		return false;
	int val;
	doc["param"]["config"] >> val;
	param.set_config(val);

	return false;
}

}
   
