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

#include <m3/hardware/ledx2xn_ec.h>

namespace m3{
	
using namespace m3rt;
using namespace std;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void M3LedX2XNEc::SetStatusFromPdo(unsigned char * data)
{
	if (IsPdoVersion(LEDX2XN_PDO_V1))
	{
		M3LedX2XNPdoV1Status * ec = (M3LedX2XNPdoV1Status *) data;
		status.set_timestamp(ec->timestamp);
		status.set_flags(ec->flags);
		status.set_adc_ext_a(ec->adc_ext_a);
		status.set_adc_ext_b(ec->adc_ext_b);
		status.set_adc_ext_c(ec->adc_ext_c);
		status.set_adc_ext_d(ec->adc_ext_d);
	}
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void M3LedX2XNEc::SetPdoFromCommand(unsigned char * data)
{
	if (IsPdoVersion(LEDX2XN_PDO_V1))
	{
		M3LedX2XNPdoV1Cmd * ec = (M3LedX2XNPdoV1Cmd *) data;
		ec->config=param.config();
		ec->enable[0]=command.enable_a();
		ec->enable[1]=command.enable_b();
		if (n_branch_a)
		{
		    ec->branch[0].r=(int16_t)command.branch_a().r(id_a);
		    ec->branch[0].g=(int16_t)command.branch_a().g(id_a);
		    ec->branch[0].b=(int16_t)command.branch_a().b(id_a);
		    ec->branch[0].id=id_a;
		    id_a=(id_a+1)%n_branch_a;
		}
		if (n_branch_b)
		{
		    ec->branch[1].r=(int16_t)command.branch_b().r(id_b);
		    ec->branch[1].g=(int16_t)command.branch_b().g(id_b);
		    ec->branch[1].b=(int16_t)command.branch_b().b(id_b);
		    ec->branch[1].id=id_b;
		    id_b=(id_b+1)%n_branch_b;
		}
		//if (tmp_cnt++%100==0)
		//	M3_INFO("A: %d %d %d %d B: %d %d %d %d\n",
		//		id_a, ec->branch[0].r,ec->branch[0].g,ec->branch[0].b,
    		//		id_b, ec->branch[1].r,ec->branch[1].g,ec->branch[1].b);
	}
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool M3LedX2XNEc::ReadConfig(const char * filename)
{
	YAML::Node doc;
	GetYamlDoc(filename, doc);
	if (!M3ComponentEc::ReadConfig(filename))
		return false;
	int val;
	doc["param"]["config"] >> val;
	doc["param"]["n_branch_a"] >> n_branch_a;
	doc["param"]["n_branch_b"] >> n_branch_b;
	param.set_config(val);
	return true;
}

void M3LedX2XNEc::Startup()
{
	int i;
	M3ComponentEc::Startup();
	for (i=0;i<n_branch_a;i++)
	{
		M3LedX2XNEcRGB * q=command.mutable_branch_a();
		q->add_r(0);
		q->add_g(0);
		q->add_b(0);
	}
	for (i=0;i<n_branch_b;i++)
	{
		M3LedX2XNEcRGB * q=command.mutable_branch_b();
		q->add_r(0);
		q->add_g(0);
		q->add_b(0);
	}
}

}
   
