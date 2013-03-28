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

#include <m3/hardware/pwr_ec.h>
#include "m3rt/base/m3ec_def.h"
#include "m3/hardware/m3ec_pdo_v0_def.h"
#include "m3/hardware/m3ec_pdo_v1_def.h"
#include "m3/hardware/m3ec_pdo_v2_def.h"

namespace m3{
	
using namespace m3rt;
using namespace std;


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void M3PwrEc::SetStatusFromPdo(unsigned char * data)
{
	
	if (IsPdoVersion(PWR_PDO_V0))
	{
		M3PwrPdoV0Status * ec = (M3PwrPdoV0Status *) data;
		status.set_timestamp(GetBaseStatus()->timestamp());//Not provided by DSP for this version
		status.set_motor_enabled(ec->motor_enabled);
		status.set_adc_bus_voltage(ec->adc_bus_voltage);
		status.set_adc_current_digital(ec->adc_current_digital);
		status.set_adc_ext_0(ec->adc_ext);
		status.set_flags(ec->flags);
	}
	if (IsPdoVersion(PWR_PDO_V1))
	{
		M3PwrPdoV1Status * ec = (M3PwrPdoV1Status *) data;
		status.set_timestamp(ec->timestamp);
		status.set_motor_enabled(ec->motor_enabled);
		status.set_adc_bus_voltage(ec->adc_bus_voltage);
		status.set_adc_current_digital(ec->adc_current_digital);
		status.set_adc_ext_0(ec->adc_ext);
		status.set_flags(ec->flags);
	}
	if (IsPdoVersion(PWR_PDO_V2))
	{
		M3PwrPdoV2Status * ec = (M3PwrPdoV2Status *) data;
		status.set_timestamp(ec->timestamp);
		status.set_motor_enabled(ec->motor_enabled);
		status.set_adc_bus_voltage(ec->adc_bus_voltage);
		status.set_adc_current_digital(ec->adc_current_digital);
		status.set_adc_ext_0(ec->adc_ext);
		status.set_flags(ec->flags);
	}
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

size_t M3PwrEc::GetStatusPdoSize()
{
	if (IsPdoVersion(PWR_PDO_V0)) return sizeof(M3PwrPdoV0Status);
	if (IsPdoVersion(PWR_PDO_V1)) return sizeof(M3PwrPdoV1Status);
	if (IsPdoVersion(PWR_PDO_V2)) return sizeof(M3PwrPdoV2Status);
	return 0;
}

size_t M3PwrEc::GetCommandPdoSize()
{
	if (IsPdoVersion(PWR_PDO_V0)) return sizeof(M3PwrPdoV0Cmd);
	if (IsPdoVersion(PWR_PDO_V1)) return sizeof(M3PwrPdoV1Cmd);
	if (IsPdoVersion(PWR_PDO_V2)) return sizeof(M3PwrPdoV2Cmd);
	return 0;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void M3PwrEc::SetPdoFromCommand(unsigned char * data)
{
	if (IsPdoVersion(PWR_PDO_V0))
	{
		M3PwrPdoV0Cmd * ec = (M3PwrPdoV0Cmd *) data;
		ec->config=param.config();
		ec->enable_motor=command.enable_motor();
	}
	if (IsPdoVersion(PWR_PDO_V1))
	{
		M3PwrPdoV1Cmd * ec = (M3PwrPdoV1Cmd *) data;
		ec->config=param.config();
		ec->enable_motor=command.enable_motor();
	}
	if (IsPdoVersion(PWR_PDO_V2))
	{
		M3PwrPdoV2Cmd * ec = (M3PwrPdoV2Cmd *) data;
		ec->config=param.config();
		ec->enable_motor=command.enable_motor();
		//Set buzzer if present
		if (enable_buzzer)
			ec->config=ec->config | M3PWR_CONFIG_BUZZER; 
		else
			ec->config=ec->config & ~M3PWR_CONFIG_BUZZER; 
		//toggle bit to give DSP a heartbeat signal
		if (toggle == 0)
		  toggle = M3ACT_CONFIG_EC_WD;
		else if (toggle != 0)
		{
		  ec->config = ec->config | toggle;
		  toggle = 0;
		}
	}
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool M3PwrEc::ReadConfig(const char * filename)
{
	YAML::Node doc;
	int val;
	if (!M3ComponentEc::ReadConfig(filename)) return false;
	GetYamlDoc(filename, doc);
	doc["param"]["config"] >> val;
	param.set_config(val);
	return true;
}

}
   
