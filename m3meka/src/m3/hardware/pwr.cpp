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

#include "m3/hardware/pwr.h"
#include "m3rt/base/m3rt_def.h"
#include "m3rt/base/component_factory.h"


namespace m3{
	
using namespace m3rt;
using namespace std;

				 
///////////////////////////////////////////////////////

void M3Pwr::SetMotorEnable(bool on){command.set_enable_motor(on);}
bool M3Pwr::IsMotorPowerOn(){return status.motor_enabled();}
mReal M3Pwr::GetCurrentDigital(){return status.current_digital();}
mReal M3Pwr::GetBusVoltage(){return status.bus_voltage();}
mReal M3Pwr::GetTimestamp(){return status.base().timestamp();}
mReal M3Pwr::GetBusCurrent(){return status.bus_current();}
		
///////////////////////////////////////////////////////
void M3Pwr::Startup()
{
	if (ecc!=NULL)
		SetStateSafeOp();
	else
		SetStateError();
}

void M3Pwr::Shutdown()
{
	SetMotorEnable(false);
	
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
						  
bool M3Pwr::ReadConfig(const char * filename)
{
	YAML::Node doc;
	if (!M3Component::ReadConfig(filename))
		return false;
	GetYamlDoc(filename, doc);
	doc["ec_component"] >> ecc_name;
	int ival;
	doc["ignore_bounds"] >> ival;
	ignore_bounds = (bool) ival;
	v_sense.ReadConfig(doc["calib"]["voltage"]);
	i_sense.ReadConfig(doc["calib"]["current"]);
	
	if (IsVersion(ISS_BASE))
	{
	  bus_i_sense.ReadConfig(doc["calib"]["bus_current"]);
	}
	
	mReal mval;
	doc["param"]["max_bus_voltage"] >> mval;
	param.set_max_bus_voltage(mval);
	doc["param"]["max_current_digital"] >> mval;
	param.set_max_current_digital(mval);
	
	if (IsVersion(ISS_BASE))
	{
	  doc["param"]["max_bus_current"] >> mval;
	  param.set_max_bus_current(mval);
	  doc["param"]["charge_bus_voltage"] >> mval;
	  param.set_charge_bus_voltage(mval);
	  doc["warn_charge_voltage"] >> ival;
	  warn_charge_voltage = (bool) ival;
	}
	doc["param"]["min_bus_voltage"] >> mval;
	param.set_min_bus_voltage(mval);
	int window=500000;
	int downsample=10;
	voltage_avg.Resize(window,downsample);
	current_avg.Resize(window,downsample);
	bus_current_avg.Resize(window,downsample);
	return true;
}

bool M3Pwr::LinkDependentComponents()
{
	ecc=(M3PwrEc*) factory->GetComponent(ecc_name);
	if (ecc==NULL)
	{
		M3_INFO("M3PwrEc component %s not found for component %s\n",ecc_name.c_str(),GetName().c_str());
		return false;
	}
	return true;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void M3Pwr::StepStatus()
{
	if (IsStateError())
	{
		return;
	}
	M3PwrEcStatus * ec_status = (M3PwrEcStatus * )ecc->GetStatus();
	v_sense.Step(ec_status->adc_bus_voltage());
	i_sense.Step(ec_status->adc_current_digital(),ec_status->adc_current_digital(),0);
	
	if (IsVersion(ISS_BASE))
	{
	   bus_i_sense.Step(ec_status->adc_ext_0(),0,0);
	   status.set_bus_current(bus_current_avg.Step(bus_i_sense.GetCurrent_mA()));
	}
	else
	{
	  status.set_bus_current(0);
	}
	
	
	status.set_bus_voltage(voltage_avg.Step(v_sense.GetVoltage_V()));
	status.set_current_digital(current_avg.Step(i_sense.GetCurrent_mA()));
	
	status.set_motor_enabled(ec_status->motor_enabled());//1);//Temp, undo, testing only. ec_status->motor_enabled());
	if (startup_cnt++>500 && ! ignore_bounds)
	{
		if (GetCurrentDigital()>param.max_current_digital())
		{
			M3_ERR("M3PWR Current Digital of %f exceeds max allowable current of %d.\n",GetCurrentDigital(),param.max_current_digital());
			SetStateError();
		}
		if (GetBusCurrent()>param.max_bus_current())
		{
			M3_ERR("M3PWR Bus Current of %f exceeds max allowable current of %f.\n",GetBusCurrent(),param.max_bus_current());
			SetStateError();
		}
		if (GetBusVoltage()>param.max_bus_voltage() || GetBusVoltage()<param.min_bus_voltage())
		{
			M3_ERR("M3PWR Bus Voltage of %f outside of allowable range of: %f to %f.\n",GetBusVoltage(),param.min_bus_voltage(),param.max_bus_voltage());
			SetStateError();
		}
	}
	ecc->SetEnableBuzzer(0);
	if (warn_charge_voltage)
	{
	  if (GetBusVoltage()<param.charge_bus_voltage())
	  {
	    if (warn_cnt++%2000>1000)
	      ecc->SetEnableBuzzer(1);
	    else
	      ecc->SetEnableBuzzer(0);
	  }
	}
}

void M3Pwr::StepCommand()
{
	if (ecc)
	{
		M3PwrEcCommand * ec_command = (M3PwrEcCommand *)ecc->GetCommand();
		if(IsStateError()||IsStateSafeOp())
			ec_command->set_enable_motor(0);
		else
			ec_command->set_enable_motor((int)command.enable_motor());
	}
}

}