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

#include "m3/hardware/pwr_virtual.h"


namespace m3{
	
using namespace m3rt;
using namespace std;


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////



bool M3PwrVirtual::LinkDependentComponents()
{
	
	return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void M3PwrVirtual::Startup()
{
	
	SetStateSafeOp();
	
}

void M3PwrVirtual::Shutdown()
{
	
}


void M3PwrVirtual::StepStatus()
{	
	if (IsStateError())
		return;
	// ToDo: set values to normal op conditions..
	status.set_bus_voltage(15.);	
	status.set_current_digital(1.);
	status.set_motor_enabled(motor_enabled_virtual);
}


void M3PwrVirtual::StepCommand()
{
	if (IsStateSafeOp())
		return;
	//Update Control
	if(IsStateError())
	{		
		return;
	}
	
	motor_enabled_virtual = command.enable_motor();
}

}