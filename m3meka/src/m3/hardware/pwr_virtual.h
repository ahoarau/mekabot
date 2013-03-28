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

#ifndef M3_POWER_VIRTUAL_H
#define M3_POWER_VIRTUAL_H

#include "m3/hardware/pwr.h"

namespace m3
{
	using namespace std;

/////////////////////////////////////////////////////////////////////////


//Joint space controllers and interface to an SEA driven joint
class M3PwrVirtual : public M3Pwr
{
	public:
		M3PwrVirtual(): motor_enabled_virtual(false){}
		//API
			
	protected:
		void Startup();
		void Shutdown();
		void StepStatus();
		void StepCommand();
		virtual bool LinkDependentComponents();		
		bool motor_enabled_virtual;			
};


}

#endif


