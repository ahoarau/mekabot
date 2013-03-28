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

#ifndef M3_ROBOT_H
#define M3_ROBOT_H

#include "m3rt/base/component_factory.h"
#include "m3rt/base/component.h"
#include "m3/hardware/pwr.h"
#include "m3/toolbox/toolbox.h"

namespace m3
{
	using namespace std;
	using namespace KDL;


///////////////////////////////////////////////////////////////////////////
//Empty class for now

class M3Robot : public m3rt::M3Component
{
	public:
		M3Robot() : m3rt::M3Component(ROBOT_PRIORITY)
		{
			RegisterVersion("default",DEFAULT);	//RBL
			RegisterVersion("iss",ISS);		//ISS. Safe as DEFAULT
		}

	protected:
		enum {DEFAULT, ISS};	
		void Startup();
		bool LinkDependentComponents();
		bool ReadConfig(const char * filename);
		Vector grav_world;
		string pwr_name;
		M3Pwr * pwr;
};

}

#endif


