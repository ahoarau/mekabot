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

#ifndef M3_JOINT_SLAVE_H
#define M3_JOINT_SLAVE_H

#include "m3/hardware/joint.h"

namespace m3
{
	using namespace std;

/////////////////////////////////////////////////////////////////////////

//A joint that is directly coupled to another by some ratio(e.g., on torso)
class M3JointSlave : public M3Joint
{
	public:
		M3JointSlave(){}	
	protected:
		void Startup();
		void Shutdown();
		void StepStatus();
		void StepCommand();
		bool LinkDependentComponents();		
		bool ReadConfig(const char * filename);
	private:		
		string cpj_name;
		mReal cb_ms_ratio;
};


}

#endif


