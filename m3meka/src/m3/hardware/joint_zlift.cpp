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

#include "m3/hardware/joint_zlift.h"
#include "math.h"

namespace m3
{
	
using namespace m3rt;
using namespace std;

bool M3JointZLift::ReadConfig(const char * filename)
{	
	if (!M3Joint::ReadConfig(filename))
		return false;
	YAML::Node doc;
	GetYamlDoc(filename, doc);
	doc["calib"]["cb_payload_mass"] >>  cb_payload_mass;
	doc["calib"]["cb_gearing"] >>  cb_gearing;
	doc["calib"]["cb_screw_pitch"] >>  cb_screw_pitch;
	doc["calib"]["cb_screw_efficiency"] >>  cb_screw_efficiency;
	cb_mm_per_deg = cb_screw_pitch/(cb_gearing*360.0);
	cb_mN_per_mNm = 1/(cb_screw_pitch*cb_screw_efficiency/(2.0*M_PI*1000));
	return true;
}
#define MN_PER_KG 9.80665*1000.0
void M3JointZLift::StepCommand()
{
	mReal tq_g_mNm = -1.0*cb_payload_mass*MN_PER_KG/cb_mN_per_mNm;
	SetTorqueGravity(tq_g_mNm);
	if (act->GetActuatorEc()->IsAuxSwitchOn())
	  command.set_q_desired(GetThetaDeg());
	M3Joint::StepCommand();
	
	/*if( tmp_cnt++ == 1000)
	{
	M3_DEBUG("-----------\n");
	M3_DEBUG("theta: %f\n", ((M3JointCommand*)GetCommand())->q_desired());
	M3_DEBUG("slew: %f\n", ((M3JointCommand*)GetCommand())->q_slew_rate());
	M3_DEBUG("stiff: %f\n", ((M3JointCommand*)GetCommand())->q_stiffness());
	M3_DEBUG("mode: %d\n", (int)((M3JointCommand*)GetCommand())->ctrl_mode());  
	M3_DEBUG("-----------\n");
	tmp_cnt = 0;
	}*/
}

}