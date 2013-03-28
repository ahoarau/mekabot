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

#ifndef M3_ZLIFT_H
#define M3_ZLIFT_H

#include "m3/hardware/joint.h"


namespace m3
{
	using namespace std;
///////////////////////////////////////////////////////////////////////////
// A single DOF prismatic joint lifting against gravity
class M3JointZLift : public M3Joint
{
	public:
		M3JointZLift():cb_payload_mass(0),cb_screw_pitch(0),cb_gearing(0),
			cb_screw_efficiency(0),cb_mN_per_mNm(1.0),cb_mm_per_deg(1.0),M3Joint(){}
		void SetDesiredPos(mReal val){command.set_q_desired(val/cb_mm_per_deg);} //mm
		void SetDesiredPosDot(mReal val){command.set_qdot_desired(val/cb_mm_per_deg);} //mm/s
		mReal GetForce(){return status.torque()*cb_mN_per_mNm;} //mN
		mReal GetForceDot(){return status.torquedot()*cb_mN_per_mNm;} //mN/s
		mReal GetForceGravity(){return status.torque_gravity()*cb_mN_per_mNm;} //mN
		mReal GetPos(){return status.theta()*cb_mm_per_deg;} //mm
		mReal GetPosDot(){return status.thetadot()*cb_mm_per_deg;} //mm/s
	protected:
		bool ReadConfig(const char * filename);
		void StepCommand();
	private:
		mReal cb_payload_mass; //Kg
		mReal cb_mN_per_mNm; //convert force at lift to torque at actuator
		mReal cb_mm_per_deg; //convert translation of lift to rotation of actuator
		mReal cb_gearing; //actuator to ball-screw N:1 reduction
		mReal cb_screw_pitch; //mm_per_rev
		mReal cb_screw_efficiency; //0.9 typical
};

}

#endif


