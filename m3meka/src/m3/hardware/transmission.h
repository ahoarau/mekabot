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

#ifndef M3_TRANSMISSION_H
#define M3_TRANSMISSION_H

#include "m3rt/base/component_factory.h"
#include "m3/hardware/actuator.h"
#include "m3rt/base/m3rt_def.h"
#include "m3/hardware/joint.h"

namespace m3
{
	using namespace std;
	class M3Joint;
/*
	Because a sensor or actuator may not be collocated with the joint we define 
	three frames of reference for a joint or torque sensor value: 
 	1. Joint frame: as measured at the output joint
 	2. Sensor frame: as measured at the sensor (the value coming from the M3Actuator component)
 	3. Actuator frame: as measured at the actuator output
	
	If non-collocated, a mechanical transmission may exist (pulleys, belts) that gear the sensor or actuator. 
	The joint torque/theta values may also be derived from multiple actuators as in a differential
	Some transmissions may have non-linear relationships between theta/torqe/etc 
	M3Transmission provides a generic transfer between different frames of reference.
	Additional types of transmissions can be added as needed.
	
	In the case of a differential, the desired sensor setpoint of the coupled actuator is needed as well, 
	hence the GetThetaDesSensor(), etc methods.
	
	Return values are mNm, degrees
*/ 	


class M3Transmission
{
	public:
		M3Transmission():type(0),cpj(0),act(0),qj_des(0),tqj_des(0),cpt(0){}
		mReal GetThetaJointDeg();
		mReal GetThetaDotJointDeg();		
		mReal GetThetaDotDotJointDeg();
		
		mReal GetThetaActuatorDeg();
		mReal GetThetaDotActuatorDeg();
		mReal GetThetaDotDotActuatorDeg();
		
		mReal GetThetaSensorDeg();
		mReal GetThetaDotSensorDeg();
		mReal GetThetaDotDotSensorDeg();
		
		mReal GetTorqueJoint();
		mReal GetTorqueDotJoint();
		mReal GetTorqueActuator();
		mReal GetTorqueDotActuator();
		mReal GetTorqueSensor();
		mReal GetTorqueDotSensor();
		
		//mReal GetThetaDesSensorDeg();: Todo: support, causes recursive loop
		mReal GetThetaDesActuatorDeg();
		mReal GetThetaDesJointDeg(){return qj_des;};
		
		//mReal GetTorqueDesSensor();
		mReal GetTorqueDesActuator();
		mReal GetTorqueDesJoint(){return tqj_des;}
		
		void SetThetaDesJointDeg(mReal q_joint){qj_des=q_joint;}
		void SetTorqueDesJoint(mReal tq_joint){tqj_des=tq_joint;}
		
		virtual bool LinkDependentComponents(m3rt::M3ComponentFactory * f); 
		virtual void ReadConfig(const YAML::Node & doc);
	protected:
		enum {NONE, GEAR_TRANS, DIFF_TRANS};
		int type;
		M3Joint * cpj; //coupled joint component
		M3Actuator * act; //actuator component
		M3Transmission * cpt; //coupled transmission
		string cpj_name,act_name;
		vector<mReal> qs_to_qj; 	//angle sensor-to-joint ratios
		vector<mReal> qj_to_qa; 	//angle joint-to-actuator ratios
		vector<mReal> tqs_to_tqj;	//torque sensor-to-joint ratios
		vector<mReal> tqj_to_tqa;	//torque joint-to-actuator ratios
		mReal qj_des, tqj_des;
};


}

#endif


