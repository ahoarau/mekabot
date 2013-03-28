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

#include "m3/hardware/transmission.h"


namespace m3{
	
using namespace m3rt;
using namespace std;

//////////////////////////////////////////////////////

void M3Transmission::ReadConfig(const YAML::Node & doc)
{	
	string t;
	doc["type"] >> t;
	doc["act_name"]>>act_name;
	if (t.compare("gear") == 0 )
		type=GEAR_TRANS;
	if (t.compare("differential") == 0 )
	{
		type=DIFF_TRANS;
		doc["cpj_name"]>>cpj_name;
	}
	if (type==DIFF_TRANS || type==GEAR_TRANS)
	{
		doc["qs_to_qj"] >>  qs_to_qj; //angle sensor-to-joint ratios
		doc["qj_to_qa"] >>  qj_to_qa; //angle joint-to-actuator ratios
		doc["tqs_to_tqj"] >> tqs_to_tqj;//torque sensor-to-joint ratios
		doc["tqj_to_tqa"] >> tqj_to_tqa;//torque joint-to-actuator ratios
	}
}

//////////////////////////////////////////////////////
bool M3Transmission::LinkDependentComponents(m3rt::M3ComponentFactory * factory)
{	
	act=(M3Actuator*) factory->GetComponent(act_name);
	if (act==NULL)
	{
		M3_INFO("M3Actuator component %s not found for M3Transmission\n",act_name.c_str());
		return false;
	}
	if (type==DIFF_TRANS)
	{
		cpj=(M3Joint*) factory->GetComponent(cpj_name);
		if (cpj==NULL)
		{
			M3_INFO("M3Joint component %s not found for M3Transmission\n",cpj_name.c_str());
			return false;
		}
		cpt = cpj->GetTransmission();
	}
	return true;
}
//////////////////////////////////////////////////////
mReal M3Transmission::GetThetaSensorDeg(){return act->GetThetaDeg();}
mReal M3Transmission::GetThetaDotSensorDeg(){return act->GetThetaDotDeg();}
mReal M3Transmission::GetThetaDotDotSensorDeg(){return act->GetThetaDotDotDeg();}
//////////////////////////////////////////////////////
mReal M3Transmission::GetThetaJointDeg()
{
	switch(type)
	{
		case GEAR_TRANS:
			return GetThetaSensorDeg()*qs_to_qj[0];
		case DIFF_TRANS:
			return GetThetaSensorDeg()*qs_to_qj[0] + cpt->GetThetaSensorDeg()*qs_to_qj[1];
		default: 
			return 0.0;
	}
}
//////////////////////////////////////////////////////
mReal M3Transmission::GetThetaDotJointDeg()
{
	switch(type)
	{
		case GEAR_TRANS:
			return GetThetaDotSensorDeg()*qs_to_qj[0];
		case DIFF_TRANS:
			return GetThetaDotSensorDeg()*qs_to_qj[0] + cpt->GetThetaDotSensorDeg()*qs_to_qj[1];
		default: 
			return 0.0;
	}
}
//////////////////////////////////////////////////////
mReal M3Transmission::GetThetaDotDotJointDeg()
{
	switch(type)
	{
		case GEAR_TRANS:
			return GetThetaDotDotSensorDeg()*qs_to_qj[0];
		case DIFF_TRANS:
			return GetThetaDotDotSensorDeg()*qs_to_qj[0] + cpt->GetThetaDotDotSensorDeg()*qs_to_qj[1];
		default: 
			return 0.0;
	}
}
//////////////////////////////////////////////////////
mReal M3Transmission::GetThetaActuatorDeg()
{
	switch(type)
	{
		case GEAR_TRANS:
			return GetThetaJointDeg()*qj_to_qa[0];
		case DIFF_TRANS:
			return GetThetaJointDeg()*qj_to_qa[0] + cpt->GetThetaJointDeg()*qj_to_qa[1];
		default: 
			return 0.0;
	}
}

//////////////////////////////////////////////////////
mReal M3Transmission::GetThetaDotActuatorDeg()
{
	switch(type)
	{
		case GEAR_TRANS:
			return GetThetaDotJointDeg()*qj_to_qa[0];
		case DIFF_TRANS:
			return GetThetaDotJointDeg()*qj_to_qa[0] + cpt->GetThetaDotJointDeg()*qj_to_qa[1];
		default: 
			return 0.0;
	}
}
//////////////////////////////////////////////////////
mReal M3Transmission::GetThetaDotDotActuatorDeg()
{
	switch(type)
	{
		case GEAR_TRANS:
			return GetThetaDotDotJointDeg()*qj_to_qa[0];
		case DIFF_TRANS:
			return GetThetaDotDotJointDeg()*qj_to_qa[0] + cpt->GetThetaDotDotJointDeg()*qj_to_qa[1];
		default: 
			return 0.0;
	}
}
//////////////////////////////////////////////////////
//Input joint-referenced desired angle
//Ouput actuator-referenced desired angle
mReal M3Transmission::GetThetaDesActuatorDeg()
{
	switch(type)
	{
		
		case GEAR_TRANS:
			return GetThetaDesJointDeg()*qj_to_qa[0];//joint->sensor->actuator
		case DIFF_TRANS:
			return GetThetaDesJointDeg()*qj_to_qa[0]+cpt->GetThetaDesJointDeg()*qj_to_qa[1];
			//return GetThetaDesJointDeg()*qj_to_qa[0]+cpt->GetThetaJointDeg()*qj_to_qa[1];
		default: 
			return 0.0;
	}
}
//////////////////////////////////////////////////////
//Input joint-referenced desired angle
//Ouput sensor-referenced desired angle
/*mReal M3Transmission::GetThetaDesSensorDeg()
{
	switch(type)
	{
		
		case GEAR_TRANS:
			return (GetThetaDesActuatorDeg()/qs_to_qa[0]);//joint->sensor
		case DIFF_TRANS:
			return (GetThetaDesJointDeg()-cpt->GetThetaDesSensorDeg()*qs_to_qj[1])/qs_to_qj[0];
		default: 
			return 0.0;
	}
}*/
//////////////////////////////////////////////////////
mReal M3Transmission::GetTorqueSensor(){ return act->GetTorque();}
mReal M3Transmission::GetTorqueDotSensor(){ return act->GetTorqueDot();}
//////////////////////////////////////////////////////
mReal M3Transmission::GetTorqueJoint()
{
	switch(type)
	{
		case GEAR_TRANS:
			return GetTorqueSensor()*tqs_to_tqj[0];
		case DIFF_TRANS:
			return GetTorqueSensor()*tqs_to_tqj[0] + cpt->GetTorqueSensor()*tqs_to_tqj[1];
		default: 
			return 0.0;
	}
}
//////////////////////////////////////////////////////
mReal M3Transmission::GetTorqueDotJoint()
{
	switch(type)
	{
		case GEAR_TRANS:
			return GetTorqueDotSensor()*tqs_to_tqj[0];
		case DIFF_TRANS:
			return GetTorqueDotSensor()*tqs_to_tqj[0] + cpt->GetTorqueDotSensor()*tqs_to_tqj[1];
		default: 
			return 0.0;
	}
}
//////////////////////////////////////////////////////
mReal M3Transmission::GetTorqueActuator()
{
	switch(type)
	{
		case GEAR_TRANS:
			return GetTorqueJoint()*tqj_to_tqa[0];
		case DIFF_TRANS:
			return GetTorqueJoint()*tqj_to_tqa[0] + cpt->GetTorqueJoint()*tqj_to_tqa[1];
		default: 
			return 0.0;
	}
}
//////////////////////////////////////////////////////
mReal M3Transmission::GetTorqueDotActuator()
{
	switch(type)
	{
		case GEAR_TRANS:
			return GetTorqueDotJoint()*tqj_to_tqa[0];
		case DIFF_TRANS:
			return GetTorqueDotJoint()*tqj_to_tqa[0] + cpt->GetTorqueDotJoint()*tqj_to_tqa[1];
		default: 
			return 0.0;
	}
}
//////////////////////////////////////////////////////
//Input joint-referenced desired torque
//Ouput actuator-referenced desired torque
mReal M3Transmission::GetTorqueDesActuator()
{
	switch(type)
	{
		
		case GEAR_TRANS:
			return GetTorqueDesJoint()*tqj_to_tqa[0];//joint->sensor->actuator
		case DIFF_TRANS:
			return GetTorqueDesJoint()*tqj_to_tqa[0]+cpt->GetTorqueDesJoint()*tqj_to_tqa[1];
		default: 
			return 0.0;
	}
}
//////////////////////////////////////////////////////
//Input joint-referenced desired torque
//Ouput sensor-referenced desired torque
/*mReal M3Transmission::GetTorqueDesSensor()
{
	switch(type)
	{
		case GEAR_TRANS:
			return (GetTorqueDesJoint()/tqs_to_tqj[0]);//joint->sensor
		case DIFF_TRANS:
			return 0;//(GetTorqueDesJoint()-cpt->GetTorqueDesSensor()*tqs_to_tqj[1])/tqs_to_tqj[0];
		default: 
			return 0.0;
	}
}*/
//////////////////////////////////////////////////////



}