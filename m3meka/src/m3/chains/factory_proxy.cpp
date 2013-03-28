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


#include <m3rt/base/component.h>
#include <m3/chains/joint_array.h>
#include <m3/chains/arm.h>
#include <m3/chains/hand.h>
#include <m3/chains/hand_ua.h>
#include <m3/chains/gripper.h>
#include <m3/chains/head.h>
#include <m3/chains/torso.h>
#include <m3/chains/dynamatics.h>


///////////////////////////////////////////////////////
extern "C" 
{
///////////////////////////////////////////////////////
//These names should match the create_xxx() and destroy_xxx() function names.
//They should also match the names used for component definition in m3_config.yml 

#define M3JOINT_ARRAY_TYPE_NAME "m3joint_array"
#define M3ARM_TYPE_NAME "m3arm"
#define M3TORSO_TYPE_NAME "m3torso"
#define M3HEAD_TYPE_NAME "m3head"
#define M3HAND_TYPE_NAME "m3hand"
#define M3HAND_UA_TYPE_NAME "m3hand_ua"
#define M3GRIPPER_TYPE_NAME "m3gripper"
#define M3DYNAMATICS_TYPE_NAME "m3dynamatics"


///////////////////////////////////////////////////////
//Creators

m3rt::M3Component * create_m3joint_array(){return new m3::M3JointArray;}
m3rt::M3Component * create_m3arm(){return new m3::M3Arm;}
m3rt::M3Component * create_m3torso(){return new m3::M3Torso;}
m3rt::M3Component * create_m3head(){return new m3::M3Head;}
m3rt::M3Component * create_m3hand(){return new m3::M3Hand;}
m3rt::M3Component * create_m3hand_ua(){return new m3::M3HandUA;}
m3rt::M3Component * create_m3gripper(){return new m3::M3Gripper;}
m3rt::M3Component * create_m3dynamatics(){return new m3::M3Dynamatics;}


//Deletors
void destroy_m3joint_array(m3rt::M3Component* c) {delete c;}
void destroy_m3arm(m3rt::M3Component* c) {delete c;}
void destroy_m3torso(m3rt::M3Component* c) {delete c;}
void destroy_m3head(m3rt::M3Component* c) {delete c;}
void destroy_m3hand(m3rt::M3Component* c) {delete c;}
void destroy_m3hand_ua(m3rt::M3Component* c) {delete c;}
void destroy_m3gripper(m3rt::M3Component* c) {delete c;}
void destroy_m3dynamatics(m3rt::M3Component* c) {delete c;}


///////////////////////////////////////////////////////
class M3FactoryProxy 
{ 
public:
	M3FactoryProxy()
	{
		m3rt::creator_factory[M3JOINT_ARRAY_TYPE_NAME] = create_m3joint_array;
		m3rt::creator_factory[M3ARM_TYPE_NAME] =  create_m3arm;
		m3rt::creator_factory[M3TORSO_TYPE_NAME] =  create_m3torso;
		m3rt::creator_factory[M3HEAD_TYPE_NAME] =  create_m3head;
		m3rt::creator_factory[M3HAND_TYPE_NAME] =  create_m3hand;
		m3rt::creator_factory[M3HAND_UA_TYPE_NAME] =  create_m3hand_ua;
		m3rt::creator_factory[M3GRIPPER_TYPE_NAME] =  create_m3gripper;
		m3rt::creator_factory[M3DYNAMATICS_TYPE_NAME] =  create_m3dynamatics;

		m3rt::destroyer_factory[M3JOINT_ARRAY_TYPE_NAME] = destroy_m3joint_array;
		m3rt::destroyer_factory[M3ARM_TYPE_NAME] =  destroy_m3arm;
		m3rt::destroyer_factory[M3TORSO_TYPE_NAME] =  destroy_m3torso;
		m3rt::destroyer_factory[M3HEAD_TYPE_NAME] =  destroy_m3head;
		m3rt::destroyer_factory[M3HAND_TYPE_NAME] =  destroy_m3hand;
		m3rt::destroyer_factory[M3HAND_UA_TYPE_NAME] =  destroy_m3hand_ua;		
		m3rt::destroyer_factory[M3GRIPPER_TYPE_NAME] =  destroy_m3gripper;
		m3rt::destroyer_factory[M3DYNAMATICS_TYPE_NAME] =  destroy_m3dynamatics;

	}
};
///////////////////////////////////////////////////////
// The library's one instance of the proxy
M3FactoryProxy proxy;
}
