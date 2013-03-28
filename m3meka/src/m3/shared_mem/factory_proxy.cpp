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
#include <m3/shared_mem/torque_shm.h>
///////////////////////////////////////////////////////
extern "C" 
{
///////////////////////////////////////////////////////
//These names should match the create_xxx() and destroy_xxx() function names.
//They should also match the names used for component definition in m3_config.yml 
#define M3TORQUE_SHM_TYPE_NAME	"m3torque_shm"
///////////////////////////////////////////////////////
//Creators
m3rt::M3Component * create_m3torque_shm(){return new m3::M3TorqueShm;}
//Deletors
void destroy_m3torque_shm(m3rt::M3Component* c) {delete c;}
///////////////////////////////////////////////////////
class M3FactoryProxy 
{ 
public:
	M3FactoryProxy()
	{
		m3rt::creator_factory[M3TORQUE_SHM_TYPE_NAME] =	create_m3torque_shm;
		m3rt::destroyer_factory[M3TORQUE_SHM_TYPE_NAME] =  destroy_m3torque_shm;
	}
};
///////////////////////////////////////////////////////
// The library's one instance of the proxy
M3FactoryProxy proxy;
}
