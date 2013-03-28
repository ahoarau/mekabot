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
 
#include <stdio.h>
#include <m3rt/base/component.h>
#include <m3/hardware/actuator_ec.h>
#include <m3/hardware/actuator.h>
#include <m3/hardware/pwr_ec.h>
#include <m3/hardware/pwr.h>
#include <m3/hardware/pwr_virtual.h>
#include <m3/hardware/joint.h>
#include <m3/hardware/joint_slave.h>
#include <m3/hardware/joint_zlift.h>
#include <m3/hardware/actuator_virtual.h>
#include <m3/hardware/loadx1_ec.h>
#include <m3/hardware/loadx1.h>
#include <m3/hardware/loadx6_ec.h>
#include <m3/hardware/loadx6.h>
#include <m3/hardware/loadx6_virtual.h>
#include <m3/hardware/tactile_pps22_ec.h>
#include <m3/hardware/ledx2_ec.h>
#include <m3/hardware/ledx2xn_ec.h>
#include <m3/hardware/led_matrix_ec.h>
#include <m3/hardware/robot_monitor.h>
#include <m3/hardware/log_test.h>
#include <m3/hardware/led_matrix_ec_shm.h>
#include <m3/hardware/ledx2xn_ec_shm.h>
#ifdef __RTAI__
//#include <m3/hardware/async_io.h>
#endif
#include <m3/hardware/ctrl_simple.h>
#include <m3/hardware/joint_zlift_shm.h>

///////////////////////////////////////////////////////
extern "C" 
{
///////////////////////////////////////////////////////
//These names should match the create_xxx() and destroy_xxx() function names.
//They should also match the names used for component definition in m3_config.yml 

#define M3ACTUATOREC_TYPE_NAME "m3actuator_ec"
#define M3ACTUATOR_TYPE_NAME "m3actuator"
#define M3ACTUATOR_VIRTUAL_TYPE_NAME "m3actuator_virtual"
#define M3PWREC_TYPE_NAME "m3pwr_ec"
#define M3PWR_TYPE_NAME "m3pwr"
#define M3PWR_VIRTUAL_TYPE_NAME "m3pwr_virtual"
#define M3JOINT_TYPE_NAME "m3joint"
#define M3JOINT_SLAVE_TYPE_NAME "m3joint_slave"
#define M3_LOADX1_EC_TYPE_NAME "m3loadx1_ec"
#define M3_LOADX1_TYPE_NAME "m3loadx1"
#define M3LOADX6EC_TYPE_NAME "m3loadx6_ec"
#define M3LOADX6_TYPE_NAME "m3loadx6"
#define M3LOADX6_VIRTUAL_TYPE_NAME "m3loadx6_virtual"
#define M3TACTILE_PPS22EC_TYPE_NAME "m3tactile_pps22_ec"
#define M3LEDX2_EC_TYPE_NAME "m3ledx2_ec"
#define M3LEDX2XN_EC_TYPE_NAME "m3ledx2xn_ec"
#define M3LED_MATRIX_EC_TYPE_NAME "m3led_matrix_ec"
#define M3JOINT_ZLIFT_TYPE_NAME "m3joint_zlift"
#define M3ROBOT_MONITOR_TYPE_NAME "m3robot_monitor"
#define M3ASYNC_IO_TYPE_NAME "m3async_io"
#define M3LOG_TEST_TYPE_NAME "m3log_test"
#define M3CTRL_SIMPLE_NAME "m3ctrl_simple"
#define M3JOINT_ZLIFT_SHM_TYPE_NAME "m3joint_zlift_shm"
#define M3LED_MATRIX_EC_SHM_TYPE_NAME "m3led_matrix_ec_shm"
#define M3LED_X2XN_EC_SHM_TYPE_NAME "m3ledx2xn_ec_shm"
///////////////////////////////////////////////////////
//Creators

m3rt::M3Component * create_m3actuator_ec(){return new m3::M3ActuatorEc;}
m3rt::M3Component * create_m3pwr_ec(){return new m3::M3PwrEc;}
m3rt::M3Component * create_m3actuator(){return new m3::M3Actuator;}
m3rt::M3Component * create_m3pwr(){return new m3::M3Pwr;}
m3rt::M3Component * create_m3pwr_virtual(){return new m3::M3PwrVirtual;}
m3rt::M3Component * create_m3joint(){return new m3::M3Joint;}
m3rt::M3Component * create_m3joint_slave(){return new m3::M3JointSlave;}
m3rt::M3Component * create_m3joint_zlift(){return new m3::M3JointZLift;}
m3rt::M3Component * create_m3actuator_virtual(){return new m3::M3ActuatorVirtual;}
m3rt::M3Component * create_m3loadx1_ec(){return new m3::M3LoadX1Ec;}
m3rt::M3Component * create_m3loadx1(){return new m3::M3LoadX1;}
m3rt::M3Component * create_m3loadx6_ec(){return new m3::M3LoadX6Ec;}
m3rt::M3Component * create_m3loadx6(){return new m3::M3LoadX6;}
m3rt::M3Component * create_m3loadx6_virtual(){return new m3::M3LoadX6Virtual;}
m3rt::M3Component * create_m3tactile_pps22_ec(){return new m3::M3TactilePPS22Ec;}
m3rt::M3Component * create_m3ledx2_ec(){return new m3::M3LedX2Ec;}
m3rt::M3Component * create_m3ledx2xn_ec(){return new m3::M3LedX2XNEc;}
m3rt::M3Component * create_m3led_matrix_ec(){return new m3::M3LedMatrixEc;}
m3rt::M3Component * create_m3robot_monitor(){return new m3::M3RobotMonitor;}
m3rt::M3Component * create_m3log_test(){return new m3::M3MekaLogTest;}
m3rt::M3Component * create_m3joint_zlift_shm(){return new m3::M3JointZLiftShm;}
m3rt::M3Component * create_m3led_matrix_ec_shm(){return new m3::M3LedMatrixEcShm;}
m3rt::M3Component * create_m3led_x2xn_ec_shm(){return new m3::M3LedX2XNEcShm;}
#ifdef __RTAI__
//m3rt::M3Component * create_m3async_io(){return new m3::M3AsyncIO;}
#endif
m3rt::M3Component * create_m3ctrl_simple(){return new m3::M3CtrlSimple;}
//Deletors

void destroy_m3actuator_ec(m3rt::M3Component* c) {delete c;}
void destroy_m3pwr_ec(m3rt::M3Component* c) {delete c;}
void destroy_m3actuator(m3rt::M3Component* c) {delete c;}
void destroy_m3pwr(m3rt::M3Component* c) {delete c;}
void destroy_m3pwr_virtual(m3rt::M3Component* c) {delete c;}
void destroy_m3joint(m3rt::M3Component* c) {delete c;}
void destroy_m3joint_slave(m3rt::M3Component* c) {delete c;}
void destroy_m3joint_zlift(m3rt::M3Component* c) {delete c;}
void destroy_m3actuator_virtual(m3rt::M3Component* c) {delete c;}
void destroy_m3loadx1_ec(m3rt::M3Component* c) {delete c;}
void destroy_m3loadx1(m3rt::M3Component* c) {delete c;}
void destroy_m3loadx6_ec(m3rt::M3Component* c) {delete c;}
void destroy_m3loadx6(m3rt::M3Component* c) {delete c;}
void destroy_m3loadx6_virtual(m3rt::M3Component* c) {delete c;}
void destroy_m3tactile_pps22_ec(m3rt::M3Component* c) {delete c;}
void destroy_m3ledx2_ec(m3rt::M3Component* c) {delete c;}
void destroy_m3ledx2xn_ec(m3rt::M3Component* c) {delete c;}
void destroy_m3led_matrix_ec(m3rt::M3Component* c) {delete c;}
void destroy_m3robot_monitor(m3rt::M3Component* c) {delete c;}
void destroy_m3log_test(m3rt::M3Component* c) {delete c;}
void destroy_m3joint_zlift_shm(m3rt::M3Component* c) {delete c;}
void destroy_m3led_matrix_ec_shm(m3rt::M3Component* c) {delete c;}
void destroy_m3led_x2xn_ec_shm(m3rt::M3Component* c) {delete c;}
#ifdef __RTAI__
//void destroy_m3async_io(m3rt::M3Component* c) {delete c;}
#endif
void destroy_m3ctrl_simple(m3rt::M3Component* c) {delete c;}
///////////////////////////////////////////////////////
class M3FactoryProxy 
{ 
public:
	M3FactoryProxy()
	{
		m3rt::creator_factory[M3_LOADX1_EC_TYPE_NAME] =	create_m3loadx1_ec;
		m3rt::destroyer_factory[M3_LOADX1_EC_TYPE_NAME] =  destroy_m3loadx1_ec;
		
		m3rt::creator_factory[M3_LOADX1_TYPE_NAME] =	create_m3loadx1;
		m3rt::destroyer_factory[M3_LOADX1_TYPE_NAME] =  destroy_m3loadx1;
		
		m3rt::creator_factory[M3ACTUATOREC_TYPE_NAME] =	create_m3actuator_ec;
		m3rt::destroyer_factory[M3ACTUATOREC_TYPE_NAME] =  destroy_m3actuator_ec;
		
		m3rt::creator_factory[M3PWREC_TYPE_NAME] =	create_m3pwr_ec;
		m3rt::destroyer_factory[M3PWREC_TYPE_NAME] =  destroy_m3pwr_ec;
		
		m3rt::creator_factory[M3ACTUATOR_TYPE_NAME] =	create_m3actuator;
		m3rt::destroyer_factory[M3ACTUATOR_TYPE_NAME] =  destroy_m3actuator;
		
		m3rt::creator_factory[M3PWR_TYPE_NAME] =	create_m3pwr;
		m3rt::destroyer_factory[M3PWR_TYPE_NAME] =  destroy_m3pwr;

		m3rt::creator_factory[M3PWR_VIRTUAL_TYPE_NAME] =	create_m3pwr_virtual;
		m3rt::destroyer_factory[M3PWR_VIRTUAL_TYPE_NAME] =  destroy_m3pwr_virtual;

		m3rt::creator_factory[M3JOINT_TYPE_NAME] =	create_m3joint;
		m3rt::destroyer_factory[M3JOINT_TYPE_NAME] =  destroy_m3joint;

		m3rt::creator_factory[M3JOINT_SLAVE_TYPE_NAME] =	create_m3joint_slave;
		m3rt::destroyer_factory[M3JOINT_SLAVE_TYPE_NAME] =  destroy_m3joint_slave;
		
		m3rt::creator_factory[M3JOINT_ZLIFT_TYPE_NAME] =	create_m3joint_zlift;
		m3rt::destroyer_factory[M3JOINT_ZLIFT_TYPE_NAME] =  destroy_m3joint_zlift;
		
		m3rt::creator_factory[M3ACTUATOR_VIRTUAL_TYPE_NAME] =	create_m3actuator_virtual;
		m3rt::destroyer_factory[M3ACTUATOR_VIRTUAL_TYPE_NAME] =  destroy_m3actuator_virtual;
		
		m3rt::creator_factory[M3LEDX2_EC_TYPE_NAME] =	create_m3ledx2_ec;
		m3rt::destroyer_factory[M3LEDX2_EC_TYPE_NAME] =  destroy_m3ledx2_ec;
		
		m3rt::creator_factory[M3LEDX2XN_EC_TYPE_NAME] =	create_m3ledx2xn_ec;
		m3rt::destroyer_factory[M3LEDX2XN_EC_TYPE_NAME] =  destroy_m3ledx2xn_ec;
		
		m3rt::creator_factory[M3LED_MATRIX_EC_TYPE_NAME] =	create_m3led_matrix_ec;
		m3rt::destroyer_factory[M3LED_MATRIX_EC_TYPE_NAME] =  destroy_m3led_matrix_ec;
		
		m3rt::creator_factory[M3LOADX6EC_TYPE_NAME] =	create_m3loadx6_ec;
		m3rt::destroyer_factory[M3LOADX6EC_TYPE_NAME] =  destroy_m3loadx6_ec;
		
		m3rt::creator_factory[M3LOADX6_TYPE_NAME] =	create_m3loadx6;
		m3rt::destroyer_factory[M3LOADX6_TYPE_NAME] =  destroy_m3loadx6;
		
		m3rt::creator_factory[M3LOADX6_VIRTUAL_TYPE_NAME] =	create_m3loadx6_virtual;
		m3rt::destroyer_factory[M3LOADX6_VIRTUAL_TYPE_NAME] =  destroy_m3loadx6_virtual;

		m3rt::creator_factory[M3TACTILE_PPS22EC_TYPE_NAME] =	create_m3tactile_pps22_ec;
		m3rt::destroyer_factory[M3TACTILE_PPS22EC_TYPE_NAME] =  destroy_m3tactile_pps22_ec;
		
		m3rt::creator_factory[M3ROBOT_MONITOR_TYPE_NAME] =	create_m3robot_monitor;
		m3rt::destroyer_factory[M3ROBOT_MONITOR_TYPE_NAME] =  destroy_m3robot_monitor;
#ifdef __RTAI__
//		m3rt::creator_factory[M3ASYNC_IO_TYPE_NAME] =	create_m3async_io;
//		m3rt::destroyer_factory[M3ASYNC_IO_TYPE_NAME] =  destroy_m3async_io;
#endif
		m3rt::creator_factory[M3LOG_TEST_TYPE_NAME] =	create_m3log_test;
		m3rt::destroyer_factory[M3LOG_TEST_TYPE_NAME] =  destroy_m3log_test;
		
		m3rt::creator_factory[M3CTRL_SIMPLE_NAME] =	create_m3ctrl_simple;
		m3rt::destroyer_factory[M3CTRL_SIMPLE_NAME] =  destroy_m3ctrl_simple;
		
		m3rt::creator_factory[M3JOINT_ZLIFT_SHM_TYPE_NAME] =	create_m3joint_zlift_shm;
		m3rt::destroyer_factory[M3JOINT_ZLIFT_SHM_TYPE_NAME] =  destroy_m3joint_zlift_shm;

		m3rt::creator_factory[M3LED_MATRIX_EC_SHM_TYPE_NAME] =	create_m3led_matrix_ec_shm;
		m3rt::destroyer_factory[M3LED_MATRIX_EC_SHM_TYPE_NAME] =  destroy_m3led_matrix_ec_shm;

		m3rt::creator_factory[M3LED_X2XN_EC_SHM_TYPE_NAME] =	create_m3led_x2xn_ec_shm;
		m3rt::destroyer_factory[M3LED_X2XN_EC_SHM_TYPE_NAME] =  destroy_m3led_x2xn_ec_shm;
		
	}
	
};
///////////////////////////////////////////////////////
// The library's one instance of the proxy
M3FactoryProxy proxy;
}
