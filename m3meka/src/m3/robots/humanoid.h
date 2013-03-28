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


/*! \mainpage M3 API Documentation
 *
 * \ref m3::M3Humanoid : The M3Humanoid class has been designed as the principal interface for controlling an M3 robot.
 * 
 */

/** \defgroup PackageName M3Humanoid
 *
 * \brief API for M3Humanoid
 */

/**
 * \class M3Humanoid
 *
 * \ingroup M3Humanoid
 *
 * \brief M3 Humanoid API
 *
 * Hi this is the M3Humanoid API.
 *
 */




#ifndef M3_HUMANOID_H
#define M3_HUMANOID_H

#include "m3rt/base/component.h"
#include "m3/toolbox/toolbox.h"
#include "m3/robots/humanoid.pb.h"
#include "m3/chains/torso.h"
#include "m3/chains/dynamatics.h"
#include "m3/chains/arm.h"
#include "m3/chains/head.h"
#include "m3/robots/robot.h"
#include "m3/robots/chain_name.h"
#include <google/protobuf/message.h>

namespace m3
{
	using namespace std;
	using namespace KDL;
	
///////////////////////////////////////////////////////////////////////////
	
class M3Humanoid : public M3Robot
{
	public:
		/// Create an M3Humanoid
		M3Humanoid() : m3::M3Robot(), head(NULL), torso(NULL), right_arm(NULL), left_arm(NULL), force_shm_r_arm(false), 
		  force_shm_l_arm(false), force_shm_torso(false), force_shm_head(false), enable_shm_r_arm(false), enable_shm_l_arm(false), enable_shm_torso(false), enable_shm_head(false),
		    startup_motor_pwr_on(false)
		{
		  head_base_2_world_frame=Frame::Identity();
		}
		~M3Humanoid(){ }
				
		/// Enable power to all motors.
		void SetMotorPowerOn();
		
		/// Disable power to all motors.
		void SetMotorPowerOff();
		
		/** \brief Gets joint values in degrees for selected chain.
		* \param chain desired joint group (RIGHT_ARM, LEFT_ARM, TORSO, or HEAD)
		* \param idx an integer specifying which joint angle to return	
		* \return specified joint angle in degrees
		*/
		mReal GetThetaDeg(M3Chain chain,unsigned int  idx);
		
		/** \brief Sets joint controller command for selected chain to desired angle.  
		* \param chain desired joint group (RIGHT_ARM, LEFT_ARM, TORSO, or HEAD)
		* \param idx an integer specifying which joint angle to command			
		* \param theta desired joint angle in degrees
		* \note Joint must be in theta or theta_gc mode for controller to track desired angle commands.
		*
		* 
		* 
		*/
		void SetThetaDeg(M3Chain chain,unsigned int  idx, mReal theta);
		
		/** \brief Gets joint velocity values in degrees/second for selected chain.
		* \param chain desired joint group (RIGHT_ARM, LEFT_ARM, TORSO, or HEAD)
		* \param idx an integer specifying which joint value to return
		* \return Joint velocity value in degrees/second for chain.
		*/
		mReal GetThetaDotDeg(M3Chain chain,unsigned int  idx);
		
		/** \brief Gets joint acceleration values in radians/(sec^2) for selected chain.
		* \param chain desired joint group (RIGHT_ARM, LEFT_ARM, TORSO, or HEAD)
		* \param idx an integer specifying which joint value to return	
		* \return Joint acceleration value in radians/(sec^2) for chain.
		*/
		mReal GetThetaDotDotDeg(M3Chain chain,unsigned int  idx);
				
		/** \brief Gets torque value of joint number idx in mN*m for a chain.
		* \param chain desired joint group (RIGHT_ARM, LEFT_ARM, TORSO, or HEAD)
		* \param idx an integer specifying which joint value to return	
		* \return Torque value in mN*m for selected chain.
		*/
		mReal GetTorque_mNm(M3Chain chain,unsigned int  idx);
		
		/** \brief Gets torque time derivative value of joint number idx in mN*m/sec for a chain.
		* \param chain desired joint group (RIGHT_ARM, LEFT_ARM, TORSO, or HEAD)
		* \param idx an integer specifying which joint value to return			
		* \return Torque derivative values in mN*m/sec for selected chain.
		*/
		mReal GetTorqueDot_mNm(M3Chain chain,unsigned int  idx);
		
		/** \brief Gets XYZ position in meters of end frame origin for selected chain with reference to the world frame.
		* \param chain desired joint group (RIGHT_ARM, LEFT_ARM, TORSO, or HEAD)		
		* \return an Eigen::Vector3d giving position of end frame for chain with respect to world frame in meters.
		* \note End frame defined by the chain DH kinematic parameters.
		*/
		Eigen::Vector3d GetEndPosition(M3Chain chain); //3x1 position of End frame in Base coords (x,y,z)
		
		/** \brief Gets rotation matrix of end frame origin for selected chain with reference to the world frame.
		* \param chain desired joint group (RIGHT_ARM, LEFT_ARM, TORSO, or HEAD)		
		* \return an Eigen::Matrix3d giving rotation of end frame for chain with respect to world frame in meters.
		* \note End frame defined by the chain DH kinematic parameters.
		*/
		Eigen::Matrix3d GetEndRotation(M3Chain chain); //3x3 Rotation Mtx
		
		/** \brief Returns Jacobian expressed in the world frame for the end frame defined by that chain's DH kinematic parameters.
		* \param chain desired joint group (RIGHT_ARM, LEFT_ARM, TORSO, or HEAD)		
		* \return an Eigen::MatrixXd (6 x ndof) Jacobian with respect to world frame.
		*/
		Eigen::MatrixXd GetJacobian(M3Chain chain); //6xndof Jacobian Frame ndof+1 to Frame 0
		
		/** \brief Gets torque value of gravity compensation controller in mN*m for given chain and joint index.
		* \param chain desired joint group (RIGHT_ARM, LEFT_ARM, TORSO, or HEAD)		
		* \return Torque value in mN*m for given chain and joint index of gravity compensation controller.
		*/
		mReal GetGravity(M3Chain chain,unsigned int idx);
		
		/** \brief Gets motor commanded pwm duty cycle percentage for chain and joint index.
		* \param chain desired joint group (RIGHT_ARM, LEFT_ARM, TORSO, or HEAD)
		* \param idx an integer specifying which joint value to return			
		* \return PWM duty cycle percentage.
		*/
		mReal GetPwm(M3Chain chain,unsigned int  idx);
		
		/** \brief Sets joint controller command for chain and joint index to desired torque value in mN*m.t index.
		* \param chain desired joint group (RIGHT_ARM, LEFT_ARM, TORSO, or HEAD)
		* \param idx an integer specifying which joint  command to modify		
		* \param torque desired torque value.
		* \note Joint must be in torque or torque_gc mode for controller to track desired torque command.
		*/				
		void SetTorque_mNm(M3Chain chain,unsigned int  idx, mReal torque);
		
		void SetThetaDotDeg(M3Chain chain,unsigned int  idx, mReal theta_dot);
		
		void SetTorqueSharedMem_mNm(M3Chain chain,unsigned int idx, mReal torque);
		
		/** \brief Sets joint controller command for chain and joint index to desired pwm duty cycle percentage.
		* \param chain desired joint group (RIGHT_ARM, LEFT_ARM, TORSO, or HEAD)
		* \param idx an integer specifying which joint  command to modify
		* \param pwm desired pwm value (0-100) for joint
		* \note Joint must be in pwm mode for controller to issue correct commands.
		*/
		void SetPwm(M3Chain chain,unsigned int  idx, mReal pwm);
		
		/** \brief Sets joint slew rate for chain and joint index to desired values in degrees/sec.
		* \param chain desired joint group (RIGHT_ARM, LEFT_ARM, TORSO, or HEAD)
		* \param idx an integer specifying which joint  command to modify
		* \param slew_rate desired slew rate
		* \note Joint must be in theta, theta_gc, theta_mj, or theta_gc_mj mode for slew rate to effect joint output.
		*/
		void SetSlewRate(M3Chain chain,unsigned int  idx, mReal slew_rate);  // degs/sec
		
		/** \brief Sets joint slew rate for chain and joint index to desired proportion of maximum slew rate defined in config files.
		* \param chain desired joint group (RIGHT_ARM, LEFT_ARM, TORSO, or HEAD)
		* \param idx an integer specifying which joint  command to modify
		* \param slew_rate desired slew rate scaled between 0 and 1.0
		* \note Joint must be in theta, theta_gc, theta_mj, or theta_gc_mj mode for slew rate to effect joint output.
		*
		* Proportion must be between 0 and 1. A value of 0 effectively disables the joint angle reference command, while a 		
		* stiffness of 1 would make the joint slew rate the maximum value.
		*/
		void SetSlewRateProportional(M3Chain chain,unsigned int  idx, mReal slew_rate);  // 0.0 -> 1.0
		
		/** \brief Sets joint controller mode for chain and joint index to pwm control.
		* \param chain desired joint group (RIGHT_ARM, LEFT_ARM, TORSO, or HEAD)
		* \param idx an integer specifying which joint command to modify				
		*		
		*/
		void SetModePwm(M3Chain chain,unsigned int  idx);
		
		/** \brief Sets joint controller mode for chain and joint index to torque control.
		* \param chain desired joint group (RIGHT_ARM, LEFT_ARM, TORSO, or HEAD)
		* \param idx an integer specifying which joint command to modify
		*		
		*/
		void SetModeTorque(M3Chain chain,unsigned int  idx);
		
		/** \brief Sets joint controller mode for chain and joint index to torque control with gravity compensation.
		* \param chain desired joint group (RIGHT_ARM, LEFT_ARM, TORSO, or HEAD)
		* \param idx an integer specifying which joint command to modify
		*		
		*/
		void SetModeTorqueGc(M3Chain chain,unsigned int  idx);
		
		/** \brief Sets joint controller mode for chain and joint index to joint angle control with gravity compensation.
		* \param chain desired joint group (RIGHT_ARM, LEFT_ARM, TORSO, or HEAD)
		* \param idx an integer specifying which joint command to modify
		*		
		*/
		void SetModeThetaGcMj(M3Chain chain,unsigned int  idx);
		
		/** \brief Sets joint controller mode for chain and joint index to joint angle control with gravity compensation and minimum jerk filtering
		* \param chain desired joint group (RIGHT_ARM, LEFT_ARM, TORSO, or HEAD)
		* \param idx an integer specifying which joint command to modify
		*		
		*/
		void SetModeThetaMj(M3Chain chain,unsigned int  idx);
		
		/** \brief Sets joint controller mode for chain and joint index to joint angle control with gravity compensation.
		* \param chain desired joint group (RIGHT_ARM, LEFT_ARM, TORSO, or HEAD)
		* \param idx an integer specifying which joint command to modify
		*		
		*/
		void SetModeThetaGc(M3Chain chain,unsigned int  idx);
		
		/** \brief Sets joint controller mode for chain and joint index to joint angle control.
		* \param chain desired joint group (RIGHT_ARM, LEFT_ARM, TORSO, or HEAD)
		* \param idx an integer specifying which joint command to modify
		*		
		*/
		void SetModeTheta(M3Chain chain,unsigned int  idx);
		
		/** \brief Sets joint controller mode for chain and joint index to OFF.
		* \param chain desired joint group (RIGHT_ARM, LEFT_ARM, TORSO, or HEAD)
		* \param idx an integer specifying which joint command to modify
		*		
		*/
		void SetModeOff(M3Chain chain,unsigned int  idx);
		
		/** \brief Sets joint controller stiffness for chain and joint index to desired value
		* \param chain desired joint group (RIGHT_ARM, LEFT_ARM, TORSO, or HEAD)
		* \param idx an integer specifying which joint command to modify
		* \param stiffness Joint stiffness value (0-1.0)
		*		
		*/
		void SetStiffness(M3Chain chain,unsigned int  idx, mReal stiffness);  // 0.0 -> 1.0	
		
		/** \brief Gets the payload mass specified for the realtime inverse dynamic model.
		* \param chain desired joint group (RIGHT_ARM, LEFT_ARM, TORSO, or HEAD)
		* \param return mass in Kg for payload.
		*/
		mReal GetPayloadMass(M3Chain chain);
		
		/** \brief Gets the specified payload inertia in the end frame for the realtime inverse dynamic model.
		* \param chain desired joint group (RIGHT_ARM, LEFT_ARM, TORSO, or HEAD)
		* \param idx index to 6D inertia vector where 0=Ixx, 1=Ixy, 2=Ixz, 3=Iyy, 4=Iyz, 5=Izz
		* \param return inertia value for payload in end frame.
		*/
		mReal GetPayloadInertia(M3Chain chain,unsigned int  idx);
		
		/** \brief Gets the specified payload center-of-mass in the end frame for the realtime inverse dynamic model.
		* \param chain desired joint group (RIGHT_ARM, LEFT_ARM, TORSO, or HEAD)
		* \param idx index to 3D COM vector where 0=X, 1=Y, 2=Z
		* \param return  meters of center-of-mass for payload in end frame.
		*/
		mReal GetPayloadCom(M3Chain chain,unsigned int  idx);
		
		/** \brief Sets the estimated payload mass for the realtime inverse dynamic model.
		* \param chain desired joint group (RIGHT_ARM, LEFT_ARM, TORSO, or HEAD)
		* \param mass mass in Kg for payload.
		*/
		void SetPayloadMass(M3Chain chain, mReal mass);
		
		/** \brief Sets the estimated payload center-of-mass in the end frame for the realtime inverse dynamic model.
		* \param chain desired joint group (RIGHT_ARM, LEFT_ARM, TORSO, or HEAD)
		* \param idx index to 3D COM vector where 0=X, 1=Y, 2=Z
		* \param com  meters of center-of-mass for payload in end frame.
		*/
		void SetPayloadCom(M3Chain chain,unsigned int  idx, mReal com);
		
		/** \brief Sets the estimated payload inertia in the end frame for the realtime inverse dynamic model.
		* \param chain desired joint group (RIGHT_ARM, LEFT_ARM, TORSO, or HEAD)
		* \param idx index to 6D inertia vector where 0=Ixx, 1=Ixy, 2=Ixz, 3=Iyy, 4=Iyz, 5=Izz
		* \param inertia inertia value for payload in end frame.
		*/
		void SetPayloadInertia(M3Chain chain,unsigned int  idx, mReal inertia);
		
		/** \brief Gets number of degrees of freedom for a chain.
		* \param chain desired joint group (RIGHT_ARM, LEFT_ARM, TORSO, or HEAD)
		* \return an integer equal to the number of degrees of freedom for selected chain.
		*/
		int GetNdof(M3Chain chain);
		
		/** \brief Gets joint control mode for selected chain.
		* \param chain desired joint group (RIGHT_ARM, LEFT_ARM, TORSO, or HEAD)
		* \param idx an integer specifying which joint ctrl mode to return	
		* \return JOINT_ARRAY_MODE
		*/
		JOINT_ARRAY_MODE GetMode(M3Chain chain, unsigned int  idx);
		/** \brief Gets timestamp assigned from RT system.
		* \return an 64-bit integer equal to the number of ns defined by ethercat timestamp.
		*/
		long long GetTimestamp(){return GetBaseStatus()->timestamp();}
		void SetThetaSharedMem_Deg(M3Chain chain,unsigned int idx, mReal theta);
		void SetSlewRateSharedMem_Deg(M3Chain chain,unsigned int idx, mReal theta);
		void DisableTorqueShmRightArm(){enable_shm_r_arm = false;}
		void EnableTorqueShmRightArm(){enable_shm_r_arm = true;}
		void DisableTorqueShmLeftArm(){enable_shm_l_arm = false;}
		void EnableTorqueShmLeftArm(){enable_shm_l_arm = true;}
		void DisableTorqueShmTorso(){enable_shm_torso = false;}
		void EnableTorqueShmTorso(){enable_shm_torso = true;}
		void DisableAngleShmHead(){enable_shm_head = false;}
		void EnableAngleShmHead(){enable_shm_head = true;}
		
		google::protobuf::Message * GetCommand(){return &command;}
		google::protobuf::Message * GetStatus(){return &status;}
		google::protobuf::Message * GetParam(){return &param;}
		Frame GetRightEye2WorldTransform(){return right_eye_2_world_frame;}
		Frame GetLeftEye2WorldTransform(){return left_eye_2_world_frame;}
		Frame GetHeadBase2WorldTransform(){return head_base_2_world_frame;}
		
	protected:
		bool ReadConfig(const char * filename);
		void Startup();
		void Shutdown();
		void StepStatus();
		void StepCommand();
		bool LinkDependentComponents();	
		M3BaseStatus * GetBaseStatus(){return status.mutable_base();}
		M3BaseHumanoidCommand * GetCmd(M3Chain chain);
		const M3BaseHumanoidStatus * GetStatus(M3Chain chain);
		M3BaseHumanoidParam * GetParam(M3Chain chain);
		M3HumanoidStatus status;
		M3HumanoidCommand command;
		M3HumanoidParam param;
		void TestAPI();
		Vector grav_end_torso;
		Frame torso_end_frame;
		Frame right_arm_end_frame;
		Frame left_arm_end_frame;	
		Frame head_end_frame;
		Twist torso_twist;
		Twist right_arm_twist;
		Twist left_arm_twist;
		Twist head_twist;
		string torso_name;
		string right_arm_name;
		string left_arm_name;
		string head_name;		
		Frame torso_offset;
		Frame right_arm_offset;
		Frame left_arm_offset;
		Frame head_offset;
		Frame left_eye_offset;
		Frame right_eye_offset;
		M3Torso * torso;
		M3Head * head;
		M3Arm * right_arm;
		M3Arm * left_arm;		
		Wrench torso_wrench;		
		Frame right_eye_2_world_frame;
		Frame left_eye_2_world_frame;
		Frame head_base_2_world_frame;
		Jacobian J_right_arm;
		Jacobian J_left_arm;
		Jacobian J_torso;
		Jacobian J_head;
		JntArray torque_shm_right_arm;
		JntArray torque_shm_left_arm;
		JntArray torque_shm_torso;
		JntArray angle_shm_head;
		JntArray slew_rate_shm_head;
		int tmp_cnt;
		bool force_shm_r_arm;
		bool force_shm_l_arm;
		bool force_shm_torso;
		bool force_shm_head;
		bool enable_shm_r_arm;
		bool enable_shm_l_arm;
		bool enable_shm_torso;
		bool enable_shm_head;
		bool startup_motor_pwr_on;
};

}

#endif


