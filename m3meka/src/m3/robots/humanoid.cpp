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

#include "m3/robots/humanoid.h"
#include "m3rt/base/component_factory.h"
#include "m3rt/base/toolbox.h"
#include "m3rt/base/m3rt_def.h"
//#include "m3rt/rt_system/rt_ros_service.h"
namespace m3
{
	
using namespace std;
using namespace KDL;


bool M3Humanoid::LinkDependentComponents()
{
	
	if (!M3Robot::LinkDependentComponents())
		return false;
	
	int nl=0;	

	if (right_arm_name.size()!=0)
	{		
		right_arm=(M3Arm*)factory->GetComponent(right_arm_name);
		if (right_arm!=NULL)
			nl++;
		else
			M3_WARN("M3Arm component %s declared for right_arm but could not be linked\n",
					right_arm_name.c_str());
	}
	if (left_arm_name.size()!=0)
	{		
		left_arm=(M3Arm*)factory->GetComponent(left_arm_name);
		if (left_arm!=NULL)
			nl++;
		else
			M3_WARN("M3Arm component %s declared for left_arm but could not be linked\n",
					left_arm_name.c_str());

	}
	if (torso_name.size()!=0)
	{	
		torso=(M3Torso*)factory->GetComponent(torso_name);
		if (torso!=NULL)
			nl++;
		else
			M3_WARN("M3Torso component %s declared for torso but could not be linked\n",
					torso_name.c_str());

	}
	if (head_name.size()!=0)
	{	
		
		
		head=(M3Head*)factory->GetComponent(head_name);
		if (head!=NULL)
			nl++;
		else
			M3_WARN("M3Head component %s declared for head but could not be linked\n",
					head_name.c_str());

	}
	
	if (nl>0)
		return true;

	M3_ERR("No M3Chains linked for M3Humanoid\n");

	return false;

}

void M3Humanoid::Startup()
{	
	M3Robot::Startup();
	
	
	if(right_arm && right_arm->GetDynamatics())
	{
		for (int i=0; i<right_arm->GetNumDof(); i++)
		{
			status.mutable_right_arm()->add_g(0);

		}
		Vector com = right_arm->GetDynamatics()->GetPayloadCom();
		for (int i=0; i<3; i++)
		{
			status.mutable_right_arm()->add_end_pos(0);
			param.mutable_right_arm()->add_payload_com(com[i]);
		}

		param.mutable_right_arm()->set_payload_mass(right_arm->GetDynamatics()->GetPayloadMass());
		param.mutable_right_arm()->set_use_velocities(((M3DynamaticsParam*)right_arm->GetDynamatics()->GetParam())->use_velocities());
		param.mutable_right_arm()->set_use_accelerations(((M3DynamaticsParam*)right_arm->GetDynamatics()->GetParam())->use_accelerations());

		for (int i=0; i<9; i++)
		{
			status.mutable_right_arm()->add_end_rot(0);
		}
	
		for (int i=0; i<6; i++)
		{
			status.mutable_right_arm()->add_end_twist(0);
			param.mutable_right_arm()->add_payload_inertia(((M3DynamaticsParam*)right_arm->GetDynamatics()->GetParam())->payload_inertia(i));
		}
	
		for (int i=0; i<(6*right_arm->GetNumDof()); i++)
		{
			status.mutable_right_arm()->add_j(0);
		}
		J_right_arm = Jacobian(right_arm->GetNumDof());
		right_arm->GetDynamatics()->SetPriority(ARM_HEAD_DYNAMATICS_PRIORITY);
	}
	if(right_arm)
	{
		for (int i=0; i<right_arm->GetNumDof(); i++)
		{			
			status.mutable_right_arm()->add_torque(0);
			status.mutable_right_arm()->add_torquedot(0);			
			status.mutable_right_arm()->add_theta(0);
			status.mutable_right_arm()->add_thetadot(0);
			status.mutable_right_arm()->add_thetadotdot(0);
			status.mutable_right_arm()->add_pwm_cmd(0);
			command.mutable_right_arm()->add_q_desired(0);	
			command.mutable_right_arm()->add_qdot_desired(0);			
			command.mutable_right_arm()->add_tq_desired(0);
			command.mutable_right_arm()->add_q_stiffness(0);
			command.mutable_right_arm()->add_q_slew_rate(0);
			command.mutable_right_arm()->add_pwm_desired(0);
			command.mutable_right_arm()->add_ctrl_mode(JOINT_ARRAY_MODE_OFF);			
			command.mutable_right_arm()->add_smoothing_mode(SMOOTHING_MODE_OFF);
		}
		torque_shm_right_arm.resize(right_arm->GetNumDof());
	}
	if(left_arm && left_arm->GetDynamatics())
	{					 
		for (int i=0; i<left_arm->GetNumDof(); i++)
		{
			status.mutable_left_arm()->add_g(0);

		}
		Vector com = left_arm->GetDynamatics()->GetPayloadCom();
		for (int i=0; i<3; i++)
		{
			status.mutable_left_arm()->add_end_pos(0);
			param.mutable_left_arm()->add_payload_com(com[i]);			
		}

		param.mutable_left_arm()->set_payload_mass(left_arm->GetDynamatics()->GetPayloadMass());
		param.mutable_left_arm()->set_use_velocities(((M3DynamaticsParam*)left_arm->GetDynamatics()->GetParam())->use_velocities());
		param.mutable_left_arm()->set_use_accelerations(((M3DynamaticsParam*)left_arm->GetDynamatics()->GetParam())->use_accelerations());

		for (int i=0; i<9; i++)
		{
			status.mutable_left_arm()->add_end_rot(0);
		}
	
		for (int i=0; i<6; i++)
		{
			status.mutable_left_arm()->add_end_twist(0);
			param.mutable_left_arm()->add_payload_inertia(((M3DynamaticsParam*)left_arm->GetDynamatics()->GetParam())->payload_inertia(i));
		}
	
		for (int i=0; i<(6*left_arm->GetNumDof()); i++)
		{
			status.mutable_left_arm()->add_j(0);
		}
		J_left_arm = Jacobian(left_arm->GetNumDof());
		left_arm->GetDynamatics()->SetPriority(ARM_HEAD_DYNAMATICS_PRIORITY);
	}
	if(left_arm)
	{
		for (int i=0; i<left_arm->GetNumDof(); i++)
		{			
			status.mutable_left_arm()->add_torque(0);
			status.mutable_left_arm()->add_torquedot(0);			
			status.mutable_left_arm()->add_theta(0);
			status.mutable_left_arm()->add_thetadot(0);
			status.mutable_left_arm()->add_thetadotdot(0);
			status.mutable_left_arm()->add_pwm_cmd(0);
			command.mutable_left_arm()->add_q_desired(0);	
			command.mutable_left_arm()->add_qdot_desired(0);			
			command.mutable_left_arm()->add_tq_desired(0);
			command.mutable_left_arm()->add_q_stiffness(0);
			command.mutable_left_arm()->add_q_slew_rate(0);
			command.mutable_left_arm()->add_pwm_desired(0);
			command.mutable_left_arm()->add_ctrl_mode(JOINT_ARRAY_MODE_OFF);
			command.mutable_left_arm()->add_smoothing_mode(SMOOTHING_MODE_OFF);
		}
		torque_shm_left_arm.resize(left_arm->GetNumDof());
	}
	if(torso && torso->GetDynamatics())
	{			 
		for (int i=0; i<torso->GetNumDof(); i++)
		{
			status.mutable_torso()->add_g(0);			
		}
		Vector com = torso->GetDynamatics()->GetPayloadCom();
		for (int i=0; i<3; i++)
		{
			status.mutable_torso()->add_end_pos(0);
			param.mutable_torso()->add_payload_com(com[i]);		
		}		

		param.mutable_torso()->set_payload_mass(torso->GetDynamatics()->GetPayloadMass());
		param.mutable_torso()->set_use_velocities(((M3DynamaticsParam*)torso->GetDynamatics()->GetParam())->use_velocities());
		param.mutable_torso()->set_use_accelerations(((M3DynamaticsParam*)torso->GetDynamatics()->GetParam())->use_accelerations());

		for (int i=0; i<9; i++)
		{
			status.mutable_torso()->add_end_rot(0);
		}
	
		for (int i=0; i<6; i++)
		{
			status.mutable_torso()->add_end_twist(0);
			param.mutable_torso()->add_payload_inertia(((M3DynamaticsParam*)torso->GetDynamatics()->GetParam())->payload_inertia(i));
		}
	
		for (int i=0; i<(6*torso->GetNumDof()); i++)
		{
			status.mutable_torso()->add_j(0);
		}
		J_torso = Jacobian(torso->GetNumDof());
	}
	if(torso)
	{
		for (int i=0; i<torso->GetNumDof(); i++)
		{			
			status.mutable_torso()->add_torque(0);
			status.mutable_torso()->add_torquedot(0);			
			status.mutable_torso()->add_theta(0);
			status.mutable_torso()->add_thetadot(0);
			status.mutable_torso()->add_thetadotdot(0);
			status.mutable_torso()->add_pwm_cmd(0);
			command.mutable_torso()->add_q_desired(0);
			command.mutable_torso()->add_qdot_desired(0);			
			command.mutable_torso()->add_tq_desired(0);
			command.mutable_torso()->add_q_stiffness(0);
			command.mutable_torso()->add_q_slew_rate(0);
			command.mutable_torso()->add_pwm_desired(0);
			command.mutable_torso()->add_ctrl_mode(JOINT_ARRAY_MODE_OFF);
			command.mutable_torso()->add_smoothing_mode(SMOOTHING_MODE_OFF);
		}
		torque_shm_torso.resize(torso->GetNumDof());
	}

	if(head && head->GetDynamatics())
	{	
		for (int i=0; i<head->GetDynamatics()->GetNumDof(); i++)//head->GetNumDof()
		{
			status.mutable_head()->add_g(0);			
		}
		Vector com = head->GetDynamatics()->GetPayloadCom();
		for (int i=0; i<3; i++)
		{
			status.mutable_head()->add_end_pos(0);
			param.mutable_head()->add_payload_com(com[i]);
			status.add_left_eye_pos(0);
			status.add_right_eye_pos(0);
		}
	
		param.mutable_head()->set_payload_mass(head->GetDynamatics()->GetPayloadMass());
		param.mutable_head()->set_use_velocities(((M3DynamaticsParam*)head->GetDynamatics()->GetParam())->use_velocities());
		param.mutable_head()->set_use_accelerations(((M3DynamaticsParam*)head->GetDynamatics()->GetParam())->use_accelerations());
	
		for (int i=0; i<9; i++)
		{
			status.mutable_head()->add_end_rot(0);
			status.add_left_eye_rot(0);
			status.add_right_eye_rot(0);
		}
	
		for (int i=0; i<6; i++)
		{
			status.mutable_head()->add_end_twist(0);
			param.mutable_head()->add_payload_inertia(((M3DynamaticsParam*)head->GetDynamatics()->GetParam())->payload_inertia(i));
		}
	
		for (int i=0; i<(6*head->GetDynamatics()->GetNumDof()); i++)//head->GetNumDof()
		{
			status.mutable_head()->add_j(0);
		}
		J_head = Jacobian(head->GetNumDof());
		head->GetDynamatics()->SetPriority(ARM_HEAD_DYNAMATICS_PRIORITY);
	}
	if(head)
	{
		for (int i=0; i<head->GetNumDof(); i++)
		{			
			status.mutable_head()->add_torque(0);
			status.mutable_head()->add_torquedot(0);
			status.mutable_head()->add_theta(0);
			status.mutable_head()->add_thetadot(0);
			status.mutable_head()->add_thetadotdot(0);
			status.mutable_head()->add_pwm_cmd(0);
			command.mutable_head()->add_q_desired(0);
			command.mutable_head()->add_qdot_desired(0);
			command.mutable_head()->add_tq_desired(0);
			command.mutable_head()->add_q_stiffness(0);
			command.mutable_head()->add_q_slew_rate(0);
			command.mutable_head()->add_pwm_desired(0);
			command.mutable_head()->add_ctrl_mode(JOINT_ARRAY_MODE_OFF);
			command.mutable_head()->add_smoothing_mode(SMOOTHING_MODE_OFF);
		}
		left_eye_offset = head->GetLeftEyeOffset();
		right_eye_offset = head->GetRightEyeOffset();
		angle_shm_head.resize(head->GetNumDof());
		slew_rate_shm_head.resize(head->GetNumDof());
	}

	if(startup_motor_pwr_on)
	  SetMotorPowerOn();
	SetStateSafeOp();
	
}



void M3Humanoid::Shutdown()
{

}


void M3Humanoid::StepCommand()
{		
	if (IsStateSafeOp() || IsStateError())
		return;		

	pwr->SetMotorEnable(command.enable_motor());
	
	if(torso)
	{
		for (int i=0; i<torso->GetNumDof(); i++)
		{
		     if (command.torso().ctrl_mode(i) == JOINT_ARRAY_MODE_TORQUE_SHM || force_shm_torso)
		     {
		       if (enable_shm_torso)
		       {
			((M3JointArrayCommand*)torso->GetCommand())->set_tq_desired(i, torque_shm_torso(i));
			((M3JointArrayCommand*)torso->GetCommand())->set_ctrl_mode(i, JOINT_ARRAY_MODE_TORQUE);	
		       } else {
			 ((M3JointArrayCommand*)torso->GetCommand())->set_tq_desired(i, 0);
			 ((M3JointArrayCommand*)torso->GetCommand())->set_ctrl_mode(i, JOINT_ARRAY_MODE_OFF);	
		       }
		     } else {		       
			((M3JointArrayCommand*)torso->GetCommand())->set_tq_desired(i, command.torso().tq_desired(i));						
			((M3JointArrayCommand*)torso->GetCommand())->set_ctrl_mode(i, command.torso().ctrl_mode(i));				
		     }		     
		     ((M3JointArrayCommand*)torso->GetCommand())->set_smoothing_mode(i, command.torso().smoothing_mode(i));				
		    ((M3JointArrayCommand*)torso->GetCommand())->set_q_stiffness(i, command.torso().q_stiffness(i));		    
		    ((M3JointArrayCommand*)torso->GetCommand())->set_q_desired(i, command.torso().q_desired(i));
		    ((M3JointArrayCommand*)torso->GetCommand())->set_qdot_desired(i, command.torso().qdot_desired(i));
		    ((M3JointArrayCommand*)torso->GetCommand())->set_q_slew_rate(i, command.torso().q_slew_rate(i));
		    ((M3JointArrayCommand*)torso->GetCommand())->set_pwm_desired(i, command.torso().pwm_desired(i));
		}
		for (int i=0;i<command.torso().vias_size();i++)
		{
			M3JointVia * via;			
			via = ((M3JointArrayCommand*)torso->GetCommand())->add_vias();
			*via = command.torso().vias(i);
		}
		command.mutable_torso()->clear_vias();
	}

	if(head)
	{
		for (int i=0; i<head->GetNumDof(); i++)
		{
		    if (command.head().ctrl_mode(i) == JOINT_ARRAY_MODE_ANGLE_SHM || force_shm_head)
		    {
		       if (enable_shm_head)
		       {
			((M3JointArrayCommand*)head->GetCommand())->set_q_desired(i, angle_shm_head(i));
			((M3JointArrayCommand*)head->GetCommand())->set_ctrl_mode(i, JOINT_ARRAY_MODE_THETA);	
			((M3JointArrayCommand*)head->GetCommand())->set_q_slew_rate(i, slew_rate_shm_head(i));		
		       } else {
			 //((M3JointArrayCommand*)head->GetCommand())->set_q_desired(i, 0);
			 //((M3JointArrayCommand*)head->GetCommand())->set_q_slew_rate(i, command.head().q_slew_rate(i));
			 ((M3JointArrayCommand*)head->GetCommand())->set_ctrl_mode(i, JOINT_ARRAY_MODE_OFF);	
		       }
		     } else {		       
			((M3JointArrayCommand*)head->GetCommand())->set_q_desired(i, command.head().q_desired(i));			
			((M3JointArrayCommand*)head->GetCommand())->set_q_slew_rate(i, command.head().q_slew_rate(i));			
			((M3JointArrayCommand*)head->GetCommand())->set_ctrl_mode(i, command.head().ctrl_mode(i));	
		     }		
		    ((M3JointArrayCommand*)head->GetCommand())->set_smoothing_mode(i, command.head().smoothing_mode(i));	  
		    ((M3JointArrayCommand*)head->GetCommand())->set_tq_desired(i, command.head().tq_desired(i));		    
		    ((M3JointArrayCommand*)head->GetCommand())->set_q_stiffness(i, command.head().q_stiffness(i));
		    ((M3JointArrayCommand*)head->GetCommand())->set_qdot_desired(i, command.head().qdot_desired(i));		    
		    ((M3JointArrayCommand*)head->GetCommand())->set_pwm_desired(i, command.head().pwm_desired(i));
		}
		for (int i=0;i<command.head().vias_size();i++)
		{
			M3JointVia * via;			
			via = ((M3JointArrayCommand*)head->GetCommand())->add_vias();
			*via = command.head().vias(i);
		}
		command.mutable_head()->clear_vias();
	}
	
	if(right_arm)
	{
		for (int i=0; i<right_arm->GetNumDof(); i++)
		{
		    if (command.right_arm().ctrl_mode(i) == JOINT_ARRAY_MODE_TORQUE_SHM || force_shm_r_arm)
		    {
			if (enable_shm_r_arm)
			{
			  ((M3JointArrayCommand*)right_arm->GetCommand())->set_tq_desired(i, torque_shm_right_arm(i));	
			  ((M3JointArrayCommand*)right_arm->GetCommand())->set_ctrl_mode(i, JOINT_ARRAY_MODE_TORQUE);			
			} else {
			  ((M3JointArrayCommand*)right_arm->GetCommand())->set_tq_desired(i, 0);	
			  ((M3JointArrayCommand*)right_arm->GetCommand())->set_ctrl_mode(i, JOINT_ARRAY_MODE_OFF);	
			}
		    } else {
			((M3JointArrayCommand*)right_arm->GetCommand())->set_tq_desired(i, command.right_arm().tq_desired(i));						
			((M3JointArrayCommand*)right_arm->GetCommand())->set_ctrl_mode(i, command.right_arm().ctrl_mode(i));	
						
		    }		    
		    ((M3JointArrayCommand*)right_arm->GetCommand())->set_smoothing_mode(i, command.right_arm().smoothing_mode(i));
		    ((M3JointArrayCommand*)right_arm->GetCommand())->set_q_stiffness(i, command.right_arm().q_stiffness(i));		    
		    ((M3JointArrayCommand*)right_arm->GetCommand())->set_q_desired(i, command.right_arm().q_desired(i));			
		    ((M3JointArrayCommand*)right_arm->GetCommand())->set_qdot_desired(i, command.right_arm().qdot_desired(i));
		    ((M3JointArrayCommand*)right_arm->GetCommand())->set_q_slew_rate(i, command.right_arm().q_slew_rate(i));
		    ((M3JointArrayCommand*)right_arm->GetCommand())->set_pwm_desired(i, command.right_arm().pwm_desired(i));
		}
		for (int i=0;i<command.right_arm().vias_size();i++)
		{
			M3JointVia * via;			
			via = ((M3JointArrayCommand*)right_arm->GetCommand())->add_vias();
			*via = command.right_arm().vias(i);
		}
		command.mutable_right_arm()->clear_vias();
	}

	if(left_arm)
	{
		for (int i=0; i<left_arm->GetNumDof(); i++)
		{
		    if (command.left_arm().ctrl_mode(i) == JOINT_ARRAY_MODE_TORQUE_SHM || force_shm_l_arm)
		    {
		      if (enable_shm_l_arm)
			{
			  ((M3JointArrayCommand*)left_arm->GetCommand())->set_tq_desired(i, torque_shm_left_arm(i));	
			  ((M3JointArrayCommand*)left_arm->GetCommand())->set_ctrl_mode(i, JOINT_ARRAY_MODE_TORQUE);	
			} else {
			  ((M3JointArrayCommand*)left_arm->GetCommand())->set_tq_desired(i, 0);	
			  ((M3JointArrayCommand*)left_arm->GetCommand())->set_ctrl_mode(i, JOINT_ARRAY_MODE_OFF);	
			}
		    } else {
			((M3JointArrayCommand*)left_arm->GetCommand())->set_tq_desired(i, command.left_arm().tq_desired(i));						
			((M3JointArrayCommand*)left_arm->GetCommand())->set_ctrl_mode(i, command.left_arm().ctrl_mode(i));			
		    }	
		    ((M3JointArrayCommand*)left_arm->GetCommand())->set_smoothing_mode(i, command.left_arm().smoothing_mode(i));
		    ((M3JointArrayCommand*)left_arm->GetCommand())->set_q_stiffness(i, command.left_arm().q_stiffness(i));		    
		    ((M3JointArrayCommand*)left_arm->GetCommand())->set_q_desired(i, command.left_arm().q_desired(i));
		    ((M3JointArrayCommand*)left_arm->GetCommand())->set_qdot_desired(i, command.left_arm().qdot_desired(i));
		    ((M3JointArrayCommand*)left_arm->GetCommand())->set_q_slew_rate(i, command.left_arm().q_slew_rate(i));
		    ((M3JointArrayCommand*)left_arm->GetCommand())->set_pwm_desired(i, command.left_arm().pwm_desired(i));
		}
		for (int i=0;i<command.left_arm().vias_size();i++)
		{
			M3JointVia * via;			
			via = ((M3JointArrayCommand*)left_arm->GetCommand())->add_vias();
			*via = command.left_arm().vias(i);
		}
		command.mutable_left_arm()->clear_vias();
	}

}

void M3Humanoid::StepStatus()
{	
	if (IsStateError())
		return;		

	grav_end_torso[0] = 0;
	grav_end_torso[1] = 0;
	grav_end_torso[2] = 0;
	
////////////////////////////////////////////////////////
////  1> Copy status from chains and dynamatics to humanoid status
////  2> Transform End Frame, Twist, and Jacobian values for each chain
////       to the world reference frame
////  3> Set gravity value for chains depending on each chain's base link orientation in world frame
////  4> Apply correct Wrenches at Torso's End Frame to compensate for arm/head torques
///////////////////////////////////////////////////////
////  chain => parent offsets are defined in humanoid yaml file using base_rotation_in_parent
////			and base_translation_in_parent
////  For a humanoid with torso, arms, and head offsets are defined as follows:
////  torso_offset = Transform from torso_T_0 => world_origin
////  right_arm_offset = Transform from right_arm_T_0 => torso_T_4
////  left_arm_offset = Transform from left_arm_T_0 => torso_T_4
////  head_offset = Transform from head_T_0 => torso_T_4
///////////////////////////////////////////////////////
////  For a humanoid without torso:
////  right_arm_offset = Transform from right_arm_T_0 => world_origin
////  left_arm_offset = Transform from left_arm_T_0 => world_origin
////  head_offset = Transform from head_T_0 => world_origin
///////////////////////////////////////////////////////


	if(torso)
	{
		for (int i=0; i<torso->GetNumDof(); i++)
		{			
			status.mutable_torso()->set_torque(i,((M3JointArrayStatus*)torso->GetStatus())->torque(i));
			status.mutable_torso()->set_torquedot(i,((M3JointArrayStatus*)torso->GetStatus())->torquedot(i));
			status.mutable_torso()->set_theta(i,((M3JointArrayStatus*)torso->GetStatus())->theta(i));
			status.mutable_torso()->set_thetadot(i,((M3JointArrayStatus*)torso->GetStatus())->thetadot(i));
			status.mutable_torso()->set_thetadotdot(i,((M3JointArrayStatus*)torso->GetStatus())->thetadotdot(i));
			status.mutable_torso()->set_pwm_cmd(i,((M3JointArrayStatus*)torso->GetStatus())->pwm_cmd(i));
		}
		status.mutable_torso()->set_completed_spline_idx(((M3JointArrayStatus*)torso->GetStatus())->completed_spline_idx());

		if(torso->GetDynamatics())
		{
			for (int i=0; i<3; i++)
				((M3DynamaticsParam*)torso->GetDynamatics()->GetParam())->set_payload_com(i,
					 param.torso().payload_com(i));
			for (int i=0; i<6; i++)
				((M3DynamaticsParam*)torso->GetDynamatics()->GetParam())->set_payload_inertia(i,
					 param.torso().payload_inertia(i));
			((M3DynamaticsParam*)torso->GetDynamatics()->GetParam())->set_payload_mass(param.torso().payload_mass());
		}
	}
	
	if(torso && torso->GetDynamatics())
	{
		for (int i=0; i<6; i++)
		{					
			for (int j=0; j<torso->GetDynamatics()->GetNumDof(); j++)		
				J_torso(i,j) =
				 ((M3DynamaticsStatus*)torso->GetDynamatics()->GetStatus())->j((i*torso->GetDynamatics()->GetNumDof())+(j));
		}

		for (int i=0; i<torso->GetDynamatics()->GetNumDof(); i++)		
			status.mutable_torso()->set_g(i,((M3DynamaticsStatus*)torso->GetDynamatics()->GetStatus())->g(i));
					
		// torso_end_frame will now be Torso_T4 => World_Origin
		torso_end_frame = torso_offset * Frame(torso->GetDynamatics()->GetEndRot(),torso->GetDynamatics()->GetEndPos());
		// grav_end_torso now gravity vector in Torso_T4 frame
		grav_end_torso = torso_end_frame.M.Inverse() * grav_world;
		torso_twist = torso_offset*torso->GetDynamatics()->GetEndTwist();
		J_torso.changeRefFrame(torso_offset);
	}
	
	if (right_arm)
	{
		for (int i=0; i<right_arm->GetNumDof(); i++)
		{			
			status.mutable_right_arm()->set_torque(i,((M3JointArrayStatus*)right_arm->GetStatus())->torque(i));
			status.mutable_right_arm()->set_torquedot(i,((M3JointArrayStatus*)right_arm->GetStatus())->torquedot(i));
			status.mutable_right_arm()->set_theta(i,((M3JointArrayStatus*)right_arm->GetStatus())->theta(i));
			status.mutable_right_arm()->set_thetadot(i,((M3JointArrayStatus*)right_arm->GetStatus())->thetadot(i));
			status.mutable_right_arm()->set_thetadotdot(i,((M3JointArrayStatus*)right_arm->GetStatus())->thetadotdot(i));
			status.mutable_right_arm()->set_pwm_cmd(i,((M3JointArrayStatus*)right_arm->GetStatus())->pwm_cmd(i));
		}
		status.mutable_right_arm()->set_completed_spline_idx(((M3JointArrayStatus*)right_arm->GetStatus())->completed_spline_idx());

		if(right_arm->GetDynamatics())
		{
			for (int i=0; i<3; i++)
				((M3DynamaticsParam*)right_arm->GetDynamatics()->GetParam())->set_payload_com(i,
					 param.right_arm().payload_com(i));
			for (int i=0; i<6; i++)
				((M3DynamaticsParam*)right_arm->GetDynamatics()->GetParam())->set_payload_inertia(i,
					 param.right_arm().payload_inertia(i));
			((M3DynamaticsParam*)right_arm->GetDynamatics()->GetParam())->set_payload_mass(param.right_arm().payload_mass());
		}
	}
		
	if(right_arm && right_arm->GetDynamatics())
	{
		for (int i=0; i<right_arm->GetDynamatics()->GetNumDof(); i++)		
			status.mutable_right_arm()->set_g(i,((M3DynamaticsStatus*)right_arm->GetDynamatics()->GetStatus())->g(i));
				
		for (int i=0; i<6; i++)
		{					
			for (int j=0; j<right_arm->GetDynamatics()->GetNumDof(); j++)		
				J_right_arm(i,j) =
				 ((M3DynamaticsStatus*)right_arm->GetDynamatics()->GetStatus())->j((i*right_arm->GetDynamatics()->GetNumDof())+(j));
		}
		// right_arm_end_frame will now be right_arm_T8 => right_arm_base
		right_arm_end_frame = right_arm_offset *
				 Frame(right_arm->GetDynamatics()->GetEndRot(),right_arm->GetDynamatics()->GetEndPos());
		right_arm_twist = right_arm_offset * right_arm->GetDynamatics()->GetEndTwist();
		J_right_arm.changeRefFrame(right_arm_offset);
		if(torso && torso->GetDynamatics())
		{
			// right_arm_end_frame will now be right_arm_T8 => world_origin
			right_arm_end_frame = torso_end_frame * right_arm_end_frame;
			// setting grav vector in right_arm_T0
			right_arm->GetDynamatics()->SetGrav(right_arm_offset.M.Inverse() * grav_end_torso);
			right_arm_twist = torso_end_frame * right_arm_twist;
			J_right_arm.changeRefFrame(torso_end_frame);
		} else 
			right_arm->GetDynamatics()->SetGrav(right_arm_offset.M.Inverse() * grav_world);		
	}

	if(left_arm)
	{
		for (int i=0; i<left_arm->GetNumDof(); i++)
		{			
			status.mutable_left_arm()->set_torque(i,((M3JointArrayStatus*)left_arm->GetStatus())->torque(i));
			status.mutable_left_arm()->set_torquedot(i,((M3JointArrayStatus*)left_arm->GetStatus())->torquedot(i));
			status.mutable_left_arm()->set_theta(i,((M3JointArrayStatus*)left_arm->GetStatus())->theta(i));
			status.mutable_left_arm()->set_thetadot(i,((M3JointArrayStatus*)left_arm->GetStatus())->thetadot(i));
			status.mutable_left_arm()->set_thetadotdot(i,((M3JointArrayStatus*)left_arm->GetStatus())->thetadotdot(i));
			status.mutable_left_arm()->set_pwm_cmd(i,((M3JointArrayStatus*)left_arm->GetStatus())->pwm_cmd(i));
		}
		status.mutable_left_arm()->set_completed_spline_idx(((M3JointArrayStatus*)left_arm->GetStatus())->completed_spline_idx());

		if(left_arm->GetDynamatics())
		{
			for (int i=0; i<3; i++)
				((M3DynamaticsParam*)left_arm->GetDynamatics()->GetParam())->set_payload_com(i,
					 param.left_arm().payload_com(i));
			for (int i=0; i<6; i++)
				((M3DynamaticsParam*)left_arm->GetDynamatics()->GetParam())->set_payload_inertia(i,
					 param.left_arm().payload_inertia(i));
			((M3DynamaticsParam*)left_arm->GetDynamatics()->GetParam())->set_payload_mass(param.left_arm().payload_mass());
		}
	}
		
	if(left_arm && left_arm->GetDynamatics())
	{
		for (int i=0; i<6; i++)
		{					
			for (int j=0; j<left_arm->GetDynamatics()->GetNumDof(); j++)		
				J_left_arm(i,j) =
				 ((M3DynamaticsStatus*)left_arm->GetDynamatics()->GetStatus())->j((i*left_arm->GetDynamatics()->GetNumDof())+(j));
		}

		for (int i=0; i<left_arm->GetDynamatics()->GetNumDof(); i++)
		{
			status.mutable_left_arm()->set_g(i,((M3DynamaticsStatus*)left_arm->GetDynamatics()->GetStatus())->g(i));			
		}					
		// left_arm_end_frame will now be left_arm_T8 => left_arm_base
		left_arm_end_frame = left_arm_offset *
				 Frame(left_arm->GetDynamatics()->GetEndRot(),left_arm->GetDynamatics()->GetEndPos());
		left_arm_twist = left_arm_offset * left_arm->GetDynamatics()->GetEndTwist();
		J_left_arm.changeRefFrame(left_arm_offset);
		if(torso && torso->GetDynamatics())
		{
			// left_arm_end_frame will now be left_arm_T8 => world_origin
			left_arm_end_frame = torso_end_frame * left_arm_end_frame;
			// setting grav vector in left_arm_T0
			left_arm->GetDynamatics()->SetGrav(left_arm_offset.M.Inverse() * grav_end_torso);
			left_arm_twist = torso_end_frame * left_arm_twist;
			J_left_arm.changeRefFrame(torso_end_frame);
		} else left_arm->GetDynamatics()->SetGrav(left_arm_offset.M.Inverse() * grav_world);
	}
	
	if(head)
	{
		for (int i=0; i<head->GetNumDof(); i++)
		{			
			status.mutable_head()->set_torque(i,((M3JointArrayStatus*)head->GetStatus())->torque(i));
			status.mutable_head()->set_torquedot(i,((M3JointArrayStatus*)head->GetStatus())->torquedot(i));
			status.mutable_head()->set_theta(i,((M3JointArrayStatus*)head->GetStatus())->theta(i));
			status.mutable_head()->set_thetadot(i,((M3JointArrayStatus*)head->GetStatus())->thetadot(i));
			status.mutable_head()->set_thetadotdot(i,((M3JointArrayStatus*)head->GetStatus())->thetadotdot(i));
			status.mutable_head()->set_pwm_cmd(i,((M3JointArrayStatus*)head->GetStatus())->pwm_cmd(i));
		}
		status.mutable_head()->set_completed_spline_idx(((M3JointArrayStatus*)head->GetStatus())->completed_spline_idx());
		
		if(head->GetDynamatics())
		{
			for (int i=0; i<3; i++)
				((M3DynamaticsParam*)head->GetDynamatics()->GetParam())->set_payload_com(i,
					 param.head().payload_com(i));
			for (int i=0; i<6; i++)
				((M3DynamaticsParam*)head->GetDynamatics()->GetParam())->set_payload_inertia(i,
					 param.head().payload_inertia(i));
			((M3DynamaticsParam*)head->GetDynamatics()->GetParam())->set_payload_mass(param.head().payload_mass());
		}
	}
		
	if(head && head->GetDynamatics())
	{	
		for (int i=0; i<head->GetDynamatics()->GetNumDof(); i++)		
			status.mutable_head()->set_g(i,((M3DynamaticsStatus*)head->GetDynamatics()->GetStatus())->g(i));			
			
		for (int i=0; i<6; i++)
		{					
			for (int j=0; j<head->GetDynamatics()->GetNumDof(); j++)		
				J_head(i,j) =
				 ((M3DynamaticsStatus*)head->GetDynamatics()->GetStatus())->j((i*head->GetDynamatics()->GetNumDof())+(j));
		}
		// head_end_frame will now be head_T5 => head_base
		head_end_frame = head_offset *
				 Frame(head->GetDynamatics()->GetEndRot(),head->GetDynamatics()->GetEndPos());
		head_twist = head_offset * head->GetDynamatics()->GetEndTwist();
		J_head.changeRefFrame(head_offset);
		
		if(torso && torso->GetDynamatics())
		{
			// head_end_frame will now be head_T5 => world_origin
			head_base_2_world_frame=torso_end_frame*head_offset;
			head_end_frame = torso_end_frame * head_end_frame;
			// setting grav vector in head_T0
			head->GetDynamatics()->SetGrav(head_offset.M.Inverse() * grav_end_torso);			
			head_twist = torso_end_frame * head_twist;
			J_head.changeRefFrame(torso_end_frame);
			
		} else head->GetDynamatics()->SetGrav(head_offset.M.Inverse() * grav_world);

////////////////////////////
/////// Eyes (*)--(*) //////
////////////////////////////
		
		Frame T_left;
		Frame T_right;

		T_right = Frame(Rotation::RotZ(head->GetThetaRad(5)), Vector());
		T_left = Frame(Rotation::RotZ(head->GetThetaRad(6)),Vector());
		// right_eye = head_T6								
		right_eye_2_world_frame = head_end_frame * right_eye_offset * T_right;
		
		// left_eye = head_T6
		left_eye_2_world_frame = head_end_frame * left_eye_offset * T_left;
		
		//Get eye down to base of head
		/*if (torso)
		{
		  right_eye_2_head_base_frame=  torso_end_frame.M.Inverse()*right_eye_2_world_frame;
		  left_eye_2_head_base_frame=  torso_end_frame.M.Inverse()*left_eye_2_world_frame;
		}
		else
		{
		  right_eye_2_head_base_frame=right_eye_2_world_frame;
		  left_eye_2_head_base_frame=left_eye_2_world_frame;
		}*/
		

	}

///////////////////////////////////////////////////////////////
////   Copy the tranformed values into humanoid status
///////////////////////////////////////////////////////////////
	for (int i=0; i<6; i++)
	{
		if(right_arm && right_arm->GetDynamatics())
		{
			for (int j=0; j<right_arm->GetDynamatics()->GetNumDof(); j++)
				status.mutable_right_arm()->set_j((i*right_arm->GetDynamatics()->GetNumDof())+(j), J_right_arm(i,j));
			if(i<3)
				status.mutable_right_arm()->set_end_twist(i,right_arm_twist.vel[i]);
			else
				status.mutable_right_arm()->set_end_twist(i,right_arm_twist.rot[i-3]);
		}
		if(left_arm && left_arm->GetDynamatics())
		{
			for (int j=0; j<left_arm->GetDynamatics()->GetNumDof(); j++)
				status.mutable_left_arm()->set_j((i*left_arm->GetDynamatics()->GetNumDof())+(j), J_left_arm(i,j));
			if(i<3)
				status.mutable_left_arm()->set_end_twist(i,left_arm_twist.vel[i]);
			else
				status.mutable_left_arm()->set_end_twist(i,left_arm_twist.rot[i-3]);
		}
		if(head && head->GetDynamatics())
		{
			for (int j=0; j<head->GetDynamatics()->GetNumDof(); j++)
				status.mutable_head()->set_j((i*head->GetDynamatics()->GetNumDof())+(j), J_head(i,j));
			if(i<3)
				status.mutable_head()->set_end_twist(i,head_twist.vel[i]);
			else
				status.mutable_head()->set_end_twist(i,head_twist.rot[i-3]);
		}
		if(torso && torso->GetDynamatics())
		{
			for (int j=0; j<torso->GetDynamatics()->GetNumDof(); j++)
				status.mutable_torso()->set_j((i*torso->GetDynamatics()->GetNumDof())+(j), J_torso(i,j));
			if(i<3)
				status.mutable_torso()->set_end_twist(i,torso_twist.vel[i]);
			else
				status.mutable_torso()->set_end_twist(i,torso_twist.rot[i-3]);
		}
	}
	for (int i=0; i<3; i++)
	{
		if(right_arm && right_arm->GetDynamatics())
			status.mutable_right_arm()->set_end_pos(i,right_arm_end_frame.p[i]);
		if(left_arm && left_arm->GetDynamatics())
			status.mutable_left_arm()->set_end_pos(i,left_arm_end_frame.p[i]);
		if(torso && torso->GetDynamatics())
			status.mutable_torso()->set_end_pos(i,torso_end_frame.p[i]);
		if(head && head->GetDynamatics())
		{
			status.set_right_eye_pos(i, right_eye_2_world_frame.p[i]);
			status.set_left_eye_pos(i, left_eye_2_world_frame.p[i]);
			status.mutable_head()->set_end_pos(i,head_end_frame.p[i]);
		}
	}
	for (int i=0; i<9; i++)
	{
		if(right_arm && right_arm->GetDynamatics())
			status.mutable_right_arm()->set_end_rot(i,right_arm_end_frame.M.data[i]);
		if(left_arm && left_arm->GetDynamatics())
			status.mutable_left_arm()->set_end_rot(i,left_arm_end_frame.M.data[i]);
		if(torso && torso->GetDynamatics())
			status.mutable_torso()->set_end_rot(i,torso_end_frame.M.data[i]);
		if(head && head->GetDynamatics())
		{
			status.set_right_eye_rot(i, right_eye_2_world_frame.M.data[i]);
			status.set_left_eye_rot(i, left_eye_2_world_frame.M.data[i]);
			status.mutable_head()->set_end_rot(i,head_end_frame.M.data[i]);
		}
	}
	
/////////////////////////////////////////////////////
// Add wrenches to torso from child chains
////////////////////////////////////////////////////
// The dynamic payloads from arms, head are found using the arm and head dynamatics component to generate equivalent 
// wrenches at each components T0 Frame.  
// A dummy joint is assigned at T0 in X, Y, and Z direction and the torque values there are the 
// torque component for that chain's wrench at T0.  The force component is mass*gravity.  This wrench gets transformed 
// to Torso_T4 and added as an external wrench for the torso's dynamatics component.
///////////////////////////////////////////////////////

	if (torso && torso->GetDynamatics())
	{	
		torso_wrench = Wrench();  // init to Zero
		// Transforming wrenches to Torso_T4
		if(right_arm && right_arm->GetDynamatics())
			torso_wrench+=right_arm_offset*right_arm->GetDynamatics()->GetBaseWrench();
		/*M3_INFO("Right arm: %f %f %f     %f %f %f\n",
			torso_wrench[0],
			torso_wrench[1],
			torso_wrench[2],
			torso_wrench[3],
			torso_wrench[4],
			torso_wrench[5]);*/
		if(left_arm && left_arm->GetDynamatics())			
			torso_wrench+=left_arm_offset*left_arm->GetDynamatics()->GetBaseWrench();

		if(head && head->GetDynamatics())					
			torso_wrench+=head_offset*head->GetDynamatics()->GetBaseWrench();
				
		torso->GetDynamatics()->SetEndWrench(torso_wrench);
	}	
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool M3Humanoid::ReadConfig(const char * filename)
{	
	if (!M3Robot::ReadConfig(filename))
		return false;

	YAML::Node doc;
	GetYamlDoc(filename, doc);
	
	const YAML::Node * ra_node;
	const YAML::Node * la_node;
	const YAML::Node * t_node;
	const YAML::Node * h_node;
		
	try {
		ra_node = &(doc["chains"]["right_arm"]);
	} catch(YAML::KeyNotFound& e) {
		ra_node = NULL;
	}
	
	try {
		la_node = &(doc["chains"]["left_arm"]);
	} catch(YAML::KeyNotFound& e) {
		la_node = NULL;
	}
	
	try {
		t_node = &(doc["chains"]["torso"]);
	} catch(YAML::KeyNotFound& e) {
		t_node = NULL;
	}
	
	try {
		h_node = &(doc["chains"]["head"]);
	} catch(YAML::KeyNotFound& e) {		
		h_node = NULL;
	}
	
	vector<mReal> ra_rot;
	vector<mReal> la_rot;
	vector<mReal> t_rot;
	vector<mReal> h_rot;
	vector<mReal> ra_trans;
	vector<mReal> la_trans;
	vector<mReal> t_trans;
	vector<mReal> h_trans;

	if (ra_node)
	{	
		(*ra_node)["chain_component"] >> right_arm_name;		
		ra_rot = YamlReadVectorM((*ra_node)["base_rotation_in_parent"]);		
		ra_trans = YamlReadVectorM((*ra_node)["base_translation_in_parent"]);
		try {
		  (*ra_node)["force_shm_mode"] >> force_shm_r_arm;
		} catch(YAML::KeyNotFound& e) {		
			force_shm_r_arm = false;
		}
		
	}
	if (la_node)
	{	
		(*la_node)["chain_component"] >> left_arm_name;
		la_rot = YamlReadVectorM((*la_node)["base_rotation_in_parent"]);
		la_trans = YamlReadVectorM((*la_node)["base_translation_in_parent"]);
		try {
		  (*la_node)["force_shm_mode"] >> force_shm_l_arm;
		} catch(YAML::KeyNotFound& e) {		
			force_shm_l_arm = false;
		}		
	}
	if (t_node)
	{	
		(*t_node)["chain_component"] >> torso_name;
		t_rot = YamlReadVectorM((*t_node)["base_rotation_in_parent"]);
		t_trans = YamlReadVectorM((*t_node)["base_translation_in_parent"]);
		try {
		  (*t_node)["force_shm_mode"] >>force_shm_torso;
		} catch(YAML::KeyNotFound& e) {		
			force_shm_torso = false;
		}
	}
	if (h_node)
	{	
		(*h_node)["chain_component"] >> head_name;
		h_rot = YamlReadVectorM((*h_node)["base_rotation_in_parent"]);
		h_trans = YamlReadVectorM((*h_node)["base_translation_in_parent"]);
		try {
		  (*h_node)["force_shm_mode"] >>force_shm_head;
		} catch(YAML::KeyNotFound& e) {		
			force_shm_head = false;
		}

	}
	
	try{
	  doc["startup_motor_pwr_on"]>>startup_motor_pwr_on;
	}
	catch(YAML::KeyNotFound& e)
	{
	  startup_motor_pwr_on=false;
	}
	
	Rotation ra_rot_kdl;
	Vector ra_vec_kdl;
	Rotation la_rot_kdl;
	Vector la_vec_kdl;
	Rotation t_rot_kdl;
	Vector t_vec_kdl;
	Rotation h_rot_kdl;
	Vector h_vec_kdl;

	
	for(int i=0;i<ra_rot.size();i++) ra_rot_kdl.data[i] = ra_rot[i];
	for(int i=0;i<ra_trans.size();i++) ra_vec_kdl[i] = ra_trans[i];
	for(int i=0;i<la_rot.size();i++) la_rot_kdl.data[i] = la_rot[i];
	for(int i=0;i<la_trans.size();i++) la_vec_kdl[i] = la_trans[i];
	for(int i=0;i<t_rot.size();i++) t_rot_kdl.data[i] = t_rot[i];
	for(int i=0;i<t_trans.size();i++) t_vec_kdl[i] = t_trans[i];
	for(int i=0;i<h_rot.size();i++) h_rot_kdl.data[i] = h_rot[i];
	for(int i=0;i<h_trans.size();i++) h_vec_kdl[i] = h_trans[i];

	right_arm_offset=Frame(ra_rot_kdl, ra_vec_kdl);
	left_arm_offset=Frame(la_rot_kdl, la_vec_kdl);
	torso_offset=Frame(t_rot_kdl, t_vec_kdl);


	head_offset=Frame(h_rot_kdl, h_vec_kdl);

	return true;
}



M3BaseHumanoidCommand * M3Humanoid::GetCmd(M3Chain chain)
{
  switch (chain)
  {
    case RIGHT_ARM:
      return command.mutable_right_arm();
    case LEFT_ARM:
      return command.mutable_left_arm();

    case TORSO:
      return command.mutable_torso();
    case HEAD:
      return command.mutable_head();      
  }
  return NULL;
}

const M3BaseHumanoidStatus * M3Humanoid::GetStatus(M3Chain chain)
{
  switch (chain)
  {
    case RIGHT_ARM:
      return &status.right_arm();
    case LEFT_ARM:
      return &status.left_arm();
    case TORSO:
      return &status.torso();
    case HEAD:
      return &status.head();      
  }
  return NULL;

}

M3BaseHumanoidParam * M3Humanoid::GetParam(M3Chain chain)
{
  switch (chain)
  {
    case RIGHT_ARM:
      return param.mutable_right_arm();
    case LEFT_ARM:
      return param.mutable_left_arm();
    case TORSO:
      return param.mutable_torso();
    case HEAD:
      return param.mutable_head();      
  }
  return NULL;

}

void M3Humanoid::SetThetaDeg(M3Chain chain,unsigned int idx, mReal theta)
{
   M3BaseHumanoidCommand * cmd = GetCmd(chain);
   if (cmd == NULL)
     return;
   if (idx < cmd->q_desired_size())
     cmd->set_q_desired(idx,theta);
}

mReal M3Humanoid::GetThetaDeg(M3Chain chain,unsigned int  idx)
{
  const M3BaseHumanoidStatus * status = GetStatus(chain);
   if (status == NULL)
     return 0;
   if (idx < status->theta_size())
     return status->theta(idx);
   else
     return 0;
}

JOINT_ARRAY_MODE M3Humanoid::GetMode(M3Chain chain,unsigned int  idx)
{
 M3BaseHumanoidCommand * cmd = GetCmd(chain);
   if (cmd == NULL)
     return JOINT_ARRAY_MODE_OFF;
   if (idx < cmd->ctrl_mode_size())
     return cmd->ctrl_mode(idx);
   else
     return JOINT_ARRAY_MODE_OFF;
}

mReal M3Humanoid::GetThetaDotDeg(M3Chain chain,unsigned int  idx)
{
  const M3BaseHumanoidStatus * status = GetStatus(chain);
   if (status == NULL)
     return 0;
   if (idx < status->thetadot_size())
     return status->thetadot(idx);
   else
     return 0;
}

mReal M3Humanoid::GetThetaDotDotDeg(M3Chain chain,unsigned int  idx)
{
  const M3BaseHumanoidStatus * status = GetStatus(chain);
   if (status == NULL)
     return 0;
   if (idx < status->thetadotdot_size())
     return status->thetadotdot(idx);
   else
     return 0;
}

mReal M3Humanoid::GetTorque_mNm(M3Chain chain,unsigned int  idx)
{
  const M3BaseHumanoidStatus * status = GetStatus(chain);
   if (status == NULL)
     return 0;
   if (idx < status->torque_size())
     return status->torque(idx);
   else
     return 0;
}


mReal M3Humanoid::GetTorqueDot_mNm(M3Chain chain,unsigned int  idx)
{
  const M3BaseHumanoidStatus * status = GetStatus(chain);
   if (status == NULL)
     return 0;
   if (idx < status->torquedot_size())
     return status->torquedot(idx);
   else
     return 0;
}

mReal M3Humanoid::GetGravity(M3Chain chain,unsigned int  idx)
{
  const M3BaseHumanoidStatus * status = GetStatus(chain);
   if (status == NULL)
     return 0;
   if (idx < status->g_size())
     return status->g(idx);
   else
     return 0;
}

mReal M3Humanoid::GetPwm(M3Chain chain,unsigned int  idx)
{
  const M3BaseHumanoidStatus * status = GetStatus(chain);
   if (status == NULL)
     return 0;
   if (idx < status->pwm_cmd_size())
     return status->pwm_cmd(idx);
   else
     return 0;
}

mReal M3Humanoid::GetPayloadMass(M3Chain chain)
{
  M3BaseHumanoidParam * param = GetParam(chain);
   if (param == NULL)
     return 0;
   
   return param->payload_mass();   
}

mReal M3Humanoid::GetPayloadInertia(M3Chain chain,unsigned int  idx)
{
  M3BaseHumanoidParam * param = GetParam(chain);
   if (param == NULL)
     return 0;
   if (idx < param->payload_inertia_size())
     return param->payload_inertia(idx);
   else
     return 0;
}

mReal M3Humanoid::GetPayloadCom(M3Chain chain,unsigned int  idx)
{
  M3BaseHumanoidParam * param = GetParam(chain);
   if (param == NULL)
     return 0;
   if (idx < param->payload_com_size())
     return param->payload_com(idx);
   else
     return 0;
}

void M3Humanoid::SetTorque_mNm(M3Chain chain,unsigned int idx, mReal torque)
{
  M3BaseHumanoidCommand * cmd = GetCmd(chain);
   if (cmd == NULL)
     return;
   if (idx < cmd->tq_desired_size())
     cmd->set_tq_desired(idx, torque);
}

void M3Humanoid::SetThetaSharedMem_Deg(M3Chain chain,unsigned int idx, mReal theta)
{
  switch (chain)
  {    
    case HEAD:
      if (idx < angle_shm_head.rows())
	angle_shm_head(idx) = theta; 
      break;
  }
}

void M3Humanoid::SetSlewRateSharedMem_Deg(M3Chain chain,unsigned int idx, mReal theta)
{
  switch (chain)
  {    
    case HEAD:
      if (idx < angle_shm_head.rows())
	slew_rate_shm_head(idx) = theta; 
      break;
  }
}

void M3Humanoid::SetTorqueSharedMem_mNm(M3Chain chain,unsigned int idx, mReal torque)
{
  switch (chain)
  {
    case RIGHT_ARM:
      if (idx < torque_shm_right_arm.rows())
	torque_shm_right_arm(idx) = torque;
      break;
    case LEFT_ARM:
      if (idx < torque_shm_left_arm.rows())
	torque_shm_left_arm(idx) = torque;
      break;
    case TORSO:
      if (idx < torque_shm_torso.rows())
	torque_shm_torso(idx) = torque;
      break;    
  }
}


void M3Humanoid::SetThetaDotDeg(M3Chain chain,unsigned int idx, mReal theta_dot)
{
  M3BaseHumanoidCommand * cmd = GetCmd(chain);
   if (cmd == NULL)
     return;
   if (idx < cmd->qdot_desired_size())
     cmd->set_qdot_desired(idx, theta_dot);
}

void M3Humanoid::SetPwm(M3Chain chain,unsigned int idx, mReal pwm)
{
  M3BaseHumanoidCommand * cmd = GetCmd(chain);
   if (cmd == NULL)
     return;
   if (idx < cmd->pwm_desired_size())
     cmd->set_pwm_desired(idx, pwm);
}

void M3Humanoid::SetSlewRate(M3Chain chain,unsigned int idx, mReal slew_rate)
{
  M3BaseHumanoidCommand * cmd = GetCmd(chain);
   if (cmd == NULL)
     return;
   if (idx < cmd->q_slew_rate_size())
     cmd->set_q_slew_rate(idx, slew_rate);
}


void M3Humanoid::SetSlewRateProportional(M3Chain chain,unsigned int idx, mReal slew_rate)
{
  mReal max_slew;
  switch(chain)
  {
    case RIGHT_ARM:
      max_slew = ((M3JointParam*)right_arm->GetJoint(idx)->GetParam())->max_q_slew_rate();
      break;
    case LEFT_ARM:
      max_slew = ((M3JointParam*)left_arm->GetJoint(idx)->GetParam())->max_q_slew_rate();
      break;
    case TORSO:
      max_slew = ((M3JointParam*)torso->GetJoint(idx)->GetParam())->max_q_slew_rate();
      break;
    case HEAD:
      max_slew = ((M3JointParam*)head->GetJoint(idx)->GetParam())->max_q_slew_rate();
      break;            
  }
  
  M3BaseHumanoidCommand * cmd = GetCmd(chain);
   if (cmd == NULL)
     return;
   if (idx < cmd->q_slew_rate_size())
     cmd->set_q_slew_rate(idx, CLAMP(slew_rate, 0.0, 1.0)*max_slew);
}

void M3Humanoid::SetModePwm(M3Chain chain,unsigned int  idx)
{
  M3BaseHumanoidCommand * cmd = GetCmd(chain);
   if (cmd == NULL)
     return;
   if (idx < cmd->ctrl_mode_size())
     cmd->set_ctrl_mode(idx, JOINT_ARRAY_MODE_PWM);
}

void M3Humanoid::SetModeTorque(M3Chain chain,unsigned int  idx)
{
  M3BaseHumanoidCommand * cmd = GetCmd(chain);
   if (cmd == NULL)
     return;
   if (idx < cmd->ctrl_mode_size())
     cmd->set_ctrl_mode(idx, JOINT_ARRAY_MODE_TORQUE);
}

void M3Humanoid::SetModeTorqueGc(M3Chain chain,unsigned int  idx)
{
  M3BaseHumanoidCommand * cmd = GetCmd(chain);
   if (cmd == NULL)
     return;
   if (idx < cmd->ctrl_mode_size())
     cmd->set_ctrl_mode(idx, JOINT_ARRAY_MODE_TORQUE_GC);
}

void M3Humanoid::SetModeThetaGcMj(M3Chain chain,unsigned int  idx)
{
  M3BaseHumanoidCommand * cmd = GetCmd(chain);
   if (cmd == NULL)
     return;
   if (idx < cmd->ctrl_mode_size())
     cmd->set_ctrl_mode(idx, JOINT_ARRAY_MODE_THETA_GC);
}

void M3Humanoid::SetModeThetaMj(M3Chain chain,unsigned int  idx)
{
  M3BaseHumanoidCommand * cmd = GetCmd(chain);
   if (cmd == NULL)
     return;
   if (idx < cmd->ctrl_mode_size())
     cmd->set_ctrl_mode(idx, JOINT_ARRAY_MODE_THETA_MJ);
}

void M3Humanoid::SetModeThetaGc(M3Chain chain,unsigned int  idx)
{
  M3BaseHumanoidCommand * cmd = GetCmd(chain);
   if (cmd == NULL)
     return;
   if (idx < cmd->ctrl_mode_size())
     cmd->set_ctrl_mode(idx, JOINT_ARRAY_MODE_THETA_GC);
}

void M3Humanoid::SetModeTheta(M3Chain chain,unsigned int  idx)
{
  M3BaseHumanoidCommand * cmd = GetCmd(chain);
   if (cmd == NULL)
     return;
   if (idx < cmd->ctrl_mode_size())
     cmd->set_ctrl_mode(idx, JOINT_ARRAY_MODE_THETA);
}

void M3Humanoid::SetModeOff(M3Chain chain,unsigned int  idx)
{
  M3BaseHumanoidCommand * cmd = GetCmd(chain);
   if (cmd == NULL)
     return;
   if (idx < cmd->ctrl_mode_size())
     cmd->set_ctrl_mode(idx, JOINT_ARRAY_MODE_OFF);
}

void M3Humanoid::SetStiffness(M3Chain chain,unsigned int  idx, mReal stiffness)
{
   M3BaseHumanoidCommand * cmd = GetCmd(chain);
   if (cmd == NULL)
     return;
   if (idx < cmd->q_stiffness_size())
     cmd->set_q_stiffness(idx, stiffness);
}

void M3Humanoid::SetPayloadMass(M3Chain chain, mReal mass)
{
   M3BaseHumanoidParam * param = GetParam(chain);
   if (param == NULL)
     return;
  param->set_payload_mass(mass);
  
}

void M3Humanoid::SetPayloadInertia(M3Chain chain,unsigned int  idx, mReal inertia)
{
   M3BaseHumanoidParam * param = GetParam(chain);
   if (param == NULL)
     return;
   if (idx < param->payload_inertia_size())
    param->set_payload_inertia(idx, inertia);
}

void M3Humanoid::SetPayloadCom(M3Chain chain,unsigned int  idx, mReal com)
{
   M3BaseHumanoidParam * param = GetParam(chain);
   if (param == NULL)
     return;
    if (idx < param->payload_com_size())
      param->set_payload_com(idx, com);
}

Eigen::Vector3d M3Humanoid::GetEndPosition(M3Chain chain)
{
  Eigen::Vector3d end_pos;
  const M3BaseHumanoidStatus * status = GetStatus(chain);
  for (int i = 0; i<3; i++)
    end_pos[i] = status->end_pos(i);
  
  return end_pos;
}

Eigen::Matrix3d M3Humanoid::GetEndRotation(M3Chain chain)
{
  Eigen::Matrix3d end_rot;
  const M3BaseHumanoidStatus * status = GetStatus(chain);
  for (int i = 0; i<3; i++)
  {
    for (int j = 0; j<3; j++)
      end_rot(i,j) = status->end_rot((i*3)+(j));
  }
  return end_rot;
}

Eigen::MatrixXd M3Humanoid::GetJacobian(M3Chain chain)
{
  const M3BaseHumanoidStatus * status = GetStatus(chain);
  Eigen::MatrixXd jac = Eigen::MatrixXd::Zero(6,GetNdof(chain));
  
  for (int i = 0; i<6; i++)
  {
    for (int j = 0; j<GetNdof(chain); j++)
      jac(i,j) = status->j((i*GetNdof(chain))+(j));    
  }
  return jac;

}

int M3Humanoid::GetNdof(M3Chain chain)
{
    switch(chain)
    {
      case RIGHT_ARM:	
	if (right_arm)
	  return right_arm->GetNumDof();
	break;
      case LEFT_ARM:
	if (left_arm)
	  return left_arm->GetNumDof();
	break;
      case TORSO:
	if (torso)
	  return torso->GetNumDof();
	break;
      case HEAD:
	if (head)
	  return head->GetNumDof();      
	break;
    }
    return 0;
}

void M3Humanoid::SetMotorPowerOn()
{
    command.set_enable_motor(true);
}


void M3Humanoid::SetMotorPowerOff()
{
    command.set_enable_motor(false);
}


void M3Humanoid::TestAPI()
{
  SetMotorPowerOn();
  
  SetModeOff(RIGHT_ARM,0);
  SetModePwm(RIGHT_ARM,0);
  SetModeTorque(RIGHT_ARM,0);
  SetModeThetaGcMj(RIGHT_ARM,0);
  SetModeThetaMj(RIGHT_ARM,0);  
  SetModeTheta(RIGHT_ARM,0);
  SetModeThetaGc(RIGHT_ARM,0);
  SetThetaDeg(RIGHT_ARM,0,20);
  SetStiffness(RIGHT_ARM,0,.7);
  M3_INFO("t: %f\n",GetThetaDeg(RIGHT_ARM, 0));
  M3_INFO("td: %f\n",GetThetaDotDeg(RIGHT_ARM, 0));
  M3_INFO("tdd: %f\n",GetThetaDotDotDeg(RIGHT_ARM, 0));
  M3_INFO("q: %f\n",GetTorque_mNm(RIGHT_ARM, 0));
  M3_INFO("qd: %f\n",GetTorqueDot_mNm(RIGHT_ARM, 0));
  Eigen::Vector3d ep = GetEndPosition(RIGHT_ARM);
  for (int i = 0; i++; i<3)
    M3_INFO("ep %i: %f\n",i,ep[i]);
  Eigen::Matrix3d er = GetEndRotation(RIGHT_ARM);
  for (int i = 0; i++; i<9)
    M3_INFO("er %i: %f\n",i,er[i]);
  Eigen::MatrixXd j = GetJacobian(RIGHT_ARM);
  for (int i = 0; i++; i<(6*7))
    M3_INFO("j %i: %f\n",i,j[i]);
  M3_INFO("g: %f\n",GetGravity(RIGHT_ARM, 0));
  M3_INFO("p: %f\n",GetPwm(RIGHT_ARM, 0));
  SetTorque_mNm(RIGHT_ARM,0,30);
  SetPwm(RIGHT_ARM,0,40);
  SetSlewRate(RIGHT_ARM,0,50);
  SetSlewRateProportional(RIGHT_ARM,0,.8);
  M3_INFO("m: %f\n",GetPayloadMass(RIGHT_ARM));
  M3_INFO("i: %f\n",GetPayloadInertia(RIGHT_ARM,0));
  M3_INFO("i: %f\n",GetPayloadCom(RIGHT_ARM,0));
  SetPayloadMass(RIGHT_ARM,2);
  SetPayloadCom(RIGHT_ARM,0,3);
  SetPayloadInertia(RIGHT_ARM,0,2);
  M3_INFO("dof: %d\n",GetNdof(RIGHT_ARM));
  
  SetMotorPowerOff();
}

}
