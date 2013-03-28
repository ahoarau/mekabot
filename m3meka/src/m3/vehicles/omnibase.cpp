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

#include "m3/vehicles/omnibase.h"
#include "omnibase.h"

namespace m3
{

using namespace m3rt;
using namespace std;

void M3Omnibase::StepOpSpaceForceCtrl()
{   
  //Compute tq_desired  
  pcv_cmd.trajMode_ = TRAJ_M_OP_FORCE; 
  pcv_cmd.ctrlFrame_ = CTRL_F_LOCAL;
  pcv_cmd.controlMode_ = CTRL_CF_CMD;
  pcv_cmd.internalForce_ = false;
  pcv_cmd.accel_FF_ = false;
  
  for (int i = 0; i < 3; i++)
    pcv_cmd.opspace_force_desired[i] = command.opspace_force_desired(i);
  
}


void M3Omnibase::StepOpSpaceTrajCtrl()
{
  
  //Compute tq_desired
  /*for (int i=0;i<motor_array->GetNumDof();i++)
        {
            motor_array->GetJoint(i)->SetDesiredControlMode(JOINT_MODE_TORQUE);
            motor_array->GetJoint(i)->SetDesiredTorque(tq_desired[i]);
        }*/
  switch (command.traj_mode())
    {          
      case OMNIBASE_TRAJ_OFF:
	StepOffCtrl();
	break;
      case OMNIBASE_TRAJ_JOYSTICK:
    	StepJoystickCtrl();
	break;
      case OMNIBASE_TRAJ_GOAL:
    	StepTrajGoalCtrl();
	break;
      case OMNIBASE_TRAJ_VIAS:
	StepTrajViaCtrl();
	break;
      default:
	StepOffCtrl();
	break;
    }
     old_traj_mode = command.traj_mode();
}

void M3Omnibase::StepHomeCtrl()
{
  //Compute tq_desired
  //holomni.StepHome();
    /*for (int i=0;i<motor_array->GetNumDof();i++)
        {
            motor_array->GetJoint(i)->SetDesiredControlMode(JOINT_MODE_TORQUE);
            motor_array->GetJoint(i)->SetDesiredTorque(command.motor_tq_desired(i));
        }*/
}

void M3Omnibase::EnableBreakbeamEncoderZero(int idx)
{
  motor_array->GetJoint(idx*2)->SetLimitSwitchNegZeroEncoder();
}

void M3Omnibase::DisableBreakbeamEncoderZero(int idx)
{
  motor_array->GetJoint(idx*2)->ClrLimitSwitchNegZeroEncoder();
}

void M3Omnibase::StepCasterCtrl()
{ 
  pcv_cmd.trajMode_ = TRAJ_M_JOINT_TORQUE; 
  pcv_cmd.ctrlFrame_ = CTRL_F_LOCAL;
  pcv_cmd.controlMode_ = CTRL_L_CONST;
  pcv_cmd.internalForce_ = false;
  pcv_cmd.accel_FF_ = false;
  
  
  for (int i=0;i<motor_array->GetNumDof()/2;i++)
  {
    switch (command.caster_mode(i))
      {          	
	case OMNIBASE_CASTER_TORQUE:
	  pcv_cmd.steer_torque_desired_Nm[i] = command.steer_torque_desired(i);
	  pcv_cmd.roll_torque_desired_Nm[i] = command.roll_torque_desired(i);
	  break;
	case OMNIBASE_CASTER_VELOCITY:
	  if (old_ctrl_mode != OMNIBASE_CTRL_CASTER)
	  {     
	    pid_steer_vel[i].ResetIntegrator();  // reset I's
	    //pid_roll_vel[i].ResetIntegrator();  // reset I's
	  }     
	  pcv_cmd.steer_torque_desired_Nm[i] = pid_steer_vel[i].Step(
	    RAD2DEG(pcv_status.steer_velocity_rad[i]),0.0,command.steer_velocity_desired(i),param.ks_p(),
	    param.ks_i(),0.0,param.ks_i_limit(),param.ks_i_range());    	 
	    pcv_cmd.roll_torque_desired_Nm[i] = 0.0;
	    
	  break;
	  
	case OMNIBASE_CASTER_THETA:
	  if (old_ctrl_mode != OMNIBASE_CTRL_CASTER)
	  {     
	    pid_steer_theta[i].ResetIntegrator();  // reset I's
	    //pid_roll_vel[i].ResetIntegrator();  // reset I's
	  }     
	  // PID proto: Step(mReal sense, mReal sense_dot,mReal des, mReal p, mReal i, 
						 //mReal d, mReal i_limit, mReal i_range);	
	  pcv_cmd.steer_torque_desired_Nm[i] = pid_steer_theta[i].Step(
	    RAD2DEG(pcv_status.steer_angle_rad[i]), RAD2DEG(pcv_status.steer_velocity_rad[i]), command.steer_theta_desired(i),param.ks_p(),
	    param.ks_i(), param.ks_d(), param.ks_i_limit(),param.ks_i_range());    	 	    
	    pcv_cmd.roll_torque_desired_Nm[i] = command.roll_torque_desired(i);
	    	    
	  break;	  
	case OMNIBASE_CASTER_OFF:
	  pcv_cmd.steer_torque_desired_Nm[i] = 0;
	  pcv_cmd.roll_torque_desired_Nm[i] = 0;
	  break;
      }      
  }
  
}

void M3Omnibase::StepJoystickCtrl()
{ 
  //Compute tq_desired  
  pcv_cmd.trajMode_ = TRAJ_M_JOYV; 
  pcv_cmd.ctrlFrame_ = CTRL_F_LOCAL;
  //pcv_cmd.controlMode_ = CTRL_L_CONST;
  pcv_cmd.controlMode_ = CTRL_LM;  
    
  pcv_cmd.internalForce_ = true;
  pcv_cmd.accel_FF_ = true;  
  /* Note: holomni is expecting commands scaled -1 -> 1 */
  pcv_cmd.joystick_x = command.joystick_x();
  pcv_cmd.joystick_y = command.joystick_y();
  pcv_cmd.joystick_yaw = command.joystick_yaw();
  pcv_cmd.joystick_button = command.joystick_button();  
  
  /*if (tmp_cnt++ == 1000)
  {
   M3_DEBUG("but: %d\n",  pcv_cmd.joystick_button);
   M3_DEBUG("x: %f\n",  pcv_cmd.joystick_x);
   M3_DEBUG("y: %f\n",  pcv_cmd.joystick_y);
   M3_DEBUG("yaw: %f\n",  pcv_cmd.joystick_yaw);
   tmp_cnt = 0;
  }*/
  
}

void M3Omnibase::StepTrajGoalCtrl()
{   
  pcv_cmd.trajMode_ = TRAJ_M_GOAL; 
  pcv_cmd.ctrlFrame_ = CTRL_F_GLOBAL;
  //pcv_cmd.controlMode_ = CTRL_L_CONST;
  pcv_cmd.controlMode_ = CTRL_LM;  
  pcv_cmd.internalForce_ = true;
  pcv_cmd.accel_FF_ = true;
  
  pcv_cmd.traj_goal[0] = command.traj_goal(0);
  pcv_cmd.traj_goal[1] = command.traj_goal(1);
  pcv_cmd.traj_goal[2] = DEG2RAD(command.traj_goal(2));
  
  /*if (pcv_status.traj_goal_reached)
    pcv->UpdateGoal(pcv_cmd);*/
}

void M3Omnibase::StepTrajViaCtrl()
{   
  pcv_cmd.trajMode_ = TRAJ_M_GOAL; 
  pcv_cmd.ctrlFrame_ = CTRL_F_GLOBAL;
  pcv_cmd.controlMode_ = CTRL_L_CONST;
  pcv_cmd.internalForce_ = false;
  pcv_cmd.accel_FF_ = false;
  
  for (int i=0;i<command.vias_size();i++)
  {    
    new_via.set_idx(command.vias(i).idx());
    for (int j=0; j<3; j++)
    {   
	new_via.set_position_desired(j,command.vias(i).position_desired(j));
    }    
    new_via.set_lin_velocity_avg(command.vias(i).lin_velocity_avg());
    new_via.set_ang_velocity_avg(command.vias(i).ang_velocity_avg());
    vias.push_back(new_via); 
    M3_INFO("to via queue: %f, %f, %f\n",new_via.position_desired(0),new_via.position_desired(1),new_via.position_desired(2) );
  }
  command.clear_vias();
  
  if (pcv_status.traj_goal_reached || old_traj_mode!=OMNIBASE_TRAJ_VIAS)
  {
    if (vias.size() > 0) {            
      active_via = vias.front();
      vias.erase(vias.begin());      
      pcv_cmd.traj_goal[0] = active_via.position_desired(0);      
      pcv_cmd.traj_goal[1] = active_via.position_desired(1);
      pcv_cmd.traj_goal[2] = DEG2RAD(active_via.position_desired(2));
      M3_INFO("cmd to holomni: %f, %f, %f\n",pcv_cmd.traj_goal[0],pcv_cmd.traj_goal[1],pcv_cmd.traj_goal[2] );
      pcv->UpdateGoal(pcv_cmd);      
    }
  }
  
}


void M3Omnibase::StepCartesianLocalCtrl()
{
  
//Compute tq_desired  
  pcv_cmd.trajMode_ = TRAJ_M_CART_LOCAL; 
  pcv_cmd.ctrlFrame_ = CTRL_F_LOCAL;
  pcv_cmd.controlMode_ = CTRL_L_CONST;
  pcv_cmd.internalForce_ = false;
  pcv_cmd.accel_FF_ = false;
  
  for (int i = 0; i < 3; i++)
  {
    if (i == 2)
    {
      pcv_cmd.local_position_desired[i] = DEG2RAD(command.local_position_desired(i));
      pcv_cmd.local_acceleration_desired[i] = DEG2RAD(command.local_acceleration_desired(i));
      pcv_cmd.local_velocity_desired[i] = DEG2RAD(command.local_velocity_desired(i));
    } else {
      pcv_cmd.local_position_desired[i] = command.local_position_desired(i);
      pcv_cmd.local_acceleration_desired[i] = command.local_acceleration_desired(i);
      pcv_cmd.local_velocity_desired[i] = command.local_velocity_desired(i);      
    }
  }
  
}


void M3Omnibase::StepCartesianGlobalCtrl()
{
  
//Compute tq_desired  
  pcv_cmd.trajMode_ = TRAJ_M_CART_GLOB; 
  pcv_cmd.ctrlFrame_ = CTRL_F_LOCAL;
  pcv_cmd.controlMode_ = CTRL_L_CONST;
  pcv_cmd.internalForce_ = false;
  pcv_cmd.accel_FF_ = false;
  
  for (int i = 0; i < 3; i++)
  {
    pcv_cmd.global_position_desired[i] = command.global_position_desired(i);
    pcv_cmd.global_velocity_desired[i] = command.global_velocity_desired(i);
    pcv_cmd.global_acceleration_desired[i] = command.global_acceleration_desired(i);
  }
  
 
}

void M3Omnibase::StepOffCtrl()
{
  

   pcv_cmd.trajMode_ = TRAJ_M_OFF; 
  pcv_cmd.ctrlFrame_ = CTRL_F_LOCAL;
  pcv_cmd.controlMode_ = CTRL_OFF;
  pcv_cmd.internalForce_ = false;
  pcv_cmd.accel_FF_ = false;
}


void M3Omnibase::StepCommand()
{    
    if (IsStateSafeOp() || IsStateError())
        return;
    
    for (int i=0;i<motor_array->GetNumDof()/2;i++)
    {
	if ((bool)param.enable_breakbeam(i))
	  EnableBreakbeamEncoderZero(i);
	else
	  DisableBreakbeamEncoderZero(i);
    }
    
    switch (command.ctrl_mode())
    {    
    case OMNIBASE_CTRL_CASTER:
        for (int i=0;i<motor_array->GetNumDof()/2;i++)
	{
	  if (bool(status.calibrated(i)) && !old_calibrated[i])
	  {
	    command.set_caster_mode(i, OMNIBASE_CASTER_OFF);
	  }
	  old_calibrated[i] = bool(status.calibrated(i));
	    
	  if (command.caster_mode(i) == OMNIBASE_CASTER_OFF)
	  {	    
	    ((M3JointArrayCommand*)motor_array->GetCommand())->set_ctrl_mode(i*2, JOINT_ARRAY_MODE_OFF);  
	    ((M3JointArrayCommand*)motor_array->GetCommand())->set_ctrl_mode(i*2+1, JOINT_ARRAY_MODE_OFF);
	  }
	  else
	  {
	    ((M3JointArrayCommand*)motor_array->GetCommand())->set_ctrl_mode(i*2, JOINT_ARRAY_MODE_TORQUE);
	    ((M3JointArrayCommand*)motor_array->GetCommand())->set_tq_desired(i*2, 1000*pcv_status.motor_torque_Nm[i*2]);
	    ((M3JointArrayCommand*)motor_array->GetCommand())->set_ctrl_mode(i*2+1, JOINT_ARRAY_MODE_TORQUE);
	    ((M3JointArrayCommand*)motor_array->GetCommand())->set_tq_desired(i*2+1, 1000*pcv_status.motor_torque_Nm[i*2+1]);
	  }	  
	}
        break;    
    case OMNIBASE_CTRL_CALIBRATE:       
      for (int i=0;i<motor_array->GetNumDof();i++)
      {
	/*((M3ActuatorEcCommand*)motor_array->GetJoint(i)->GetActuator()->GetActuatorEc()->GetCommand())->set_t_desire(
		 motor_array->GetJoint(i)->mNmToTicks(1000*pcv_status.motor_torque_Nm[i]));	
	((M3ActuatorEcCommand*)motor_array->GetJoint(i)->GetActuator()->GetActuatorEc()->GetCommand())->set_mode(ACTUATOR_EC_MODE_PWM);*/
	/*((M3JointArrayCommand*)motor_array->GetCommand())->set_pwm_desired(i,
		 motor_array->GetJoint(i)->mNmToTicks(round(1000*pcv_status.motor_torque_Nm[i])));	
	((M3JointArrayCommand*)motor_array->GetCommand())->set_ctrl_mode(i, JOINT_ARRAY_MODE_PWM);*/
	((M3JointArrayCommand*)motor_array->GetCommand())->set_ctrl_mode(i, JOINT_ARRAY_MODE_TORQUE);
	((M3JointArrayCommand*)motor_array->GetCommand())->set_tq_desired(i, 1000*pcv_status.motor_torque_Nm[i]);
      }
	break;
    case OMNIBASE_CTRL_OPSPACE_TRAJ:
    case OMNIBASE_CTRL_OPSPACE_FORCE:            
    case OMNIBASE_CTRL_CART_LOCAL:	
    case OMNIBASE_CTRL_CART_GLOBAL:    
      for (int i=0;i<motor_array->GetNumDof();i++)
      {
	
	// EXPERIMENTAL FLIPOUT PREVENTION CODE BELOW.  IGNORE FOR NOW
	
	/*((M3ActuatorEcCommand*)motor_array->GetJoint(i)->GetActuator()->GetActuatorEc()->GetCommand())->set_t_desire(
		 motor_array->GetJoint(i)->mNmToTicks(1000*pcv_status.motor_torque_Nm[i]));	
	((M3ActuatorEcCommand*)motor_array->GetJoint(i)->GetActuator()->GetActuatorEc()->GetCommand())->set_mode(ACTUATOR_EC_MODE_PWM);*/
	/*((M3JointArrayCommand*)motor_array->GetCommand())->set_pwm_desired(i,
		 motor_array->GetJoint(i)->mNmToTicks(round(1000*pcv_status.motor_torque_Nm[i])));	
	((M3JointArrayCommand*)motor_array->GetCommand())->set_ctrl_mode(i, JOINT_ARRAY_MODE_PWM);*/
	/*if ( i % 2 == 0 && flip_out_detect_enable) // if we are on a steer motor_angles_rad
	{
	  // get avg vel of other steer motors
	  mReal avg_vel = 0;
	  for (int j = 0; j < motor_array->GetNumDof()/2; j++)
	  {
	   if (j*2 != i)
	     avg_vel += ABS(RAD2DEG(pcv_status.steer_velocity_rad[j]));
	  }
	  avg_vel = avg_vel / (motor_array->GetNumDof()/2 - 1);
	  bool flip_detected = ((ABS(RAD2DEG(pcv_status.steer_velocity_rad[i/2])) >  flip_out_detect_mult*avg_vel) && (ABS(RAD2DEG(pcv_status.steer_velocity_rad[i/2])) > flip_out_detect_min));
	  if (flip_detected || (flip_out_detect_timeout[i/2] > 0)) // wheel is freaking out
	  {
	    if (flip_detected)
	      flip_out_detect_time[i/2]++;
	    if (flip_out_detect_time[i/2] > flip_out_detect_time_cnt) // start reducing torque to prevent flipout
	    {
	      if (first_flip_out[i/2])
	      {
		if ((i == 2) || (i == 4))
		{
		  M3_DEBUG("flip: %i\n", i/2);
		  M3_DEBUG("avg: %f\n", avg_vel);
		  M3_DEBUG("my: %f\n", ABS(RAD2DEG(pcv_status.steer_velocity_rad[i/2])));
		}
		flip_out_detect_timeout[i/2] = flip_out_detect_timeout_cnt;
	      }
	      first_flip_out[i/2] = false;
	      if (flip_out_detect_timeout[i/2] == 0) // still flipping out after timeout? restart it..
		flip_out_detect_timeout[i/2] = flip_out_detect_timeout_cnt;
	      flip_out_detect_timeout[i/2]--;	    
	      ((M3JointArrayCommand*)motor_array->GetCommand())->set_ctrl_mode(i, JOINT_ARRAY_MODE_TORQUE);
	      ((M3JointArrayCommand*)motor_array->GetCommand())->set_tq_desired(i, 0);
	    } else {
	      ((M3JointArrayCommand*)motor_array->GetCommand())->set_ctrl_mode(i, JOINT_ARRAY_MODE_TORQUE);
	      ((M3JointArrayCommand*)motor_array->GetCommand())->set_tq_desired(i, 1000*pcv_status.motor_torque_Nm[i]);
	    }
	  } else {
	    first_flip_out[i/2] = true;
	    //if (flip_out_detect_time[i/2] > 0)
	    //  flip_out_detect_time[i/2]--;
	    flip_out_detect_time[i/2] = 0;
	    ((M3JointArrayCommand*)motor_array->GetCommand())->set_ctrl_mode(i, JOINT_ARRAY_MODE_TORQUE);
	    ((M3JointArrayCommand*)motor_array->GetCommand())->set_tq_desired(i, 1000*pcv_status.motor_torque_Nm[i]);
	  }
	  if ((i/2 == 0) && flip_out_detect_disable_0)
	  {
	    ((M3JointArrayCommand*)motor_array->GetCommand())->set_ctrl_mode(i, JOINT_ARRAY_MODE_TORQUE);
	    ((M3JointArrayCommand*)motor_array->GetCommand())->set_tq_desired(i, 1000*pcv_status.motor_torque_Nm[i]);
	    flip_out_detect_timeout[i/2] = 0;
	  } else if ((i/2 == 1) && flip_out_detect_disable_1)
	  {
	    ((M3JointArrayCommand*)motor_array->GetCommand())->set_ctrl_mode(i, JOINT_ARRAY_MODE_TORQUE);
	    ((M3JointArrayCommand*)motor_array->GetCommand())->set_tq_desired(i, 1000*pcv_status.motor_torque_Nm[i]);
	    flip_out_detect_timeout[i/2] = 0;
	  } if ((i/2 == 2) && flip_out_detect_disable_2)
	  {
	    ((M3JointArrayCommand*)motor_array->GetCommand())->set_ctrl_mode(i, JOINT_ARRAY_MODE_TORQUE);
	    ((M3JointArrayCommand*)motor_array->GetCommand())->set_tq_desired(i, 1000*pcv_status.motor_torque_Nm[i]);
	    flip_out_detect_timeout[i/2] = 0;
	  } if ((i/2 == 3) && flip_out_detect_disable_3)
	  {
	    ((M3JointArrayCommand*)motor_array->GetCommand())->set_ctrl_mode(i, JOINT_ARRAY_MODE_TORQUE);
	    ((M3JointArrayCommand*)motor_array->GetCommand())->set_tq_desired(i, 1000*pcv_status.motor_torque_Nm[i]);
	    flip_out_detect_timeout[i/2] = 0;
	  }  
	    
	} else { // roll motor 
	  if (flip_out_detect_timeout[i/2-1] > 0 )
	  {
	    ((M3JointArrayCommand*)motor_array->GetCommand())->set_ctrl_mode(i, JOINT_ARRAY_MODE_TORQUE);
	    ((M3JointArrayCommand*)motor_array->GetCommand())->set_tq_desired(i, 0);
	  } else */{
	    ((M3JointArrayCommand*)motor_array->GetCommand())->set_ctrl_mode(i, JOINT_ARRAY_MODE_TORQUE);
	    ((M3JointArrayCommand*)motor_array->GetCommand())->set_tq_desired(i, 1000*pcv_status.motor_torque_Nm[i]);
	  }
	//}
	
	  
      }
	break;
    case OMNIBASE_CTRL_OFF:
    default:
        for (int i=0;i<motor_array->GetNumDof();i++)
	    ((M3JointArrayCommand*)motor_array->GetCommand())->set_ctrl_mode(i, JOINT_ARRAY_MODE_OFF);  
        break;
    };
    old_ctrl_mode = command.ctrl_mode(); 
}



bool M3Omnibase::ReadConfig(const char * filename)
{
  
    YAML::Node doc;
    GetYamlDoc(filename, doc);
    doc["pwr_component"] >> pwr_name;
    doc["holomni_pcv_config"] >> holomni_pcv_config;	
    doc["joint_array_component"] >> joint_array_name;	
        
    mReal val;
    doc["param"]["ks_i"] >> val;
    param.set_ks_i(val);
    doc["param"]["ks_i_limit"] >> val;
    param.set_ks_i_limit(val);
    doc["param"]["ks_i_range"] >> val;
    param.set_ks_i_range(val);
    doc["param"]["ks_p"] >> val;
    param.set_ks_p(val);
    doc["param"]["ks_d"] >> val;
    param.set_ks_d(val); 
    
    
    /*doc["flip_out_detect_disable_0"] >> flip_out_detect_disable_0;
    doc["flip_out_detect_disable_1"] >> flip_out_detect_disable_1;
    doc["flip_out_detect_disable_2"] >> flip_out_detect_disable_2;
    doc["flip_out_detect_disable_3"] >> flip_out_detect_disable_3;
    
    doc["flip_out_detect_mult"] >> flip_out_detect_mult;
    
    doc["flip_out_detect_min"] >> flip_out_detect_min;
    
    doc["flip_out_detect_enable"] >> flip_out_detect_enable;
    
    doc["flip_out_detect_timeout"] >> flip_out_detect_timeout_cnt;
    
    doc["flip_out_detect_time"] >> flip_out_detect_time_cnt;*/
    
    //angle_df.ReadConfig( doc["calib"]["angle_df"]);
    
    return M3Vehicle::ReadConfig(filename);;
}


void M3Omnibase::Startup()
{	    
    
  pcv_cmd.trajMode_ = TRAJ_M_OFF; 
  pcv_cmd.ctrlFrame_ = CTRL_F_LOCAL;
  pcv_cmd.controlMode_ = CTRL_OFF;
  pcv_cmd.internalForce_ = false;
  pcv_cmd.accel_FF_ = false;
  
  for (int i = 0; i < 4; i++)
  {
    first_flip_out[i] = true;
    flip_out_detect_timeout[i] = 0;
    flip_out_detect_time[i] = 0;
  }
  
  for (int i = 0; i < 3; i++)
  {
   command.add_opspace_force_desired(0); 
   command.add_local_position_desired(0); 
   command.add_local_velocity_desired(0); 
   command.add_local_acceleration_desired(0); 
   command.add_global_position_desired(0); 
   command.add_global_velocity_desired(0); 
   command.add_global_acceleration_desired(0);    
   command.add_traj_goal(0);    
   command.add_local_position(0); 
   command.add_global_position(0);   
   status.add_global_position(0);
   status.add_global_velocity(0);
   status.add_local_velocity(0);   
   status.add_local_force(0);
   status.add_local_acceleration(0);
   status.add_local_position(0);
   status.add_position_desired(0);
   status.add_velocity_desired(0);   
   status.add_acceleration_desired(0);   
   new_via.add_position_desired(0);
   status.add_position_error(0);
   status.add_velocity_error(0);      
  }
        
  
  for (int i = 0; i < motor_array->GetNumDof()/2; i++)
  {    
    //M3_DEBUG("oo %d\n", i);
    status.add_steer_torque_desired(0);
    status.add_steer_torque_internal(0);
    status.add_steer_angle(0);
    status.add_steer_velocity(0);
    status.add_roll_torque_desired(0);
    status.add_roll_torque_internal(0);
    status.add_roll_angle(0);
    status.add_roll_velocity(0);
    status.add_calibrated(0);
    param.add_enable_breakbeam(0);
    command.add_steer_torque_desired(0);
    command.add_steer_theta_desired(0);
    command.add_roll_torque_desired(0);
    command.add_steer_velocity_desired(0);
    command.add_roll_velocity_desired(0);
    command.add_caster_mode(OMNIBASE_CASTER_OFF);    
  }
  
  for (int i = 0; i < motor_array->GetNumDof(); i++)
  {
   status.add_motor_torque_desired(0); 
   status.add_motor_current(0);
  }
  
  command.set_ctrl_mode(OMNIBASE_CTRL_OFF);
  command.set_adjust_local_position(0);
  command.set_adjust_global_position(0);
  
    string path;    
    if (GetEnvironmentVar(M3_ROBOT_ENV_VAR, path))
    {		
	    path.append("/robot_config/");
	    path.append(holomni_pcv_config);
    }
    pcv = new PCV(path, RT_TASK_FREQUENCY);
    if (pcv == NULL)
    {
      M3_ERR("Error creating Holomni_lib component.\n");
      SetStateError();
    } else {
	SetStateSafeOp();	
    }
    
    pid_steer_vel = vector<M3PID>(motor_array->GetNumDof()/2);
    pid_roll_vel = vector<M3PID>(motor_array->GetNumDof()/2);
    pid_steer_theta = vector<M3PID>(motor_array->GetNumDof()/2);
    
    old_calibrated = vector<bool>(motor_array->GetNumDof()/2);
    for (int i = 0; i < motor_array->GetNumDof()/2; i++)
      old_calibrated[i] = false;
    
    M3OmniVia via_a;
    M3OmniVia via_b;
    for (int i = 0; i < 3; i++)
    {
	via_a.add_position_desired(0);
	via_b.add_position_desired(0);
    }
    
    via_a.set_position_desired(0,0);
    via_a.set_position_desired(1,0.5);
    via_a.set_position_desired(2,0);
    
    via_b.set_position_desired(0,0.5);
    via_b.set_position_desired(1,0);
    via_b.set_position_desired(2,0);
    
    //vias.push_back(via_a);
    //vias.push_back(via_b);
    
    for (int i=0;i<motor_array->GetNumDof();i++)
      motor_array->GetJoint(i)->DisablePwmRamp();
 
}

void M3Omnibase::Shutdown()
{
  if (pcv != NULL)
    delete pcv;
  pcv = NULL;    
}

void M3Omnibase::StepStatus()
{
  if (IsStateError())
		return;
  
  switch (command.ctrl_mode())
    {    
    case OMNIBASE_CTRL_CALIBRATE:
       StepHomeCtrl();
        break;    
    case OMNIBASE_CTRL_OPSPACE_FORCE:
        StepOpSpaceForceCtrl();
        break;
    case OMNIBASE_CTRL_OPSPACE_TRAJ:
	StepOpSpaceTrajCtrl();
        break;
    case OMNIBASE_CTRL_CART_LOCAL:
	StepCartesianLocalCtrl();
        break;
    case OMNIBASE_CTRL_CART_GLOBAL:
	StepCartesianGlobalCtrl();
        break;    
    case OMNIBASE_CTRL_OFF:
        StepOffCtrl();
        break;    
    case OMNIBASE_CTRL_CASTER:
	StepCasterCtrl(); 
	break;    
    };
  
  
  pcv_cmd.adjust_local_position = (bool)command.adjust_local_position();
  pcv_cmd.adjust_global_position = (bool)command.adjust_global_position();
  //pcv_cmd.adjust_local_position = true;
  //pcv_cmd.adjust_global_position = true;
  for (int i = 0; i < 2; i++)
  {
   pcv_cmd.local_position[i] = command.local_position(i); 
   pcv_cmd.global_position[i] = command.global_position(i);  
  }
  
  pcv_cmd.local_position[2] = DEG2RAD(command.local_position(2)); 
  pcv_cmd.global_position[2] = DEG2RAD(command.global_position(2));
  
  pcv_cmd.max_linear_acceleration = command.max_linear_acceleration();
  pcv_cmd.max_rotation_acceleration = DEG2RAD(command.max_rotation_acceleration());
  pcv_cmd.max_linear_velocity = command.max_linear_velocity();
  pcv_cmd.max_rotation_velocity = DEG2RAD(command.max_rotation_velocity());
   
  pcv_cmd.use_angles_instead_of_ticks = false;  
  pcv_cmd.use_external_motor_velocities = false;
  for (int i=0;i<motor_array->GetNumDof();i++)
  {
    pcv_cmd.enc_ticks[i] = motor_array->GetJoint(i)->GetTicks();      
    pcv_cmd.motor_angles_rad[i] = motor_array->GetJoint(i)->GetThetaRad();      
    pcv_cmd.motor_velocities_rad[i] = motor_array->GetJoint(i)->GetThetaDotRad();
  }
  
  pcv_status = pcv->Step(pcv_cmd);
   
  ///////////////////
  status.set_traj_goal_reached((int)pcv_status.traj_goal_reached);
    
  for (int i = 0; i < 3; i++)
  {
     if (i==2)
     {
      status.set_global_position(i, WrapDeg(RAD2DEG(pcv_status.global_position[i])));
      status.set_global_velocity(i, RAD2DEG(pcv_status.global_velocity[i]));
      status.set_local_velocity(i, RAD2DEG(pcv_status.local_velocity[i]));
      status.set_local_force(i, pcv_status.local_force[i]);
      status.set_local_acceleration(i, RAD2DEG(pcv_status.local_acceleration[i]));
      status.set_local_position(i, WrapDeg(RAD2DEG(pcv_status.local_position[i])));     
      status.set_position_desired(i, WrapDeg(RAD2DEG(pcv_status.position_desired[i])));      
      status.set_velocity_desired(i, RAD2DEG(pcv_status.velocity_desired[i]));      
      status.set_acceleration_desired(i, RAD2DEG(pcv_status.acceleration_desired[i]));      
      status.set_position_error(i, RAD2DEG(pcv_status.position_error[i]));      
      status.set_velocity_error(i, RAD2DEG(pcv_status.velocity_error[i])); 
     } else {
      status.set_global_position(i, pcv_status.global_position[i]);
      status.set_global_velocity(i, pcv_status.global_velocity[i]);
      status.set_local_velocity(i, pcv_status.local_velocity[i]);
      status.set_local_force(i, pcv_status.local_force[i]);
      status.set_local_acceleration(i, pcv_status.local_acceleration[i]);
      status.set_local_position(i, pcv_status.local_position[i]);
      status.set_position_desired(i, pcv_status.position_desired[i]);      
      status.set_velocity_desired(i, pcv_status.velocity_desired[i]);      
      status.set_acceleration_desired(i, pcv_status.acceleration_desired[i]);      
      status.set_position_error(i, pcv_status.position_error[i]);      
      status.set_velocity_error(i, pcv_status.velocity_error[i]);      
     }
  }
  
  for (int i = 0; i < motor_array->GetNumDof()/2; i++)
  {
      status.set_roll_torque_desired(i, pcv_status.roll_torque_Nm[i]);
      status.set_steer_torque_desired(i, pcv_status.steer_torque_Nm[i]);
      status.set_roll_torque_internal(i, pcv_status.roll_torque_internal[i]);
      status.set_steer_torque_internal(i, pcv_status.steer_torque_internal[i]);
      //status.set_steer_angle(i, WrapDeg(RAD2DEG(pcv_status.steer_angle_rad[i])));
      status.set_steer_angle(i, RAD2DEG(pcv_status.steer_angle_rad[i]));
      status.set_steer_velocity(i, RAD2DEG(pcv_status.steer_velocity_rad[i]));
      status.set_roll_angle(i, RAD2DEG(pcv_status.roll_angle_rad[i]));
      status.set_roll_velocity(i, RAD2DEG(pcv_status.roll_velocity_rad[i]));
      status.set_calibrated(i,(int) motor_array->GetJoint(i*2)->IsEncoderCalibrated());
  }
  
  for (int i = 0; i < motor_array->GetNumDof(); i++)
  {
    status.set_motor_torque_desired(i, pcv_status.motor_torque_Nm[i]);
    status.set_motor_current(i, motor_array->GetJoint(i)->GetCurrent());
  }
    
  status.set_bus_voltage(pwr->GetBusVoltage());
  
  //M3_INFO("%f\n", status.motor_current(1) * 0.000257 - status.motor_torque_desired(1));
    ///////////////
  
/*  if (cnt>=0)
  {
    M3_INFO("Max I\n");    
    for (int i = 0; i < 8; i++)
    {
      M3_INFO("%d  %f\n",i,max_tq[i]);
      max_tq[i] = -999999;
    }
    M3_INFO("Min I\n");    
    for (int i = 0; i < 8; i++)
    {
      M3_INFO("%d  %f\n",i,min_tq[i]);
      min_tq[i] = 999999.0;
    }
    M3_INFO("Err, ctrl\n");
    for (int i = 0; i < 3; i++)
    {
      M3_INFO("Pos: %d  %f\n",i,max_pos_err[i]);
      M3_INFO("Vel: %d  %f\n",i,max_vel_err[i]);
      M3_INFO("Max Cxdd: %d  %f\n",i,max_cdxx[i]);
      M3_INFO("Min Cxdd: %d  %f\n",i,min_cdxx[i]);
      max_pos_err[i] = 0.0;
      max_vel_err[i] = 0.0;
      max_cdxx[i] = -99999.0;
      min_cdxx[i] = 99999.0;
    }    
    cnt = -750;
  }
  cnt++;
  for (int i = 0; i< 8; i++)
  {
    if (status.motor_current(i) > max_tq[i])
      max_tq[i] = status.motor_current(i);
    if (status.motor_current(i) < min_tq[i])
      min_tq[i] = status.motor_current(i);
  }
  for (int i = 0; i< 3; i++)
  {
    if (ABS(pcv_status.position_error[i]) > ABS(max_pos_err[i]))
      max_pos_err[i] = pcv_status.position_error[i];
    if (ABS(pcv_status.velocity_error[i]) > ABS(max_vel_err[i]))
      max_vel_err[i] = pcv_status.velocity_error[i];
    if (pcv_status.local_acceleration[i] > max_cdxx[i])
      max_cdxx[i] = pcv_status.local_acceleration[i];
    if (pcv_status.local_acceleration[i] < min_cdxx[i])
      min_cdxx[i] = pcv_status.local_acceleration[i];
  }   */
}

mReal M3Omnibase::WrapDeg(mReal deg)
{
    while (deg > 180)
        deg -= 360;
    while (deg < -180)      
        deg += 360;
    return deg;
}

bool M3Omnibase::LinkDependentComponents()
{
  if (joint_array_name!="")
	motor_array=(M3JointArray*)factory->GetComponent(joint_array_name);			
  if (motor_array==NULL)
  {
		M3_ERR("M3JointArray component %s not found for component %s. Proceeding without it...\n",joint_array_name.c_str(),GetName().c_str());
    return false;
  }
      pwr=(M3Pwr*) factory->GetComponent(pwr_name);
      if (pwr==NULL)
      {
	      M3_INFO("M3Pwr component %s not found for component %s. Proceeding without it...\n",pwr_name.c_str(),GetName().c_str());
      }
    return true;
}

}
