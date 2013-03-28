#M3 -- Meka Robotics Robot Components
#Copyright (c) 2010 Meka Robotics
#Author: edsinger@mekabot.com (Aaron Edsinger)

#M3 is free software: you can redistribute it and/or modify
#it under the terms of the GNU Lesser General Public License as published by
#the Free Software Foundation, either version 3 of the License, or
#(at your option) any later version.

#M3 is distributed in the hope that it will be useful,
#but WITHOUT ANY WARRANTY; without even the implied warranty of
#MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#GNU Lesser General Public License for more details.

#You should have received a copy of the GNU Lesser General Public License
#along with M3.  If not, see <http://www.gnu.org/licenses/>.

import yaml
import os 
import m3.toolbox as m3t
import m3.toolbox_ctrl as m3tc
import m3.joint_array_pb2 as mab
import m3.joint_array_mode_pb2 as mam
import m3.smoothing_mode_pb2 as msm
import m3.joint_pb2 as mrt
from m3.component import M3Component
from m3.unit_conversion import *

import scipy.linalg
import numpy as nu


class M3JointArray(M3Component):
    """Interface for a joint array.
    The kinematic data and sensor data from all DOF is loaded into Numeric arrays.
    on each update from the server. 
    If not specified, all units are meters/mNm/radians/seconds.

    The set_xxx commands support a number of different forms:
    For example:
    -- set_pwm(a) : sets all joints to a
    -- set_pwm(a,[1,2,3]), set joints 1,2,3 to a
    -- set_pwm(a,1), set joint 1 to a
    -- set_pwm([a b c],[1,2,3]) set joints 1,2,3 to a,b,c
    -- set_pwm([a,b,...]): sets all joints to a,b,...
    """

    def __init__(self,name,ndof,ctype):
        M3Component.__init__(self,name,ctype)
	self.read_config()
        self.status=mab.M3JointArrayStatus()
        self.command=mab.M3JointArrayCommand()
        self.param=mab.M3JointArrayParam()
	if ndof is None:
	    ndof=self.config['ndof']
        self.ndof=ndof
        self.vias=[]
        self.via_idx=0
        # Grow the protocol buffer messages to correct size
        # NOTE: should really support mixed sizes but...
        for i in range(self.ndof):
            self.command.pwm_desired.append(0)
            self.command.tq_desired.append(0)
            self.command.q_desired.append(0)
            self.command.qdot_desired.append(0)
            self.command.q_stiffness.append(0)
	    self.command.q_slew_rate.append(0)
	    self.command.smoothing_mode.append(msm.SMOOTHING_MODE_OFF)
            self.command.ctrl_mode.append(mam.JOINT_ARRAY_MODE_OFF)
        
        self.motor_temp=nu.zeros(self.ndof,float)
        self.amp_temp=nu.zeros(self.ndof,float)
        self.torque=nu.zeros(self.ndof,float)
        self.torquedot=nu.zeros(self.ndof,float)	
        self.current=nu.zeros(self.ndof,float)
        self.theta=nu.zeros(self.ndof,float)
        self.thetadot=nu.zeros(self.ndof,float)
        self.thetadotdot=nu.zeros(self.ndof,float)
	
	self.max_slew_rates=self.__get_max_slew_rates_from_config()

    
    def get_limb_name(self): 
        """Unique limb name: 'left_arm','right_arm',etc"""
        return self.config['limb_name'] 

    def get_num_dof(self):
        return self.ndof    

    def get_motor_temp_C(self): 
        return self.motor_temp
    def get_motor_temp_F(self): 
        return C2F(self.motor_temp)
    def get_amp_temp_C(self): 
        return self.amp_temp
    def get_amp_temp_F(self): 
        return C2F(self.amp_temp)
    def get_current_mA(self): 
        return self.current
    def get_total_current_mA(self):
        return nu.sum(self.current)

    def get_torque_mNm(self): 
        return self.torque
    def get_torque_inLb(self): 
        return mNm2inLb(self.torque)
    def get_torquedot_mNm(self): 
        return self.torquedot
    def get_torquedot_inLb(self): 
        return mNm2inLb(self.torquedot)       
    def get_theta_deg(self): 
        return self.theta
    def get_theta_rad(self): 
        return deg2rad(self.theta)
    def get_thetadot_rad(self): 
        return deg2rad(self.thetadot)
    def get_thetadot_deg(self): 
        return self.thetadot
    def get_thetadotdot_rad(self): 
        return deg2rad(self.thetadotdot)
    def get_thetadotdot_deg(self): 
        return self.thetadotdot
    def get_timestamp_uS(self):
        return self.status.base.timestamp
    
    def get_pwm_cmd(self):
        return self.status.pwm_cmd
    def get_flags(self):
        return self.status.flags
    def get_limitswitch_pos(self):
	return [ int((x & aepb.ACTUATOR_EC_FLAG_POS_LIMITSWITCH)!=0) for x in self.status.flags]
    def get_limitswitch_neg(self):
	return [int((x & aepb.ACTUATOR_EC_FLAG_NEG_LIMITSWITCH)!=0) for x in self.status.flags]
    def get_encoder_calibrated(self):
	return [int((x & aepb.ACTUATOR_EC_FLAG_QEI_CALIBRATED)!=0) for x in self.status.flags]
    
    def set_pwm(self,v,ind=None):
        M3Component.set_int_array(self,self.command.pwm_desired,v,ind)
    def set_torque_mNm(self,v,ind=None):
        M3Component.set_float_array(self,self.command.tq_desired,v,ind)
    def set_torque_inLb(self,v,ind=None):
        M3Component.set_float_array(self,self.command.tq_desired,inLb2mNm(nu.array(v)),ind)
    def set_theta_rad(self,v,ind=None):
        M3Component.set_float_array(self,self.command.q_desired,rad2deg(nu.array(v)),ind)
    def set_theta_deg(self,v,ind=None):
        M3Component.set_float_array(self,self.command.q_desired,v,ind)
    def set_stiffness(self,v,ind=None):
        M3Component.set_float_array(self,self.command.q_stiffness,v,ind)
    def set_slew_rate(self,v):
        M3Component.set_float_array(self,self.command.q_slew_rate,v,ind)
    def set_slew_rate_proportion(self,v,ind=None):
	slew_rates = []
	if ind is not None:
	    for i in range(len(ind)):
	    	slew_rates.append(self.max_slew_rates[ind[i]]*max(0.0,min(1.0,v[i])))
	else:
	    for i in range(len(v)):
		slew_rates.append(self.max_slew_rates[i]*max(0.0,min(1.0,v[i])))	
	M3Component.set_float_array(self,self.command.q_slew_rate,slew_rates,ind)
    def set_mode(self,v,ind=None):
        M3Component.set_int_array(self,self.command.ctrl_mode,v,ind)
    def set_mode_off(self,ind=None):
        M3Component.set_int_array(self,self.command.ctrl_mode,mam.JOINT_ARRAY_MODE_OFF,ind)
    def set_mode_pwm(self,ind=None):
        M3Component.set_int_array(self,self.command.ctrl_mode,mam.JOINT_ARRAY_MODE_PWM,ind)
    def set_mode_torque(self,ind=None):
        M3Component.set_int_array(self,self.command.ctrl_mode,mam.JOINT_ARRAY_MODE_TORQUE,ind)
    def set_mode_torque_gc(self,ind=None):
        M3Component.set_int_array(self,self.command.ctrl_mode,mam.JOINT_ARRAY_MODE_TORQUE_GC,ind)
    def set_mode_theta(self,ind=None):
        M3Component.set_int_array(self,self.command.ctrl_mode,mam.JOINT_ARRAY_MODE_THETA,ind)
    def set_mode_theta_gc(self,ind=None):
        M3Component.set_int_array(self,self.command.ctrl_mode,mam.JOINT_ARRAY_MODE_THETA_GC,ind)
    def set_mode_theta_mj(self,ind=None):
        M3Component.set_int_array(self,self.command.ctrl_mode,mam.JOINT_ARRAY_MODE_THETA_MJ,ind)
    def set_mode_theta_gc_mj(self,ind=None):
        M3Component.set_int_array(self,self.command.ctrl_mode,mam.JOINT_ARRAY_MODE_THETA_GC_MJ,ind)
    def set_mode_splined_traj(self,ind=None):
        M3Component.set_int_array(self,self.command.ctrl_mode,mam.JOINT_ARRAY_MODE_SPLINED_TRAJ,ind)
    def set_mode_splined_traj_gc(self,ind=None):
        M3Component.set_int_array(self,self.command.ctrl_mode,mam.JOINT_ARRAY_MODE_SPLINED_TRAJ_GC,ind)

    def add_splined_traj_via_rad(self,theta_des,thetadot_avg):
        """Add a desired via point to the queue. 
	-- thetadot_avg is the desired average velocity of the joint with the farthest to travel
	Requires that a NDOF are specified even if not all are in via control mode"""
        self.vias.append([rad2deg(nu.array(theta_des,float)),rad2deg(nu.array(thetadot_avg))])

    def add_splined_traj_via_deg(self,theta_des,thetadot_avg):
        self.vias.append([theta_des,thetadot_avg])

    def is_splined_traj_complete(self):
        """Have all the joint vias finished executing on the server"""
        return len(self.vias)==0 and self.via_idx==self.status.completed_spline_idx

    def load_command(self):
        self.command.ClearField('vias')
        nadd=min(20,len(self.vias)) #only add 20 per cycle to keep packet size down
        for n in range(nadd):
            self.via_idx=self.via_idx+1
            theta_des=self.vias[n][0]
            thetadot_avg=self.vias[n][1]
            self.command.vias.add()
            for i in range(self.ndof):
                self.command.vias[-1].q_desired.append(float(theta_des[i]))
                self.command.vias[-1].qdot_avg.append(float(thetadot_avg[i]))
            self.command.vias[-1].idx=self.via_idx
        self.vias=self.vias[nadd:]

    def update_status(self):
        self.motor_temp=nu.array(self.status.motor_temp,float)
        self.amp_temp=  nu.array(self.status.amp_temp,float)
        self.current=  nu.array(self.status.current,float)
        self.torque=  nu.array(self.status.torque,float)
        self.torquedot=  nu.array(self.status.torquedot,float)	
        self.theta=  nu.array(self.status.theta,float)
        self.thetadot=  nu.array(self.status.thetadot,float)
        self.thetadotdot=  nu.array(self.status.thetadotdot,float)

    def __get_max_slew_rates_from_config(self):
	file_name = m3t.get_component_config_filename(self.name)
	try:
            f=file(file_name,'r')
            config= yaml.safe_load(f.read())
        except (IOError, EOFError):
            print 'Config file not present:',file_name
            return
	max_slew_rates = [0.0]*self.get_num_dof()
	for k in config['joint_components']:
	    joint_name = config['joint_components'][k]
	    joint_file_name = m3t.get_component_config_filename(joint_name)
	    try:
		jf=file(joint_file_name,'r')
		joint_config= yaml.safe_load(jf.read())
	    except (IOError, EOFError):
		print 'Config file not present:', joint_file_name
		return	    
	    max_slew_rates[int(k[1:])] = joint_config['param']['max_q_slew_rate']
	return max_slew_rates