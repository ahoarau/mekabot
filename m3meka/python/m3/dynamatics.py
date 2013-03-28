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
from m3.toolbox import *
import m3.dynamatics_pb2 as mrt
import m3.toolbox as m3t
import m3.toolbox_ctrl as m3tc
from m3.component import M3Component
from m3.unit_conversion import *
import numpy as nu
from PyKDL import *
import scipy.linalg

class M3Dynamatics(M3Component):
    """Interface for joint-space controllers of a M3Sea
    """
    def __init__(self,name,type='m3dynamatics'):
        M3Component.__init__(self,name,type=type)
        self.status=mrt.M3DynamaticsStatus()
        self.command=mrt.M3DynamaticsCommand()
        self.param=mrt.M3DynamaticsParam()
        
	for i in range(3):
	    self.param.payload_com.append(0)
	for i in range(6):
            self.param.payload_inertia.append(0)
	    
	self.read_config()	

	self.T80=nu.zeros([4,4],nu.float32) #Transform from end to base frame
	self.T08=nu.zeros([4,4],nu.float32)  #Transform from base to end frame
	self.G=nu.zeros(self.ndof+1,nu.float32) #Gravity vector on joints
	self.C=nu.zeros(self.ndof+1,nu.float32) #Coriolis vector on joints
	self.J=nu.zeros([6,self.ndof],nu.float32) #Jacobian from joint torques to end torques
	self.Jt=nu.zeros([self.ndof,6],nu.float32)#Transform end wrench to joint torques.
	self.end_twist=nu.zeros(6,nu.float32)
	self.end_pos=nu.zeros(3,nu.float32)
	self.end_rot=nu.zeros([3,3],nu.float32)
	#self.set_tool_transform(nu.identity(4,nu.float32)) #default
	
		
    #Utility API
    def get_end_twist(self): 
	return self.end_twist
   
	
    # ##########################################################
    """ End and Eff frame related functions 
    Assumes that wrench is 6x1 Numeric array. 
    Assumes that positions are [x,y,z,1] Numeric array. 
    All units in meters/mNm/mN/radians/seconds unless otherwise noted.
    """
    def get_end_force(self):
	print 'get_end_force not yet implemented'
	return nu.zeros(3,nu.float32)
	#return self.end_wrench[:3]
	
    def get_end_moment(self):
	print 'get_end_moment not yet implemented'
	return nu.zeros(3,nu.float32)
	#return self.end_wrench[3:]
	
    def get_end_velocity(self):
	return self.end_twist[:3]
    
    def get_end_angular_velocity(self):
	return self.end_twist[3:]
    
    def get_end_roll_pitch_yaw_rad(self):
	return nu.array(self.__get_end_rotation_kdl().GetRPY(),float)
    
    def get_end_euler_zyx_rad(self):
	return nu.array(self.__get_end_rotation_kdl().GetEulerZYX(),float)
    
    def get_end_euler_zyx_deg(self):
	return rad2deg(self.get_end_euler_zyx_rad())
    
    def get_end_euler_zyz_rad(self):
	return nu.array(self.__get_end_rotation_kdl().GetEulerZYZ())
    
    def get_end_euler_zyz_deg(self):
	return rad2deg(self.get_end_euler_zyz_rad())
        
    def __get_end_rotation_kdl(self):
	c = self.end_rot
	return Rotation(c[0,0],c[0,1],c[0,2],c[1,0],c[1,1],c[1,2],c[2,0],c[2,1],c[2,2])
    
    def get_end_roll_pitch_yaw_deg(self):
	return rad2deg(self.get_end_roll_pitch_yaw_rad())
    
    def get_end_position(self):
	return self.end_pos
    
    def get_end_position_In(self):
	return m2in(self.end_pos)
    
    def end_wrench_2_joint_torques(self,w):
	return nu.dot(self.Jt*nu.array(w))
    
    def joint_torques_2_end_wrench(self,tq):
	#return nu.dot(self.J,nu.array(tq))
	print 'get_end_force not yet implemented'
	return nu.zeros(6,nu.float32)
    
    def thetadot_2_end_twist(self,td):
	return nu.dot(self.J,nu.array(td))
    
    def eff_frame_2_base_frame(self,x):
	return nu.dot(self.T80,x)
    
    def base_frame_2_eff_frame(self,x):
	return nu.dot(self.T08,x)
      
	    
    
    # ##########################################################
    """ Tool frame related functions 
     Assumes that wrench is 6x1 Numeric array. 
     Assumes that positions are [x,y,z,1] Numeric array. 
     All units in meters/mNm/radians/seconds unless otherwise noted.."""
    
    def set_tool_transform(self,T):
	""" Set the homgenous transform from tool frame to end frame"""
	self.T2E = nu.array(T,nu.float32) #Transform point in hand frame to arm tool.
	self.E2T =  nu.transpose(self.T2E)  #Transform point in arm tool to hand frame.
	self.FE2FT = m3tc.force_moment_transform(self.E2T)  #Transform wrench in end frame to tool frame
	self.FT2FE = m3tc.force_moment_transform(self.T2E)  #Transform wrench in tool frame to end frame
	
    def tool_wrench_2_joint_torques(self,wrench):
	return self.end_wrench_2_joint_torques(self.tool_wrench_2_end_wrench(wrench))
    
    def joint_torques_2_tool_wrench(self,tq):
	return self.end_wrench_2_tool_wrench(self.joint_torques_2_end_wrench(tq))
    
    def tool_wrench_2_end_wrench(self,wrench):
	return nu.matrixmultiply(self.tool_wrench_2_end_wrench_transform(),wrench)
    
    def end_wrench_2_tool_wrench(self,wrench):
	return nu.matrixmultiply(self.end_wrench_2_tool_wrench_transform(),wrench)
    
    def end_wrench_2_eff_wrench_transform(self):
	T=self.T08.copy()
	T[0:3,3]=0
	return m3tc.force_moment_transform(T)
    
    def eff_wrench_2_end_wrench_transform(self):
	T=self.T80.copy()
	T[0:3,3]=0
	return m3tc.force_moment_transform(T)
    
    def tool_wrench_2_end_wrench_transform(self):
	return nu.matrixmultiply(self.eff_wrench_2_end_wrench_transform(),self.FT2FE)
    
    def end_wrench_2_tool_wrench_transform(self):
	return nu.matrixmultiply(self.FE2FT,self.end_wrench_2_eff_wrench_transform())
	
	
    def base_frame_2_tool_frame(self,x):
	return nu.dot(self.E2T,self.base_frame_2_eff_frame(x))
    
    def tool_frame_2_base_frame(self,x):
	return self.eff_frame_2_base_frame(nu.dot(self.T2E,x))
    # ##########################################################
    
    def get_torque_gravity_mNm(self): 
	return self.G
    def get_torque_gravity_inLb(self): 
	return mNm2inLb(self.G)
    
    def set_payload_com(self,com): #meters
	"""Set the payload center-of-mass (tool Frame)"""
	self.param.payload_com[0]=float(com[0])
	self.param.payload_com[1]=float(com[1])
	self.param.payload_com[2]=float(com[2])
    def set_payload_mass(self,m): #Kg
	"""Set the estimated payload mass"""
	self.param.payload_mass=m
 
    def get_end_rotation(self):	
	return self.end_rot
    
    def get_end_frame(self):
	return self.T80
	
    def update_status(self):
	
	self.end_rot = self.list_to_numpy_rotation(self.status.end_rot)	
	self.end_pos = nu.array(self.status.end_pos,nu.float32)	
	self.T80[:3,:3] = self.end_rot
	self.T80[0,3] = self.end_pos[0]
	self.T80[1,3] = self.end_pos[1]
	self.T80[2,3] = self.end_pos[2]	
	self.T80[3,3] = 1	
	self.T08=scipy.linalg.inv(self.T80)
	self.G=nu.array(self.status.G,nu.float32)	
	self.J=nu.array(self.status.J,nu.float32).resize([6,self.ndof])
	self.Jt=nu.transpose(self.J)	
	self.end_twist=nu.array(self.status.end_twist,nu.float32)
	
    def list_to_numpy_rotation(self, l):
	mtx = mtx = nu.zeros([3,3])
	mtx[0,0] = l[0]
	mtx[0,1] = l[1]
	mtx[0,2] = l[2]
	mtx[1,0] = l[3]
	mtx[1,1] = l[4]
	mtx[1,2] = l[5]
	mtx[2,0] = l[6]
	mtx[2,1] = l[7]
	mtx[2,2] = l[8]
	return mtx

    def read_config(self):
	M3Component.read_config(self)
	
	try:
            f=file(self.config_name,'r')
            config= yaml.safe_load(f.read())
        except (IOError, EOFError):
            print 'Config file not present:',self.config_name
            return
	
	self.ndof = config['ndof']
        
    
