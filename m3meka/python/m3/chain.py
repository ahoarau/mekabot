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
import m3.joint_array_pb2 as mab
#import m3.joint_array as mja
from m3.joint_array import M3JointArray


class M3Chain(M3JointArray):
    """Interface for a serial kinematic chain. 
    """
    def __init__(self,name,ndof,ctype):
        M3JointArray.__init__(self,name,ndof,ctype)

        # Grow the protocol buffer messages to correct size
        for i in range(6):
            self.command.pos_desired.append(0)

    def set_payload_inertia(self,I): #Kg m^2 3x3 matrix
        """Set the payload inertial tensor matrix"""
        self.param.payload_inertia[0]=float(I[0,0])#Ixx
        self.param.payload_inertia[1]=float(I[0,1])#Ixy
        self.param.payload_inertia[2]=float(I[0,2])#Ixz
        self.param.payload_inertia[3]=float(I[1,1])#Iyy
        self.param.payload_inertia[4]=float(I[1,2])#Iyz
        self.param.payload_inertia[5]=float(I[2,2])#Izz

    def set_end_pose(self,v,ind=None):
        """m / rad units"""
        M3Component.set_float_array(self,self.command.pos_desired,v,ind)
