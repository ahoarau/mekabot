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
import m3.head_s2csp_ctrl_pb2 as hpb
from m3.component import M3Component
from m3.unit_conversion import *
import numpy as nu

class M3HeadS2CSPCtrl(M3Component):
	"""Interface for smooth pursuit controller of an S2 head"""
	def __init__(self,name,type='m3head_s2csp_ctrl'):
		M3Component.__init__(self,name,type=type)
		self.status=hpb.M3HeadS2CSPCtrlStatus()
		self.command=hpb.M3HeadS2CSPCtrlCommand()
		self.param=hpb.M3HeadS2CSPCtrlParam()
		self.read_config()
		for i in range(3):
			self.command.target.append(0)
			self.status.xe.append(0)
	
	def enable(self):
		self.command.enable=1

	def disable(self):
		self.command.enable=0

	def update_status(self):
		self.xe=nu.array(self.status.xe,nu.Float32)
		self.theta_des=nu.array(self.status.theta_des,nu.Float32)

	def set_slew_rate_proportion(self,joint,val):
		self.param.slew_des[joint]=val

	def get_slew_rate_proportion(self,joint):
		return self.param.slew_des[joint]

	def set_theta_j2_deg(self,x):
		self.command.theta_des_j2=x

	#Target is in the head-base frame
	def set_target_head_base_frame(self,x):
		self.command.target[0]=x[0]
		self.command.target[1]=x[1]
		self.command.target[2]=x[2]

	#CSP frame is translation of head-base frame to neutral eye frame. Origin translates from CSP to head-base
	def set_target_csp_frame(self,x):
		self.command.target[0]=x[0]+self.param.origin[0]
		self.command.target[1]=x[1]+self.param.origin[0]
		self.command.target[2]=x[2]+self.param.origin[0]

	
