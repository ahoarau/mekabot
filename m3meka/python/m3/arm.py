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

from m3.chain import M3Chain
import m3.toolbox as m3t
import m3.toolbox_ctrl as m3tc
import scipy.linalg
import numpy as nu
from m3.unit_conversion import *

#Wrapper only
class M3Arm(M3Chain):
	"""Wrapper for 7DOF SEA Arm"""
	def __init__(self,name):
		M3Chain.__init__(self,name,ndof=7,ctype='m3arm')
		self.set_fts_transform(nu.identity(4,float)) #default
		self.ikjt_q=None
	# ############################################################################# 

	"""Force-torque sensor support.
	Assumes that wrench is 6x1 Numeric array. 
	See documentation for frame definitions"""

	def set_fts_transform(self,T):
		""" Set the homogenous transform from the force-torque-sensor frame to eff frame"""
		self.S2E = nu.array(T,float) #Transform point in sensor frame to eff
		self.E2S =  scipy.linalg.inv(self.S2E)  #Transform point in eff to sensor frame.
		self.FS2FE = m3tc.force_moment_transform(self.S2E)  #Transform wrench in sensor frame to eff frame
		self.FE2FS = m3tc.force_moment_transform(self.E2S)  #Transform wrench in eff frame to sensor frame

	def fts_wrench_to_tool_wrench(self,wrench):
		effw=nu.matrixmultiply(self.FS2FE,wrench)
		endw=nu.matrixmultiply(self.eff_wrench_2_end_wrench_transform(),effw)
		return self.end_wrench_2_tool_wrench(endw)

	def tool_wrench_to_fts_wrench(self,wrench):
		endw=self.tool_wrench_2_end_wrench(wrench) 
		effw=nu.matrixmultiply(self.end_wrench_2_eff_wrench_transform(),endw)
		return nu.matrixmultiply(self.FE2FS,effw)

	def fts_wrench_to_joint_torques(self,wrench):
		effw=nu.matrixmultiply(self.FS2FE,wrench)
		endw=nu.matrixmultiply(self.eff_wrench_2_end_wrench_transform(),effw)
		return self.end_wrench_2_joint_torques(endw)

	def joint_torques_to_fts_wrench(self,tq):
		endw=self.joint_torques_2_end_wrench(tq)
		effw=nu.matrixmultiply(self.end_wrench_2_eff_wrench_transform(),endw)
		return nu.matrixmultiply(self.FE2FS,effw)

	# ############################################################################# 

	def start_ikjt_simple(self):
		self.ikjt_q=self.get_theta_rad()

	def step_ikjt_simple(self,target,step_size):
		""" A simple method for doing inverse-kinematics using the Jacobian Transpose. Because
	of the redundant DOF , this only good for small excursions from a known
	good posture in joint-space. Othewise strange elbow configurations can arise.
	"""
		self.set_virtual_theta_rad(self.ikjt_q)
		x=self.get_virtual_end_position()
		dx=(target-x)*step_size
		verror=nu.sqrt(sum((target-x)**2))
		dx.resize(6) #roll/pitch/yaw=0
		dq= nu.matrixmultiply(nu.transpose(self.vJ),dx)
		self.ikjt_q=self.ikjt_q+dq
		return verror,self.ikjt_q







