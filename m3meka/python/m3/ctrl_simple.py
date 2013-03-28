#! /usr/bin/python

#*************************************************************************
# 
# REDWOOD CONFIDENTIAL
# Author: Aaron Edsinger
# __________________
# 
#  [2012] - [+] Redwood Robotics Incorporated 
#  All Rights Reserved.
# 
# All information contained herein is, and remains
# the property of Redwood Robotics Incorporated and its suppliers,
# if any.  The intellectual and technical concepts contained
# herein are proprietary to Redwood Robotics Incorporated
# and its suppliers and may be covered by U.S. and Foreign Patents,
# patents in process, and are protected by trade secret or copyright law.
# Dissemination of this information or reproduction of this material
# is strictly forbidden unless prior written permission is obtained
# from Redwood Robotics Incorporated.
#
import yaml
import os 
from m3.toolbox import *
import m3.ctrl_simple_pb2
import m3.actuator_pb2  as a
from m3.component import M3Component
from m3.unit_conversion import *

	
class M3CtrlSimple(M3Component):
	"""Calibrated interface for the M3 ctrl simple"""
	def __init__(self,name,type='m3ctrl_simple'):
		M3Component.__init__(self,name,type=type)
		self.status		= m3.ctrl_simple_pb2.M3CtrlSimpleStatus()
		self.command	= m3.ctrl_simple_pb2.M3CtrlSimpleCommand()
		self.param		= m3.ctrl_simple_pb2.M3CtrlSimpleParam()
		self.read_config()

# SETTERS
	# modes
	def set_control_mode(self,m):
		self.command.ctrl_mode = m
		
	def set_traj_mode(self,m):
		self.command.traj_mode = m
	
	def set_mode_off(self):
		self.command.ctrl_mode = m3.ctrl_simple_pb2.CTRL_MODE_OFF

	def set_mode_current(self):
		self.command.ctrl_mode = m3.ctrl_simple_pb2.CTRL_MODE_CURRENT
		
	def set_mode_theta(self):
		self.command.ctrl_mode = m3.ctrl_simple_pb2.CTRL_MODE_THETA

	def set_mode_torque(self):
		self.command.ctrl_mode = m3.ctrl_simple_pb2.CTRL_MODE_TORQUE
		
		
	# commands
	def set_current(self, t):
		self.command.desired_current = t
	def set_theta(self, t):
		self.command.desired_theta = t
	def set_theta_deg(self, t):
		self.command.desired_theta = t*math.pi/180.0
	def set_torque(self, t):
		self.command.desired_torque = t

# GETTERS
	
	# misc	
	def get_timestamp_uS(self):
		return self.status.base.timestamp
	
	def get_flags(self):
		return self.status.flags
	
