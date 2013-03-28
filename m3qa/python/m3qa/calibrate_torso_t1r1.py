# -*- coding: utf-8 -*-
#Copyright  2008, Meka Robotics
#All rights reserved.
#http://mekabot.com

#Redistribution and use in source and binary forms, with or without
#modification, are permitted. 


#THIS SOFTWARE IS PROVIDED BY THE Copyright HOLDERS AND CONTRIBUTORS
#"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#Copyright OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES INCLUDING,
#BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#POSSIBILITY OF SUCH DAMAGE.

import time
import numpy.numarray as na
import Numeric as nu
import math
import os 
import sys
import yaml
import m3.unit_conversion as m3u
from m3qa.calibrate import *
from m3qa.calibrate_sensors import *
from m3qa.calibrate_actuator_ec_a1r1 import *
import m3.actuator_ec_pb2 as aec
import m3qa.config_torso_t1r1 as t1r1

# ###################################### t1 J0 ##############################################################
config_default_t1_j0={
	'calib':t1r1.config_torso_t1r1_actuator_j0['calib'],
	'param':t1r1.config_torso_t1r1_actuator_j0['param'],
	'param_internal':
	{
		'joint_limits': [-90.0,90.0],
		'pwm_theta':[-700,700]
	}
}

# ######################################## t1 J1 ############################################################
config_default_t1_j1={
	'calib':t1r1.config_torso_t1r1_actuator_j1['calib'],
	'param':t1r1.config_torso_t1r1_actuator_j1['param'],
	'param_internal':
	{
		'joint_limits': [-11,53],
		'pwm_theta':[-100,250]
	}
}

# ###########################################################################

class M3Calibrate_Torso_T1R1(M3CalibrateActuatorEcA1R1):
	def __init__(self):
		M3CalibrateActuatorEcA1R1.__init__(self)
		
		self.joint_names=['J0',
				  'J1']
		self.config_default=[
			config_default_t1_j0,
			config_default_t1_j1]
		
	def start(self,ctype):
		if not M3CalibrateActuatorEcA1R1.start(self,ctype):
			return False
		self.jid=int(self.comp_ec.name[self.comp_ec.name.find('_j')+2:])
		self.param_internal=self.config_default[self.jid]['param_internal']
		self.calib_default=self.config_default[self.jid]['calib']
		self.param_default=self.config_default[self.jid]['param']
		print 'Calibrating joint',self.joint_names[self.jid]
		return True
	
	def do_task(self,ct):
		if ct=='tt':
			self.reset_sensor('theta')
			if self.jid==0:
			  self.calibrate_theta(use_pwm=False,use_brake=False)
			else:
			  self.calibrate_theta(use_pwm=True,use_brake=False)
			self.write_config()
			return True
		if M3CalibrateActuatorEcR1.do_task(self,ct):
			return True
		return False
	
	def print_tasks(self):
		M3CalibrateActuatorEcA1R1.print_tasks(self)
	    