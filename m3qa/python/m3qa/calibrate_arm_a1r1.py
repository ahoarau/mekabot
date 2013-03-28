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
import m3qa.config_arm_a1r1 as a1r1

# ###################################### a1 J0 ##############################################################
config_default_a1_j0={
	'calib':a1r1.config_arm_a1r1_actuator_j0['calib'],
	'param':a1r1.config_arm_a1r1_actuator_j0['param'],
	'param_internal':
	{
		'joint_limits': {'both_arms':[-47.0,197.0],'note':'Positive is reaching upward'}
	}
}

# ######################################## a1 J1 ############################################################
config_default_a1_j1={
	'calib':a1r1.config_arm_a1r1_actuator_j1['calib'],
	'param':a1r1.config_arm_a1r1_actuator_j1['param'],
	'param_internal':
	{
		'joint_limits': {'right_arm':[-19,121],'left_arm':[-121,19],'note':'positive is elbow to its right'}
	}
}
# ########################################## a1 J2 ##########################################################
config_default_a1_j2={
	'calib':a1r1.config_arm_a1r1_actuator_j2['calib'],
	'param':a1r1.config_arm_a1r1_actuator_j2['param'],
	'param_internal':
	{
		'joint_limits': {'both_arms':[-76.0,76.0],'note':'positive is reaching to its right'}
	}
}
# ############################################# a1 J3 #######################################################
config_default_a1_j3={
	'calib':a1r1.config_arm_a1r1_actuator_j3['calib'],
	'param':a1r1.config_arm_a1r1_actuator_j3['param'],
	'param_internal':
	{
		'joint_limits': {'both_arms':[0,140.0],'note':'positive is wrist towards chest'}
	}
}
# ############################################# a1 J4 #######################################################
config_default_a1_j4={
	'calib':a1r1.config_arm_a1r1_actuator_j4['calib'],
	'param':a1r1.config_arm_a1r1_actuator_j4['param'],
	'param_internal':
	{
		'joint_limits': {'right_arm':[-78,123],'left_arm':[-123,78],'note':'positive is top of forearm rotating to its right'}
	}
}
# ############################################# a1 J5 #######################################################
config_default_a1_j5={
	'calib':a1r1.config_arm_a1r1_actuator_j5['calib'],
	'param':a1r1.config_arm_a1r1_actuator_j5['param'],
	'param_internal':
	{
		'joint_limits': {'both_arms':[-45,45],'note': 'positive is hand rotating up'}
	}
}
# ############################################# a1 J6 #######################################################
config_default_a1_j6={
	'calib':a1r1.config_arm_a1r1_actuator_j6['calib'],
	'param':a1r1.config_arm_a1r1_actuator_j6['param'],
	'param_internal':
	{
		'joint_limits': {'both_arms':[-45,45],'note':'positive is fingers rotating to its right'}
	}
}

# ###########################################################################

class M3Calibrate_Arm_A1R1(M3CalibrateActuatorEcA1R1):
	def __init__(self):
		M3CalibrateActuatorEcA1R1.__init__(self)
		
		self.joint_names=['Shoulder J0',
				  'Shoulder J1',
				  'Shoulder J2',
				  'Elbow J3',
				  'Wrist J4',
				  'Wrist J5',
				  'Wrist J6']
		self.config_default=[
			config_default_a1_j0,
			config_default_a1_j1,
			config_default_a1_j2,
			config_default_a1_j3,
			config_default_a1_j4,
			config_default_a1_j5,
			config_default_a1_j6]
		
	def start(self,ctype):
		if not M3CalibrateActuatorEcA1R1.start(self,ctype):
			return False
		self.jid=int(self.comp_ec.name[self.comp_ec.name.find('_j')+2:])
		self.param_internal=self.config_default[self.jid]['param_internal']
		self.calib_default=self.config_default[self.jid]['calib']
		self.param_default=self.config_default[self.jid]['param']
		print 'Calibrating joint',self.joint_names[self.jid]
		return True

	    