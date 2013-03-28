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
from m3qa.calibrate_actuator_ec_r3 import *
import m3.actuator_ec_pb2 as aec
import m3qa.config_arm_a2r3 as a2r3

# ###################################### A2 J0 ##############################################################
config_default_a2_j0={
	'calib':a2r3.config_arm_a2r3_actuator_j0['calib'],
	'param':a2r3.config_arm_a2r3_actuator_j0['param'],
	'param_internal':
	{
		'calib_tq_lc_amp':2500.0,
		'calib_lever_mass':327.6,
		'calib_lever_com':157.5,
		'calib_lever_len':304.8,
		'joint_limits': {'both_arms':[-80.0,200.0],'note':'Positive is reaching upward'},
		'calib_tq_degree':1,
		'calib_hub_diam':70
	}
}

# ######################################## A2 J1 ############################################################
config_default_a2_j1={
	'calib':a2r3.config_arm_a2r3_actuator_j1['calib'],
	'param':a2r3.config_arm_a2r3_actuator_j1['param'],
	'param_internal':
	{
		'calib_tq_lc_amp':2500.0,
		'calib_lever_mass':327.6,
		'calib_lever_com':157.5,
		'calib_lever_len':304.8,
		'joint_limits': {'right_arm':[-24.1,150.46],'left_arm':[-150.46,24.1],'note':'positive is elbow to its right'},
		'calib_tq_degree':1,
		'calib_hub_diam':70
	}
}
# ########################################## A2 J2 ##########################################################
config_default_a2_j2={
	'calib':a2r3.config_arm_a2r3_actuator_j2['calib'],
	'param':a2r3.config_arm_a2r3_actuator_j2['param'],
	'param_internal':
	{
		'calib_tq_lc_amp':2500.0,
		'calib_lever_mass':327.6,
		'calib_lever_com':157.5,
		'calib_lever_len':304.8,
		'joint_limits': {'both_arms':[-85.0,85.0],'note':'positive is reaching to its right'},
		'calib_tq_degree':1,
		'calib_hub_diam':70
	}
}
# ############################################# A2 J3 #######################################################
config_default_a2_j3={
	'calib':a2r3.config_arm_a2r3_actuator_j3['calib'],
	'param':a2r3.config_arm_a2r3_actuator_j3['param'],
	'param_internal':
	{
		'calib_tq_lc_amp':2500.0,
		'calib_lever_mass':327.6,
		'calib_lever_com':157.5,
		'calib_lever_len':304.8,
		'joint_limits': {'both_arms':[0,133.0],'note':'positive is wrist towards chest'},
		'calib_tq_degree':1,
		'calib_hub_diam':70
	}
}
# ############################################# A2 J4 #######################################################
config_default_a2_j4={
	'calib':a2r3.config_arm_a2r3_actuator_j4['calib'],
	'param':a2r3.config_arm_a2r3_actuator_j4['param'],
	'param_internal':
	{
		'calib_tq_lc_amp':1750.0,
		'calib_lever_mass':128.7,
		'calib_lever_com':89.59,
		'calib_lever_len':175.0, #use center hole
		'joint_limits': {'right_arm':[110,-110],'left_arm':[-110,110],'note':'positive is top of forearm rotating to its right'},
		'calib_tq_degree':1,
		'calib_hub_diam':70
	}
}
# ############################################# A2 J5 #######################################################
config_default_a2_j5={
	'calib':a2r3.config_arm_a2r3_actuator_j5['calib'],
	'param':a2r3.config_arm_a2r3_actuator_j5['param'],
	'param_internal':
	{
		'calib_tq_lc_amp':1750.0,
		'calib_lever_mass':128.7,
		'calib_lever_com':85.41,
		'calib_lever_len':187.8, #use furthest hole
		'joint_limits': {'both_arms':[-58.81,58.81],'note': 'positive is hand rotating up'},
		'calib_tq_degree':1,
		'calib_hub_diam':70
	}
}
# ############################################# A2 J6 #######################################################
config_default_a2_j6={
	'calib':a2r3.config_arm_a2r3_actuator_j6['calib'],
	'param':a2r3.config_arm_a2r3_actuator_j6['param'],
	'param_internal':
	{
		'calib_tq_lc_amp':1750.0,
		'calib_lever_mass':128.7,
		'calib_lever_com':85.41,
		'calib_lever_len':187.8, #use furthest hole
		'joint_limits': {'both_arms':[-60.0,60.0],'note':'positive is fingers rotating to its right'},
		'calib_tq_degree':1,
		'calib_hub_diam':70
	}
}

# ###########################################################################

class M3Calibrate_Arm_A2R3(M3CalibrateActuatorEcR3):
	def __init__(self):
		M3CalibrateActuatorEcR3.__init__(self)
		
		self.joint_names=['Shoulder J0',
				  'Shoulder J1',
				  'Shoulder J2',
				  'Elbow J3',
				  'Wrist J4',
				  'Wrist J5',
				  'Wrist J6']
		self.config_default=[
			config_default_a2_j0,
			config_default_a2_j1,
			config_default_a2_j2,
			config_default_a2_j3,
			config_default_a2_j4,
			config_default_a2_j5,
			config_default_a2_j6]
		
	def start(self,ctype):
		if not M3CalibrateActuatorEcR3.start(self,ctype):
			return False
		self.jid=int(self.comp_ec.name[self.comp_ec.name.find('_j')+2:])
		self.param_internal=self.config_default[self.jid]['param_internal']
		self.calib_default=self.config_default[self.jid]['calib']
		self.param_default=self.config_default[self.jid]['param']
		print 'Calibrating joint',self.joint_names[self.jid]
		return True

	    