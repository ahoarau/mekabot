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
from m3qa.calibrate_actuator_ec_r1 import *
import m3.actuator_ec_pb2 as aec

# ###################################### A2 J0 ##############################################################
config_default_a2_j0={
	'calib':{
		'motor':{
			'name': 'Maxon RE40 150W 24V',
			'winding_resistance': .316,
			'thermal_resistance_housing_ambient': 4.7,
			'thermal_resistance_rotor_housing': 1.9,
			'max_winding_temp': 155,
			'gear_ratio': 120.0,
			'thermal_time_constant_winding': 41.0
			},
	      'theta':{
		      'type': 'vertx_14bit',
		      'name': 'ContElec VertX13',
		      'cb_scale': 1.0,
		      'cb_bias': 0.0},
	      'amp_temp':{
		      'type': 'adc_linear_5V', #divider
		      'name': 'Microchip TC1047',
		      'cb_mV_at_25C': 750.0,
		      'cb_mV_per_C': 10.0,
		      'cb_scale': 1.0,
		      'cb_bias': 0.0,
		      },
	      'motor_temp':{
		      'type': 'adc_linear_3V3', #no divider 
		      'name': 'Analog TMP36',
		      'cb_mV_at_25C': 750.0,
		      'cb_mV_per_C': 10.0,
		      'cb_scale': 1.0,
		      'cb_bias': 0.0},
	      'torque':{
		      'type': 'sea_vertx_14bit',
		      'name': 'ContElec VertX13',
		      'cb_inv_torque': [1,0],
		      'cb_torque': [1,0],
		      'cb_scale': 1.0,
		      'cb_bias': 0.0},
	      'current':{
		      'type': 'adc_linear_5V',
		      'name': 'Allegro ACS712-20',
		      'cb_mV_per_A': 100.0,
		      'cb_ticks_at_zero_a': 0.0,
		      'cb_ticks_at_zero_b': 0.0,
		      'cb_scale': 1.0,
		      'cb_bias': 0.0},
	      },
	'param':{
		'max_amp_temp': 100.0,
		'max_current': 12000,
		'max_motor_temp': 145.0,
		'max_tq': 50000.0,
		'min_tq': -50000.0,
		'thetadot_deadband': 1.0
		},
	'param_internal':
	{
		'calib_tq_lc_amp':2000.0,
		'analyze_tq_lc_amp':10000.0,
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
	'calib':{
		'motor':{
			'name': 'Maxon RE40 150W 24V',
			'winding_resistance': .316,
			'thermal_resistance_housing_ambient': 4.7,
			'thermal_resistance_rotor_housing': 1.9,
			'max_winding_temp': 155,
			'gear_ratio': 100.0,
			'thermal_time_constant_winding': 41.0
			},
	      'theta':{
		      'type': 'vertx_14bit',
		      'name': 'ContElec VertX13',
		      'cb_scale': 1.0,
		      'cb_bias': 0.0},
	      'amp_temp':{
		      'type': 'adc_linear_5V', # divider 
		      'name': 'Microchip TC1047',
		      'cb_mV_at_25C': 750.0,
		      'cb_mV_per_C': 10.0,
		      'cb_scale': 1.0,
		      'cb_bias': 0.0,
		      },
	      'motor_temp':{
		      'type': 'adc_linear_3V3', #no divider
		      'name': 'Analog TMP36',
		      'cb_mV_at_25C': 750.0,
		      'cb_mV_per_C': 10.0,
		      'cb_scale': 1.0,
		      'cb_bias': 0.0},
	      'torque':{
		      'type': 'sea_vertx_14bit',
		      'name': 'ContElec VertX13',
		      'cb_inv_torque': [1,0],
		      'cb_torque': [1,0],
		      'cb_scale': 1.0,
		      'cb_bias': 0.0},
	      'current':{
		      'type': 'adc_linear_5V',
		      'name': 'Allegro ACS712-20',
		      'cb_mV_per_A': 100.0,
		      'cb_ticks_at_zero_a': 0.0,
		      'cb_ticks_at_zero_b': 0.0,
		      'cb_scale': 1.0,
		      'cb_bias': 0.0},
	      },
	'param':{
		'max_amp_temp': 100.0,
		'max_current': 12000,
		'max_motor_temp': 145.0,
		'max_tq': 40000.0,
		'min_tq': -40000.0,
		'thetadot_deadband': 1.0
		},
	'param_internal':
	{
		'calib_tq_lc_amp':2000.0,
		'analyze_tq_lc_amp':10000.0,
		'calib_lever_mass':327.6,
		'calib_lever_com':157.5,
		'calib_lever_len':304.8,
		'joint_limits': {'right_arm':[-24.2,150.5],'left_arm':[-150.5,24.2],'note':'positive is elbow to its right'},
		'calib_tq_degree':1,
		'calib_hub_diam':70
	}
}
# ########################################## A2 J2 ##########################################################
config_default_a2_j2={
	'calib':{
		'motor':{
			'name': 'Maxon EC45 Flat 50W 24V',
			'winding_resistance': 1.03,
			'thermal_resistance_housing_ambient': 3.9,
			'thermal_resistance_rotor_housing': 5.3,
			'max_winding_temp': 155,
			'gear_ratio':100,
			'thermal_time_constant_winding': 11.7
			},
	      'theta':{
		      'type': 'vertx_14bit',
		      'name': 'ContElec VertX13',
		      'cb_scale': 1.0,
		      'cb_bias': 0.0},
	      'amp_temp':{
		      'type': 'adc_linear_5V', # divider 
		      'name': 'Microchip TC1047',
		      'cb_mV_at_25C': 750.0,
		      'cb_mV_per_C': 10.0,
		      'cb_scale': 1.0,
		      'cb_bias': 0.0,
		      },
	      'motor_temp':{
		      'type': 'adc_linear_3V3', # no divider 
		      'name': 'Microchip TC1047',
		      'cb_mV_at_25C': 750.0,
		      'cb_mV_per_C': 10.0,
		      'cb_scale': 1.0,
		      'cb_bias': 0.0},
	      'torque':{
		      'type': 'sea_vertx_14bit',
		      'name': 'ContElec VertX13',
		      'cb_inv_torque': [1,0],
		      'cb_torque': [1,0],
		      'cb_scale': 1.0,
		      'cb_bias': 0.0},
	      'current':{
		      'type': 'adc_linear_5V',
		      'name': 'Allegro ACS712-20',
		      'cb_mV_per_A': 100.0,
		      'cb_ticks_at_zero_a': 0.0,
		      'cb_ticks_at_zero_b': 0.0,
		      'cb_scale': 1.0,
		      'cb_bias': 0.0},
	      },
	'param':{
		'max_amp_temp': 100.0,
		'max_current': 6000,
		'max_motor_temp': 115.0,
		'max_tq': 20000.0,
		'min_tq': -20000.0,
		'thetadot_deadband': 1.0
		},
	'param_internal':
	{
		'calib_tq_lc_amp':2000.0,
		'analyze_tq_lc_amp':10000.0,
		'calib_lever_mass':327.6,
		'calib_lever_com':157.5,
		'calib_lever_len':304.8,
		'joint_limits': {'both_arms':[-85,85],'note':'positive is reaching to its right'},
		'calib_tq_degree':1,
		'calib_hub_diam':70
	}
}
# ############################################# A2 J3 #######################################################
config_default_a2_j3={
	'calib':{
		'motor':{
			'name': 'Maxon EC45 Flat 50W 24V',
			'winding_resistance': 1.03,
			'thermal_resistance_housing_ambient': 3.9,
			'thermal_resistance_rotor_housing': 5.3,
			'max_winding_temp': 155,
			'gear_ratio':100,
			'thermal_time_constant_winding': 11.7
			},
	      'theta':{
		      'type': 'vertx_14bit',
		      'name': 'ContElec VertX13',
		      'cb_scale': 1.0,
		      'cb_bias': 0.0},
	      'amp_temp':{
		      'type': 'adc_linear_5V', # divider 
		      'name': 'Microchip TC1047',
		      'cb_mV_at_25C': 750.0,
		      'cb_mV_per_C': 10.0,
		      'cb_scale': 1.0,
		      'cb_bias': 0.0,
		      },
	      'motor_temp':{
		      'type': 'adc_linear_3V3', # no divider 
		      'name': 'Microchip TC1047',
		      'cb_mV_at_25C': 750.0,
		      'cb_mV_per_C': 10.0,
		      'cb_scale': 1.0,
		      'cb_bias': 0.0},
	      'torque':{
		      'type': 'sea_vertx_14bit',
		      'name': 'ContElec VertX13',
		      'cb_inv_torque': [1,0],
		      'cb_torque': [1,0],
		      'cb_scale': 1.0,
		      'cb_bias': 0.0},
	      'current':{
		      'type': 'adc_linear_5V',
		      'name': 'Allegro ACS712-20',
		      'cb_mV_per_A': 100.0,
		      'cb_ticks_at_zero_a': 0.0,
		      'cb_ticks_at_zero_b': 0.0,
		      'cb_scale': 1.0,
		      'cb_bias': 0.0},
	      },
	'param':{
		'max_amp_temp': 100.0,
		'max_current': 6000,
		'max_motor_temp': 115.0,
		'max_tq': 20000.0,
		'min_tq': -20000.0,
		'thetadot_deadband': 1.0
		},
	'param_internal':
	{
		'calib_tq_lc_amp':2000.0,
		'analyze_tq_lc_amp':10000.0,
		'calib_lever_mass':327.6,
		'calib_lever_com':157.5,
		'calib_lever_len':304.8,
		'joint_limits': {'both_arms':[0,130.0],'note':'positive is wrist towards chest'},
		'calib_tq_degree':1,
		'calib_hub_diam':70
	}
}
# ############################################# A2 J4 #######################################################
config_default_a2_j4={
	'calib':{
		'motor':{
			'name': 'Maxon EC32 Flat 15W 24V',
			'winding_resistance': 13.7,
			'thermal_resistance_housing_ambient': 9.3,
			'thermal_resistance_rotor_housing': 5.2,
			'max_winding_temp': 125,
			'gear_ratio': 100.0,
			'thermal_time_constant_winding': 7.7
			},
	      'theta':{
		      'type': 'vertx_14bit',
		      'name': 'ContElec VertX13',
		      'cb_scale': 1.0,
		      'cb_bias': 0.0},
	      'amp_temp':{
		      'type': 'adc_linear_3V3', # no divider 
		      'name': 'Microchip TC1047',
		      'cb_mV_at_25C': 750.0,
		      'cb_mV_per_C': 10.0,
		      'cb_scale': 1.0,
		      'cb_bias': 0.0,
		      },
	      'motor_temp':{
		      'type': 'adc_linear_3V3', # no divider 
		      'name': 'Microchip TC1047',
		      'cb_mV_at_25C': 750.0,
		      'cb_mV_per_C': 10.0,
		      'cb_scale': 1.0,
		      'cb_bias': 0.0},
	      'torque':{
		      'type': 'sea_vertx_14bit',
		      'name': 'ContElec VertX13',
		      'cb_inv_torque': [1,0],
		      'cb_torque': [1,0],
		      'cb_scale': 1.0,
		      'cb_bias': 0.0},
	      'current':{
		      'type': 'adc_linear_5V',
		      'name': 'Allegro ACS712-05',
		      'cb_mV_per_A': 185.0,
		      'cb_ticks_at_zero_a': 0.0,
		      'cb_ticks_at_zero_b': 0.0,
		      'cb_scale': 1.0,
		      'cb_bias': 0.0},
	      },
	'param':{
		'max_amp_temp': 100.0,
		'max_current': 1000,
		'max_motor_temp': 100.0,
		'max_tq': 3000.0,
		'min_tq': -3000.0,
		'thetadot_deadband': 1.0
		},
	'param_internal':
	{
		'calib_tq_lc_amp':1000.0,
		'analyze_tq_lc_amp':1000.0,
		'calib_lever_mass':128.7,
		'calib_lever_com':89.59,
		'calib_lever_len':175.0, #use center hole
		'joint_limits': {'both_arms':[-160.0,160.0],'note':'positive is top of forearm rotating to its right'},
		'calib_tq_degree':1,
		'calib_hub_diam':70
	}
}
# ############################################# A2 J5 #######################################################
config_default_a2_j5={
	'calib':{
		'motor':{
			'name': 'Maxon EC32 Flat 15W 24V',
			'winding_resistance': 13.7,
			'thermal_resistance_housing_ambient': 9.3,
			'thermal_resistance_rotor_housing': 5.2,
			'max_winding_temp': 125,
			'gear_ratio': 100.0,
			'thermal_time_constant_winding': 7.7
			},
	      'theta':{
		      'type': 'vertx_14bit',
		      'name': 'ContElec VertX13',
		      'cb_scale': 1.0,
		      'cb_bias': 0.0},
	      'amp_temp':{
		      'type': 'adc_linear_3V3', # no divider 
		      'name': 'Microchip TC1047',
		      'cb_mV_at_25C': 750.0,
		      'cb_mV_per_C': 10.0,
		      'cb_scale': 1.0,
		      'cb_bias': 0.0,
		      },
	      'motor_temp':{
		      'type': 'adc_linear_3V3', # no divider 
		      'name': 'Microchip TC1047',
		      'cb_mV_at_25C': 750.0,
		      'cb_mV_per_C': 10.0,
		      'cb_scale': 1.0,
		      'cb_bias': 0.0},
	      'torque':{
		      'type': 'sea_vertx_14bit',
		      'name': 'ContElec VertX13',
		      'cb_inv_torque': [1,0],
		      'cb_torque': [1,0],
		      'cb_scale': 1.0,
		      'cb_bias': 0.0},
	      'current':{
		      'type': 'adc_linear_5V',
		      'name': 'Allegro ACS712-05',
		      'cb_mV_per_A': 185.0,
		      'cb_ticks_at_zero_a': 0.0,
		      'cb_ticks_at_zero_b': 0.0,
		      'cb_scale': 1.0,
		      'cb_bias': 0.0},
	      },
	'param':{
		'max_amp_temp': 100.0,
		'max_current': 1000,
		'max_motor_temp': 100.0,
		'max_tq': 3000.0,
		'min_tq': -3000.0,
		'thetadot_deadband': 1.0
		},
	'param_internal':
	{
		'calib_tq_lc_amp':1000.0,
		'analyze_tq_lc_amp':1000.0,
		'calib_lever_mass':128.7,
		'calib_lever_com':85.41,
		'calib_lever_len':187.8, #use furthest hole
		'joint_limits': {'both_arms':[-60,60],'note': 'positive is hand rotating up'},
		'calib_tq_degree':1,
		'calib_hub_diam':70
	}
}
# ############################################# A2 J6 #######################################################
config_default_a2_j6={
	'calib':{
		'motor':{
			'name': 'Maxon EC32 Flat 15W 24V',
			'winding_resistance': 13.7,
			'thermal_resistance_housing_ambient': 9.3,
			'thermal_resistance_rotor_housing': 5.2,
			'max_winding_temp': 125,
			'gear_ratio': 100.0,
			'thermal_time_constant_winding': 7.7
			},
	      'theta':{
		      'type': 'vertx_14bit',
		      'name': 'ContElec VertX13',
		      'cb_scale': 1.0,
		      'cb_bias': 0.0},
	      'amp_temp':{
		      'type': 'adc_linear_3V3', # no divider 
		      'name': 'Microchip TC1047',
		      'cb_mV_at_25C': 750.0,
		      'cb_mV_per_C': 10.0,
		      'cb_scale': 1.0,
		      'cb_bias': 0.0,
		      },
	      'motor_temp':{
		      'type': 'adc_linear_3V3', # no divider 
		      'name': 'Microchip TC1047',
		      'cb_mV_at_25C': 750.0,
		      'cb_mV_per_C': 10.0,
		      'cb_scale': 1.0,
		      'cb_bias': 0.0},
	      'torque':{
		      'type': 'sea_vertx_14bit',
		      'name': 'ContElec VertX13',
		      'cb_inv_torque': [1,0],
		      'cb_torque': [1,0],
		      'cb_scale': 1.0,
		      'cb_bias': 0.0},
	      'current':{
		      'type': 'adc_linear_5V',
		      'name': 'Allegro ACS712-05',
		      'cb_mV_per_A': 185.0,
		      'cb_ticks_at_zero_a': 0.0,
		      'cb_ticks_at_zero_b': 0.0,
		      'cb_scale': 1.0,
		      'cb_bias': 0.0},
	      },
	'param':{
		'max_amp_temp': 100.0,
		'max_current': 1000,
		'max_motor_temp': 100.0,
		'max_tq': 3000.0,
		'min_tq': -3000.0,
		'thetadot_deadband': 1.0
		},
	'param_internal':
	{
		'calib_tq_lc_amp':1000.0,
		'analyze_tq_lc_amp':1000.0,
		'calib_lever_mass':128.7,
		'calib_lever_com':85.41,
		'calib_lever_len':187.8, #use furthest hole
		'joint_limits': {'both_arms':[-60,60],'note':'positive is fingers rotating to its right'},
		'calib_tq_degree':1,
		'calib_hub_diam':70
	}
}

# ###########################################################################

class M3Calibrate_Arm_A2R1(M3CalibrateActuatorEcR1):
	def __init__(self):
		M3CalibrateActuatorEcR1.__init__(self)
		
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
		if not M3CalibrateActuatorEcR1.start(self,ctype):
			return False
		self.jid=int(self.comp_ec.name[self.comp_ec.name.find('_j')+2:])
		self.calib_default=self.config_default[self.jid]['calib']
		self.param_default=self.config_default[self.jid]['param']
		self.param_internal=self.config_default[self.jid]['param_internal']
		print 'Calibrating joint',self.joint_names[self.jid]
		return True
	
		

