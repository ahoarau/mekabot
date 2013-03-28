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


# ######################################## T2 J0 ############################################################
config_default_t2_j0={
	'calib':{
		'motor':{
			'name': 'Maxon RE40 150W 24V',
			'winding_resistance': .316,
			'thermal_resistance_housing_ambient': 4.7,
			'thermal_resistance_rotor_housing': 1.9,
			'max_winding_temp': 155,
			'gear_ratio': 240.0,
			'thermal_time_constant_winding': 41.0
			},
	      'theta':{
		      'type': 'vertx_14bit',
		      'name': 'ContElec VertX13',
		      'cb_scale': 1.0,
		      'cb_bias': 0.0},
	      'amp_temp':{
		      'type': 'adc_linear_5V', 
		      'name': 'Microchip TC1047',
		      'cb_mV_at_25C': 750.0,
		      'cb_mV_per_C': 10.0,
		      'cb_scale': 1.0,
		      'cb_bias': 0.0,
		      },
	      'motor_temp':{
		      'type': 'adc_linear_3V3', 
		      'name': 'Analog TMP36',
		      'cb_mV_at_25C': 750.0,
		      'cb_mV_per_C': 10.0,
		      'cb_scale': 1.0,
		      'cb_bias': 0.0},
	      'torque':{
		      'type': 'adc_poly',
		      'name': 'Linear torque-load cell',
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
		'joint_limits': [-180.0,180.0],
		'calib_tq_degree':1,
		'calib_hub_diam':70,
		'pwm_theta':[-700,700],
	}
}
# ###################################### T2 J1 ##############################################################
config_default_t2_j1={
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
		      'type': 'adc_linear_5V', #5V supply, divider
		      'name': 'Microchip TC1047',
		      'cb_mV_at_25C': 750.0,
		      'cb_mV_per_C': 10.0,
		      'cb_scale': 1.0,
		      'cb_bias': 0.0,
		      },
	      'motor_temp':{
		      'type': 'adc_linear_3V3', #3v3 supply, no divider 
		      'name': 'Analog TMP36',
		      'cb_mV_at_25C': 750.0,
		      'cb_mV_per_C': 10.0,
		      'cb_scale': 1.0,
		      'cb_bias': 0.0},
	      'torque':{
		      'type': 'adc_poly',
		      'name': 'Linear torque-load cell',
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
		'joint_limits': [-29.0,29.0],
		'pwm_theta':[-450,450],
		'calib_tq_degree':1,
		'calib_hub_diam':70
	}
}


# ###########################################################################

class M3Calibrate_Torso_T2R1(M3CalibrateActuatorEcR1):
	def __init__(self):
		M3CalibrateActuatorEcR1.__init__(self)
		
		self.joint_names=['Pan J0',
				  'Pitch J1']
		self.config_default=[
			config_default_t2_j0,
			config_default_t2_j1]
		
	def start(self,ctype):
		if not M3CalibrateActuatorEcR1.start(self,ctype):
			return False
		self.jid=int(self.comp_ec.name[self.comp_ec.name.find('_j')+2:])
		self.calib_default=self.config_default[self.jid]['calib']
		self.param_default=self.config_default[self.jid]['param']
		self.param_internal=self.config_default[self.jid]['param_internal']
		print 'Calibrating joint',self.joint_names[self.jid]
		return True
	
	def do_task(self,ct):
		if ct=='tt':
			self.reset_sensor('theta')
			self.calibrate_theta()
			self.write_config()
			return True
		if M3CalibrateActuatorEcR1.do_task(self,ct):
			return True
		return False
	
	def print_tasks(self):
		M3CalibrateActuatorEcR1.print_tasks(self)

		
	def calibrate_theta(self):
		print 'Torso will be driven to limits. Proceed [y]?'
		if not m3t.get_yes_no('y'):
			return 
		pconfig=self.comp_ec.param.config #disable qei limits
		self.comp_ec.param.config=0 
		self.proxy.publish_command(self.comp_rt)
		self.proxy.publish_param(self.comp_rt)
		self.proxy.make_operational(self.name_rt)
		self.step()
		
		print 'Moving joint to first limit. Hit any key when ready'
		raw_input()
		self.comp_rt.set_mode_pwm()
		print 'Desired pwm? [',self.param_internal['pwm_theta'][0],']?'
		p=int(m3t.get_float(self.param_internal['pwm_theta'][0]))
		self.comp_rt.set_pwm(p)
		self.step()
		print 'Hit any key when motion done'
		raw_input()
		self.step()
		q_on_a=self.comp_ec.status.qei_on
		q_p_a=self.comp_ec.status.qei_period
		q_r_a=self.comp_ec.status.qei_rollover
		
		print 'Expected joint limits: ',self.param_internal['joint_limits']
		print 'Enter theta (Deg)'
		theta_a=m3t.get_float()
		
		print 'Moving joint to second limit. Hit any key when ready'
		raw_input()
		print 'Desired pwm? [',self.param_internal['pwm_theta'][1],']?'
		p=int(m3t.get_float(self.param_internal['pwm_theta'][1]))
		self.comp_rt.set_pwm(p)
		self.step()
		print 'Hit any key when motion done'
		raw_input()
		self.step()
		q_on_b=self.comp_ec.status.qei_on
		q_p_b=self.comp_ec.status.qei_period
		q_r_b=self.comp_ec.status.qei_rollover
		
		print 'Expected joint limits: ',self.param_internal['joint_limits']
		print 'Enter theta (Deg)'
		theta_b=m3t.get_float()
		
		theta_as=self.theta.raw_2_deg(self.comp_rt.config['calib']['theta'],q_on_a,q_p_a,q_r_a)
		theta_bs=self.theta.raw_2_deg(self.comp_rt.config['calib']['theta'],q_on_b,q_p_b,q_r_b)
		
		self.comp_rt.set_mode_off()
		self.comp_ec.param.config=pconfig #enable qei limits
		self.step()
		self.proxy.make_safe_operational(self.name_rt)
		self.step()
			
		print 'Raw',[theta_as,theta_bs]
		print 'True',[theta_a,theta_b]
		poly,inv_poly=self.get_polyfit_to_data([theta_as,theta_bs],[theta_a,theta_b],n=1)
		
		self.comp_rt.config['calib']['theta']['cb_scale']=poly[0]
		self.comp_rt.config['calib']['theta']['cb_bias']=poly[1]
		
		theta_as=self.theta.raw_2_deg(self.comp_rt.config['calib']['theta'],q_on_a,q_p_a,q_r_a)
		theta_bs=self.theta.raw_2_deg(self.comp_rt.config['calib']['theta'],q_on_b,q_p_b,q_r_b)
		print 'New calibrated range',theta_as,theta_bs
		
		max_q=max(theta_as,theta_bs)
		min_q=min(theta_as,theta_bs)
		if self.comp_j is not None:
			print 'Setting joint limits to',min_q,max_q
			print 'Expected joint limits: ',self.param_internal['joint_limits']
			self.comp_j.param.max_q=float(max_q) 
			self.comp_j.param.min_q=float(min_q)
		else:
			print 'Joint component missing. Unable to set joint limits to',min_q,max_q
			

