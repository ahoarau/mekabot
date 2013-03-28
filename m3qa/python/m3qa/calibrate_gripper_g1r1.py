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

# ####################################################################################################


config_default_g1_j0={
	'calib':{
		'motor':{
			'name': 'Maxon RE13 2.5W 24V',
			'winding_resistance': 53.2,#Ohm
			'winding_inductance':1.79,#mH
			'torque_constant':19.7, #mNm/A
			'thermal_resistance_housing_ambient': 33.0,#K/W
			'thermal_resistance_rotor_housing': 7.0,#K/W
			'max_winding_temp': 85, #C
			'gear_ratio': 275.0,
			'thermal_time_constant_winding': 4.85, #S
			'thermal_time_constant_motor':346, #S
			'temp_sensor_type':'housing'
			},
	      'theta':{
		      'type': 'ma3_12bit',
		      'name': 'US Digital MA3',
		      'cb_scale': 1.0,
		      'cb_bias': 0.0},
	      'amp_temp':{
		      'type': 'adc_linear_3V3', #3V3 supply, no divider
		      'name': 'Microchip TC1047',
		      'cb_mV_at_25C': 750.0,
		      'cb_mV_per_C': 10.0,
		      'cb_scale': 1.0,
		      'cb_bias': 0.0,
		      },
	      'motor_temp':{
		      'type': 'adc_linear_3V3', #5V supply, no divider
		      'name': 'Analog TMP36',
		      'cb_mV_at_25C': 750.0,
		      'cb_mV_per_C': 10.0,
		      'cb_scale': 1.0,
		      'cb_bias': 0.0},
	      'torque':{
		      'type': 'adc_poly',
		      'name': 'Allegro A1321',
		      'cb_inv_torque': [1,0],
		      'cb_torque': [1,0],
		      'cb_scale': 1.0,
		      'cb_bias': 0.0},
	      'current':{
		      'type': 'none',
		      'cb_scale': 0.0,
		      'cb_bias': 0.0},
	      },
	'param':{
		'max_amp_temp': 100.0,
		'max_current': 800,
		'max_motor_temp': 75.0,
		'max_tq': 150.0,
		'min_tq': -30.0,
		'thetadot_deadband': 1.0
		},
	'param_internal':
	{
		'calib_tq_degree':1,
		'pwm_theta':[-800,800],
		'pwm_torque':[-1000,-1000],
		'joint_limits':[0,315.0]
	}
}

config_default_g1_j1={
	'calib':{
		'motor':{
			'name': 'Maxon RE13 2.5W 24V',
			'winding_resistance': 53.2,#Ohm
			'winding_inductance':1.79,#mH
			'torque_constant':19.7, #mNm/A
			'thermal_resistance_housing_ambient': 33.0,#K/W
			'thermal_resistance_rotor_housing': 7.0,#K/W
			'max_winding_temp': 85, #C
			'gear_ratio': 275.0,
			'thermal_time_constant_winding': 4.85, #S
			'thermal_time_constant_motor':346, #S
			'temp_sensor_type':'housing'
			},
	      'theta':{
		      'type': 'ma3_12bit',
		      'name': 'US Digital MA3',
		      'cb_scale': 1.0,
		      'cb_bias': 0.0},
	      'amp_temp':{
		      'type': 'adc_linear_3V3', #3V3 supply, no divider
		      'name': 'Microchip TC1047',
		      'cb_mV_at_25C': 750.0,
		      'cb_mV_per_C': 10.0,
		      'cb_scale': 1.0,
		      'cb_bias': 0.0,
		      },
	      'motor_temp':{
		      'type': 'adc_linear_3V3', #5V supply, no divider=3V3 
		      'name': 'Analog TMP36',
		      'cb_mV_at_25C': 750.0,
		      'cb_mV_per_C': 10.0,
		      'cb_scale': 1.0,
		      'cb_bias': 0.0},
	      'torque':{
		      'type': 'adc_poly',
		      'name': 'Allegro A1321',
		      'cb_inv_torque': [1,0],
		      'cb_torque': [1,0],
		      'cb_scale': 1.0,
		      'cb_bias': 0.0},
	      'current':{
		      'type': 'none',
		      'cb_scale': 0.0,
		      'cb_bias': 0.0},
	      },
	'param':{
		'max_amp_temp': 100.0,
		'max_current': 800,
		'max_motor_temp': 75.0,
		'max_tq': 150.0,
		'min_tq': -30.0,
		'thetadot_deadband': 1.0
		},
	'param_internal':
	{
		'calib_tq_degree':1,
		'pwm_theta':[-800,800],
		'pwm_torque':[-1000,-1000],
		'joint_limits':[0,315.0]
	}
}


		
class M3Calibrate_Gripper_G1R1(M3CalibrateActuatorEcR1):
	def __init__(self):
		M3CalibrateActuatorEcR1.__init__(self)
		self.joint_names=['Left Digit J0',
				  'Right Digit J1']
		self.config_default=[
			config_default_g1_j0,
			config_default_g1_j1]
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
		if ct=='ch':
			self.reset_sensor('torque')
			self.calibrate_torque()
			self.write_config()
			return True
		if ct=='tt':
			self.reset_sensor('theta')
			self.calibrate_theta()
			self.write_config()
			return True
		if M3CalibrateActuatorEc.do_task(self,ct):
			return True
		return False
	
	def print_tasks(self):
		M3CalibrateActuatorEcR1.print_tasks(self)
		print 'ch: calibrate torque' 
		print 'tt: calibrate theta' 
		
	def display_sensors(self):
		M3CalibrateActuatorEcR1.display_sensors(self)
		q_on=self.comp_ec.status.qei_on
		q_p=self.comp_ec.status.qei_period
		q_r=self.comp_ec.status.qei_rollover
		c=self.theta.raw_2_deg(self.comp_rt.config['calib']['theta'],q_on,q_p,q_r)
		pos=1000.0*math.pi*2*self.comp_j.config['calib']['cb_drive_radius_m']*c/360.0
		print 'Pos:    (mm) : '+'%3.3f'%pos+' Qei On '+'%d'%q_on+' Qei Period '+'%d'%q_p+' Qei Rollover '+'%d'%q_r
		raw=self.comp_ec.status.adc_torque
		c=self.torque.raw_2_mNm(self.comp_rt.config['calib']['torque'],raw)
		mN=c/self.comp_j.config['calib']['cb_drive_radius_m']
		print 'Force:   (g) : '+'%3.2f'%m3u.mN2g(mN)+' (mN): '+'%3.2f'%mN+' (ADC) '+'%d'%raw
		
	def calibrate_torque(self):
		self.proxy.publish_command(self.comp_rt)
		self.proxy.publish_param(self.comp_rt)
		self.proxy.make_operational(self.name_rt)
		self.step()
		print 'Make sure other digit is all the way open'
		print 'Place digit in zero load condition'
		print 'Hit enter when ready'
		raw_input()
		self.step()
		raw_a=int(self.get_sensor_list_avg(['adc_torque'],1.0)['adc_torque'])
		load_a=0
		
		print 'Hang 1Kg weight from gripper near slider'
		print 'Hit enter to move joint in first direction.'
		raw_input()
		self.comp_rt.set_mode_pwm()
		print 'Desired pwm? [',self.param_internal['pwm_torque'][0],']?'
		p=int(m3t.get_float(self.param_internal['pwm_theta'][0]))
		self.comp_rt.set_pwm(p)
		self.step()
		print 'Hit any key when ready to sample'
		raw_input()
		raw_b=int(self.get_sensor_list_avg(['adc_torque'],1.0)['adc_torque'])
		print 'Was load in the opening direction [y]?'
		if m3t.get_yes_no('y'):
			load_b=m3u.g2mN(1000.0)*self.comp_j.config['calib']['cb_drive_radius_m']
		else:
			load_b=m3u.g2mN(-1000.0)*self.comp_j.config['calib']['cb_drive_radius_m']
			
		print 'Hit enter to move joint in second direction.'
		raw_input()
		self.comp_rt.set_mode_pwm()
		print 'Desired pwm? [',self.param_internal['pwm_torque'][1],']?'
		p=int(m3t.get_float(self.param_internal['pwm_theta'][1]))
		self.comp_rt.set_pwm(p)
		self.step()
		print 'Hit any key when ready to sample'
		raw_input()
		raw_c=int(self.get_sensor_list_avg(['adc_torque'],1.0)['adc_torque'])
		load_c=-1*load_b

		log_adc_torque=[raw_a,raw_b,raw_c]
		log_load_mNm=[load_a,load_b,load_c]
		poly,inv_poly=self.get_polyfit_to_data(x=log_adc_torque,y=log_load_mNm,n=1)
		self.write_raw_calibration({'log_adc_torque':log_adc_torque,'log_load_mNm':log_load_mNm,
					    'cb_torque':poly,'cb_inv_torque':inv_poly})
		self.comp_rt.config['calib']['torque']['cb_torque']=poly
		self.comp_rt.config['calib']['torque']['cb_inv_torque']=inv_poly
		print 'Poly',poly
		s=m3tc.PolyEval(poly,[raw_a,raw_b,raw_c])
		m3t.mplot2(range(len(log_adc_torque)),log_load_mNm,s,xlabel='Samples',ylabel='Torque (mNm)',
			   y1name='load',y2name='raw')
		
			
	def calibrate_theta(self):
		pconfig=self.comp_ec.param.config #disable qei limits
		self.comp_ec.param.config=0 
		self.proxy.publish_command(self.comp_rt)
		self.proxy.publish_param(self.comp_rt)
		self.proxy.make_operational(self.name_rt)
		self.step()
		print 'Make sure other digit is all the way open'
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
		print 'RawA',q_on_a
			
		print 'Moving joint to second limit. Hit any key when ready'
		raw_input()
		self.comp_rt.set_mode_pwm()
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
		print 'Rawb',q_on_b
		
		theta_as=self.theta.raw_2_deg(self.comp_rt.config['calib']['theta'],q_on_a,q_p_a,q_r_a)
		theta_bs=self.theta.raw_2_deg(self.comp_rt.config['calib']['theta'],q_on_b,q_p_b,q_r_b)
		
		print 'Did this last motion open the gripper [y]?' #At zero position
		if m3t.get_yes_no('y'):
			theta_b=0
			theta_a=abs(theta_bs-theta_as)
		else:
			theta_a=0
			theta_b=abs(theta_bs-theta_as)
			
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
			print 'Expected joint limits of',self.param_internal['joint_limits']
			self.comp_j.param.max_q=float(max_q) 
			self.comp_j.param.min_q=float(min_q)
		else:
			print 'Joint component missing. Unable to set joint limits to',min_q,max_q
			
		#Assume 0-Ndeg, where N is defined by the encoder soft limits
		self.comp_ec.config['param']['qei_min']=min(q_on_a,q_on_b)+100
		self.comp_ec.config['param']['qei_max']=max(q_on_a,q_on_b)-100
		self.comp_ec.param.qei_min=min(q_on_a,q_on_b)+100
		self.comp_ec.param.qei_max=max(q_on_a,q_on_b)-100
		print 'Setting DSP qei min/max to',self.comp_ec.config['param']['qei_min'],self.comp_ec.config['param']['qei_max']
		

