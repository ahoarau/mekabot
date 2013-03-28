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


config_default_h2_j0={
	'calib':{
		'motor':{
			'name': 'Maxon EC13 6W 24V',
			'winding_resistance': 24.6,
			'thermal_resistance_housing_ambient': 32.0,
			'thermal_resistance_rotor_housing': 2.46,
			'max_winding_temp': 155,
			'gear_ratio': 131.0,
			'thermal_time_constant_winding': 0.72
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
		'max_current': 1000,
		'max_motor_temp': 75.0,
		'max_tq': 150.0,
		'min_tq': -30.0,
		'thetadot_deadband': 1.0
		},
	'param_internal':
	{
		'calib_tq_degree':1,
		'pwm_theta':[-200,200],
		'calib_hub_diam':180.0,
		'joint_limits':[0,135.0]
	}
}

config_default_h2_j1={
	'calib':{
		'motor':{
			'name': 'Maxon EC13 12W 24V',
			'winding_resistance': 8.93,
			'thermal_resistance_housing_ambient': 23.9,
			'thermal_resistance_rotor_housing': 1.26,
			'max_winding_temp': 155,
			'gear_ratio': 131.0,
			'thermal_time_constant_winding': 0.6
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
		'max_current': 1000,
		'max_motor_temp': 75.0,
		'max_tq': 150.0,
		'min_tq': -30.0,
		'thetadot_deadband': 1.0
		},
	'param_internal':
	{
		'calib_tq_degree':1,
		'pwm_theta':[-200,200],
		'calib_hub_diam':15.5,
		'joint_limits':[0,330.0]
	}
}

config_default_h2_j2={
	'calib':{
		'motor':{
			'name': 'Maxon EC13 12W 24V',
			'winding_resistance': 8.93,
			'thermal_resistance_housing_ambient': 23.9,
			'thermal_resistance_rotor_housing': 1.26,
			'max_winding_temp': 155,
			'gear_ratio': 131.0,
			'thermal_time_constant_winding': 0.6
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
		'max_current': 1000,
		'max_motor_temp': 75.0,
		'max_tq': 150.0,
		'min_tq': -30.0,
		'thetadot_deadband': 1.0
		},
	'param_internal':
	{
		'calib_tq_degree':1,
		'pwm_theta':[-200,200],
		'calib_hub_diam':15.5,
		'joint_limits':[0,330.0]
	}
}

config_default_h2_j3={
	'calib':{
		'motor':{
			'name': 'Maxon EC13 12W 24V',
			'winding_resistance': 8.93,
			'thermal_resistance_housing_ambient': 23.9,
			'thermal_resistance_rotor_housing': 1.26,
			'max_winding_temp': 155,
			'gear_ratio': 131.0,
			'thermal_time_constant_winding': 0.6
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
		'max_current': 1000,
		'max_motor_temp': 75.0,
		'max_tq': 150.0,
		'min_tq': -30.0,
		'thetadot_deadband': 1.0
		},
	'param_internal':
	{
		'calib_tq_degree':1,
		'pwm_theta':[-200,200],
		'calib_hub_diam':15.5,
		'joint_limits':[0,330.0]
	}
}

config_default_h2_j4={
	'calib':{
		'motor':{
			'name': 'Maxon EC13 12W 24V',
			'winding_resistance': 8.93,
			'thermal_resistance_housing_ambient': 23.9,
			'thermal_resistance_rotor_housing': 1.26,
			'max_winding_temp': 155,
			'gear_ratio': 131.0,
			'thermal_time_constant_winding': 0.6
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
		'max_current': 1000,
		'max_motor_temp': 75.0,
		'max_tq': 150.0,
		'min_tq': -30.0,
		'thetadot_deadband': 1.0
		},
	'param_internal':
	{
		'calib_tq_degree':1,
		'pwm_theta':[-200,200],
		'calib_hub_diam':15.5,
		'joint_limits':[0,330.0]
	}
}


#self.param={'adc_tq_amp':100,
	    #
	    #'r_capstan':0,'r_loadcell':0}
		
class M3Calibrate_Hand_H2R1(M3CalibrateActuatorEcR1):
	def __init__(self):
		M3CalibrateActuatorEcR1.__init__(self)

		self.joint_names=['Thumb Adduction J0',
				  'Ring Finger J1',
				  'Index Finger J2',
				  'Middle Finger J3',
				  'Thumb J4']
		
		self.config_default=[
			config_default_h2_j0,
			config_default_h2_j1,
			config_default_h2_j2,
			config_default_h2_j3,
			config_default_h2_j4]
		
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
			self.calibrate_torque_hub(run_neg=False)
			self.write_config()
			return True
		if ct=='tt':
			self.reset_sensor('theta')
			if self.jid==0:
				M3CalibrateActuatorEcR1.calibrate_theta(self)
			else:
				self.calibrate_theta()
			self.write_config()
			return True
		if ct=='cs':
			self.reset_sensor('torque')
			self.calibrate_torque_servo_theta()
			self.write_config()
			return True
		if M3CalibrateActuatorEcR1.do_task(self,ct):
			return True
		return False
	
	def print_tasks(self):
		M3CalibrateActuatorEcR1.print_tasks(self)
		print 'cs: calibrate torque servo theta' 
		
	def calibrate_torque_servo_theta(self):
		print 'This routine requires M3Joint Theta mode control of the hub'
		print 'Proceed [y]?'
		if not m3t.get_yes_no('y'):
			return
		print 'Enter pose theta [90]'
		pose=m3t.get_float(90)
		print 'Number of calibration weights [2]?'
		nw=m3t.get_int(2)
		print 'Calibration hub diameter (mm)[',self.param_internal['calib_hub_diam'],']?'
		ch=m3t.get_float(self.param_internal['calib_hub_diam'])/2.0
		
		print 'Place actuator in unloaded configuration. Hit enter when ready'
		raw_input()
		self.step()
		adc_zero=float(self.get_sensor_list_avg(['adc_torque'],1.0)['adc_torque'])
		print 'adc_zero',adc_zero
		adc_act=[adc_zero]
		load_mNm=[0.0]
		
		print 'Turn on power. Hit enter when ready'
		raw_input()
		self.proxy.subscribe_status(self.comp_j)
		self.proxy.publish_command(self.comp_j)
		self.proxy.publish_param(self.comp_j)
		self.proxy.make_operational(self.name_j)
		self.proxy.make_operational(self.name_rt)
		self.comp_j.set_mode_theta()
		self.comp_j.set_theta_deg(pose)
		self.comp_j.set_slew_rate(25.0)
		self.step(3.0)
		
		for i in range(nw):
			print 'Place positive load ',i,'. Hit enter when ready'
			raw_input()
			self.step()
			adc_act.append(float(self.get_sensor_list_avg(['adc_torque'],1.0)['adc_torque']))
			print 'adc_act',adc_act[-1]
			print 'Enter load weight (g)'
			w=m3u.g2mN(m3t.get_float())*(ch/1000.0)
			load_mNm.append(w)
			print 'Calibration load (mNm)',load_mNm[-1]
		print adc_act
		print load_mNm
		print 'Enter degree [',self.param_internal['calib_tq_degree'],']?'
		n=m3t.get_int(self.param_internal['calib_tq_degree'])
		poly,inv_poly=self.get_polyfit_to_data(adc_act,load_mNm,n=n)
		self.comp_rt.config['calib']['torque']['cb_torque']=list(poly)
		self.comp_rt.config['calib']['torque']['cb_inv_torque']=list(inv_poly)
		self.comp_j.set_mode_off()
		self.step()
		self.proxy.make_safe_operational(self.name_j)
		self.proxy.make_safe_operational(self.name_rt)
		self.step()
		self.write_raw_calibration({'log_adc_torque':adc_act,'log_load_mNm':load_mNm,
					    'cb_torque':poly,'cb_inv_torque':inv_poly})
		
	def calibrate_theta(self):
		print 'The actuator should not be hooked up to anything. Proceed [y]?'
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
		
		theta_as=self.theta.raw_2_deg(self.comp_rt.config['calib']['theta'],q_on_a,q_p_a,q_r_a)
		theta_bs=self.theta.raw_2_deg(self.comp_rt.config['calib']['theta'],q_on_b,q_p_b,q_r_b)
		
		print 'Did this last motion unspool the cable [y]?' #At zero position
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
		
	#def calibrate_torque(self):
		#print 'This calibration routine assumes that the adc_torque PID has been tuned. Proceed [y]?'
		#if not m3t.get_yes_no('y'):
			#return
		#print 'This calibration routine requires the actuator tendon to torque a load-cell through a hub'
		#print 'Enter amplitude [',self.param_internal['adc_tq_amp'],']'
		#amp=m3t.get_float(self.param_internal['adc_tq_amp'])
		#print 'Enter load cell hub radius (m) [',self.param['r_loadcell'],']'
		#rl=m3t.get_float(self.param_internal['r_loadcell'])
		#print 'Enter actuator capstan radius (m) [',self.param['r_capstan'],']'
		#rc=m3t.get_float(self.param_internal['r_capstan'])
		#print 'Place actuator in unloaded configuration. Hit enter when ready'
		#raw_input()
		#self.step()
		#adc_zero=float(self.get_sensor_list_avg(['adc_torque'],1.0)['adc_torque'])
		#load_zero=(rc/rl)*self.comp_l.get_torque_mNm()
		
		#ns=100.0
		#t=nu.arange(0,1+1/ns,1/ns)
		#self.comp_ec.command.mode=aec.ACTUATOR_EC_MODE_TORQUE
		#log_adc_torque=[]
		#log_load_mNm=[]
		#for idx in range(len(t)):
			#des=amp*t[idx]+adc_zero
			#self.comp_ec.command.t_desire=int(des)
			#self.step()
			#lmNm=(rc/rl)*self.comp_l.get_torque_mNm()-load_zero
			#tq=self.comp_ec.status.adc_torque
			#print '------------------------'
			#print 'Des',idx,'/',ns,': ',t[idx]
			#print 'Adc SEA',tq
			#print 'mNm Load Cell',lmNm
			#log_adc_torque.append(tq)
			#log_load_mNm.append(lmNm)
		#self.comp_ec.command.mode=aec.ACTUATOR_EC_MODE_OFF
		#self.step()
		#print 'Poly degree [',self.param_default['calib_tq_degree'],']?'
		#n=m3t.get_int(self.param_default['calib_tq_degree'])
		#poly,inv_poly=self.get_polyfit_to_data(x=log_adc_torque,y=log_load_mNm,n=n)
		#print 'Save raw data [y]?'
		#if m3t.get_yes_no('y'):
			#self.write_raw_calibration({'log_adc_torque':log_adc_torque,'log_load_mNm':log_load_mNm,
					    #'cb_torque':poly,'cb_inv_torque':inv_poly})
		#self.comp_rt.config['calib']['cb_torque']=poly
		#self.comp_rt.config['calib']['cb_inv_torque']=inv_poly
		#print 'Poly',poly
	
		

