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
from m3qa.calibrate_actuator_ec_r2 import *
import m3.actuator_ec_pb2 as aec
import m3qa.config_torso_t2r2 as t2r2

# ######################################## T2 J0 ############################################################
config_default_t2_j0={
	'calib':t2r2.config_torso_t2r2_actuator_j0['calib'],
	'param':t2r2.config_torso_t2r2_actuator_j0['param'],
	'param_internal':
	{
		'calib_tq_lc_amp':2000.0,
		'analyze_tq_lc_amp':10000.0,
		'calib_lever_mass':327.6,
		'calib_lever_com':157.5,
		'calib_lever_len':304.8,
		'joint_limits': [-90.0*60.0/45.0,90.0*60.0/45.0], #60:45 belt on encoder
		'calib_tq_degree':1,
		'calib_hub_diam':70,
		'pwm_theta':[-700,700],
	}
}
# ###################################### T2 J1 ##############################################################
config_default_t2_j1={
	'calib':t2r2.config_torso_t2r2_actuator_j1['calib'],
	'param':t2r2.config_torso_t2r2_actuator_j1['param'],
	'param_internal':
	{
		'calib_tq_lc_amp':2000.0,
		'analyze_tq_lc_amp':10000.0,
		'calib_lever_mass':327.6,
		'calib_lever_com':157.5,
		'calib_lever_len':304.8,
		'joint_limits': [-3.5,45.0], #multiple setting possible, [-15.0/2, 67.5/2.0], [-7.5/2, 90.0/2],...
		'pwm_theta':[-150,250],
		'calib_tq_degree':1,
		'calib_hub_diam':70
	}
}


# ###########################################################################

class M3Calibrate_Torso_T2R2(M3CalibrateActuatorEcR2):
	def __init__(self):
		M3CalibrateActuatorEcR2.__init__(self)
		
		self.joint_names=['Pan J0',
				  'Pitch J1']
		self.config_default=[
			config_default_t2_j0,
			config_default_t2_j1]
		
	def start(self,ctype):
		if not M3CalibrateActuatorEcR2.start(self,ctype):
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
			self.calibrate_theta(use_pwm=True,use_brake=True)
			self.write_config()
			return True
		if ct=='cw':
			self.reset_sensor('torque')
			self.calibrate_torque()
			self.write_config()
			return True
		if M3CalibrateActuatorEcR2.do_task(self,ct):
			return True
		return False
	
	def print_tasks(self):
		M3CalibrateActuatorEcR2.print_tasks(self)

		
	def calibrate_torque(self):
		print 'Procedure for calibrating T2.R2 loadcells. See QA sheet'
		print 'Place load cell in unloaded configuration. Hit enter when ready'
		raw_input()
		self.step()
		adc_zero=float(self.get_sensor_list_avg(['adc_torque'],1.0)['adc_torque'])
		print 'adc_zero',adc_zero
		print 'Number of calibration weights [2]?'
		nw=m3t.get_int(2)
		adc_act=[adc_zero]
		load_mNm=[0.0]
		for i in range(nw):
			print 'Place positive load ',i,'. Hit enter when ready'
			raw_input()
			self.step()
			adc_act.append(float(self.get_sensor_list_avg(['adc_torque'],1.0)['adc_torque']))
			print 'adc_act positive',adc_act[-1]
			print 'Enter load torque (in-lb)'
			w=m3u.inLb2mNm(m3t.get_float())
			load_mNm.append(w)
		for i in range(nw):
			print 'Place negative load ',i,'. Hit enter when ready'
			raw_input()
			self.step()
			adc_act.append(float(self.get_sensor_list_avg(['adc_torque'],1.0)['adc_torque']))
			print 'adc_act positive',adc_act[-1]
			print 'Enter load torque (in-lb)'
			w=m3u.inLb2mNm(m3t.get_float())
			load_mNm.append(w)
		print 'Raw',adc_act
		print 'Given',load_mNm
		poly,inv_poly=self.get_polyfit_to_data(adc_act,load_mNm,n=1)
		self.comp_rt.config['calib']['torque']['cb_torque']=list(poly)
		self.comp_rt.config['calib']['torque']['cb_inv_torque']=list(inv_poly)
		self.write_raw_calibration({'log_adc_torque':adc_act,'log_load_mNm':load_mNm,
					    'cb_torque':poly,'cb_inv_torque':inv_poly})


			
		
