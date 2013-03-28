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
import m3qa.config_hand_h2r2 as h2r2
# ####################################################################################################


config_default_h2_j0={
	'calib':h2r2.config_hand_h2r2_actuator_j0['calib'],
	'param':h2r2.config_hand_h2r2_actuator_j0['param'],
	'param_internal':
	{
		'calib_tq_degree':1,
		'pwm_theta':[-200,200],
		'joint_limits':[0,135.0],
		'tq_ext_amp':650.0, #ticks #-1*X for left hand
		'tq_ext_ks': 8.11, #Pounds-per-inch (spec 6.5, measured emperically)
		'tq_ext_windup':-100.0, #ticks
		'tq_ext_deadband':10.0, #deg
		'r_capstan':7.5 #mm
	}
}

config_default_h2_j1={
	'calib':h2r2.config_hand_h2r2_actuator_j1['calib'],
	'param':h2r2.config_hand_h2r2_actuator_j1['param'],
	'param_internal':
	{
		'calib_tq_degree':1,
		'pwm_theta':[-600,600],
		'joint_limits':[0,330.0],
		'tq_ext_amp':650.0, #ticks#-1*X for left hand
		'tq_ext_ks': 8.11, #Pounds-per-inch (spec 6.5, measured emperically)
		'tq_ext_windup':-100.0, #ticks
		'tq_ext_deadband':10.0, #deg
		'r_capstan':7.75 #mm
	}
}


config_default_h2_j2={
	'calib':h2r2.config_hand_h2r2_actuator_j2['calib'],
	'param':h2r2.config_hand_h2r2_actuator_j2['param'],
	'param_internal':
	{
		'calib_tq_degree':1,
		'pwm_theta':[-600,600],
		'joint_limits':[0,330.0],
		'tq_ext_amp':650.0, #ticks#-1*X for left hand
		'tq_ext_ks': 8.11, #Pounds-per-inch (spec 6.5, measured emperically)
		'tq_ext_windup':-100.0, #ticks
		'tq_ext_deadband':10.0, #deg
		'r_capstan':7.75 #mm
	}
}

config_default_h2_j3={
	'calib':h2r2.config_hand_h2r2_actuator_j3['calib'],
	'param':h2r2.config_hand_h2r2_actuator_j3['param'],
	'param_internal':
	{
		'calib_tq_degree':1,
		'pwm_theta':[-600,600],
		'joint_limits':[0,330.0],
		'tq_ext_amp':650.0, #ticks#-1*X for left hand
		'tq_ext_ks': 8.11, #Pounds-per-inch (spec 6.5, measured emperically)
		'tq_ext_windup':-100.0, #ticks
		'tq_ext_deadband':10.0, #deg
		'r_capstan':7.75 #mm
	}
}

config_default_h2_j4={
	'calib':h2r2.config_hand_h2r2_actuator_j4['calib'],
	'param':h2r2.config_hand_h2r2_actuator_j4['param'],
	'param_internal':
	{
		'calib_tq_degree':1,
		'pwm_theta':[-600,600],
		'joint_limits':[0,330.0],
		'tq_ext_amp':650.0, #ticks#-1*X for left hand
		'tq_ext_ks': 8.11, #Pounds-per-inch (spec 6.5, measured emperically)
		'tq_ext_windup':-100.0, #ticks
		'tq_ext_deadband':10.0, #deg
		'r_capstan':7.75 #mm
	}
}

		
class M3Calibrate_Hand_H2R2(M3CalibrateActuatorEcR2):
	def __init__(self):
		M3CalibrateActuatorEcR2.__init__(self)

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
			if self.jid==0:
				M3CalibrateActuatorEcR2.calibrate_theta(self)
			else:
				self.calibrate_theta()
			self.write_config()
			return True
		if ct=='ce':
			self.reset_sensor('torque')
			self.calibrate_torque_ext_spring()
			self.write_config()
			return True
		if M3CalibrateActuatorEcR2.do_task(self,ct):
			return True
		return False
	
	def print_tasks(self):
		print 'q: quit'
		print 'ds: display sensors'
		print 'et: ext_temp'
		print 'at: amp_temp'
		print 'ii: calibrate current'
		print 'tt: calibrate theta'
		print 'zt: zero theta'
		print 'zq: zero torque'
		#M3CalibrateActuatorEcR2.print_tasks(self)
		print 'ce: calibrate torque ext spring' 
		
	def calibrate_torque_ext_spring(self):
		#temporary, no 12bit encoder available for j0 calibration
		if self.jid==0:
			self.theta.ctype='ma3_10bit'
		print 'This routine requires actuator_ec PID control and encoder feedback of the extension spring'
		print 'Proceed [y]?'
		if not m3t.get_yes_no('y'):
			return
		if self.jid==0:
			print 'Routine not valid for joint 0'
			return
		print 'Enter amplitude [',self.param_internal['tq_ext_amp'],']'
		amp=m3t.get_float(self.param_internal['tq_ext_amp'])
		
		print 'Moving capstan to unloaded position. Enable power. Hit enter when ready'
		raw_input()
		self.comp_ec.command.mode=aec.ACTUATOR_EC_MODE_TORQUE
		self.comp_ec.command.t_desire=int(self.param_internal['tq_ext_windup'])
		self.step(4.0)
		self.comp_ec.command.mode=aec.ACTUATOR_EC_MODE_OFF
		self.step()
		print 'Unload gearhead by pushing on spring arm'
		print 'Place actuator in unloaded configuration. Hit enter when ready'
		raw_input()
		self.step()
		adc_zero=float(self.get_sensor_list_avg(['adc_torque'],1.0)['adc_torque'])
		enc_zero=self.theta.raw_2_deg(self.comp_rt.config['calib']['theta'],
					self.comp_ec.status.qei_on,self.comp_ec.status.qei_period,self.comp_ec.status.qei_rollover)
		ns=500.0
		t=nu.arange(.75,-.25-1.0/ns,-1.0/ns)
		print 'T',t
		des=adc_zero+amp*(.5*nu.sin(t*2*math.pi)+.5)
		print 'DES',des
		log_adc_torque=[]
		log_enc=[]
		log_mNm=[]
		#Init
		self.comp_ec.command.mode=aec.ACTUATOR_EC_MODE_TORQUE
		self.comp_ec.command.t_desire=int(adc_zero)
		print 'Connect extension spring. Enable motor power. Hit enter when ready'
		raw_input()
		self.step(time_sleep=1.0)
		for idx in range(len(des)):
			self.comp_ec.command.t_desire=int(des[idx])
			self.step(time_sleep=0.05)
			enc=self.theta.raw_2_deg(self.comp_rt.config['calib']['theta'],
					self.comp_ec.status.qei_on,self.comp_ec.status.qei_period,self.comp_ec.status.qei_rollover)
			print 'Des',des[idx],'Enc',enc-enc_zero
			if abs(enc-enc_zero)>self.param_internal['tq_ext_deadband']:
				ks=self.param_internal['tq_ext_ks']
				r=self.param_internal['r_capstan']/1000.0 #m
				dx=2*math.pi*m3u.m2in(r)*(enc-enc_zero)/360.0 #inches
				lb=dx*ks
				mN=m3u.Lb2mN(lb)
				tq_enc=mN*r
				tq_raw=self.comp_ec.status.adc_torque
				print '------------------------'
				print 'Des',idx,'/',len(t),': ',t[idx]
				print 'Adc SEA',tq_raw
				print 'Lb spring',lb
				print 'tq spring',tq_enc
				log_adc_torque.append(tq_raw)
				log_mNm.append(tq_enc)
				log_enc.append(enc-enc_zero)
		self.comp_ec.command.mode=aec.ACTUATOR_EC_MODE_OFF
		self.step()
		print 'Poly degree [',self.param_internal['calib_tq_degree'],']?'
		n=m3t.get_int(self.param_internal['calib_tq_degree'])
		poly,inv_poly=self.get_polyfit_to_data(x=log_adc_torque,y=log_mNm,n=n)
		self.write_raw_calibration({'log_adc_torque':log_adc_torque,'log_mNm':log_mNm,'log_enc':log_enc,
					    'cb_torque':poly,'cb_inv_torque':inv_poly})
		self.comp_rt.config['calib']['torque']['cb_torque']=poly
		self.comp_rt.config['calib']['torque']['cb_inv_torque']=inv_poly
		print 'Poly',poly
		s=m3tc.PolyEval(poly,log_adc_torque)
		m3t.mplot2(range(len(log_adc_torque)),log_mNm,s,xlabel='Samples',ylabel='Torque (mNm)',
			   y1name='loadcell',y2name='actuator')
	
		
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
		
	