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
import m3.actuator_ec_pb2 as aec
import serial
from threading import Thread

# ###############################################################################################################
class dpm3_thread(Thread):
    def __init__ (self):
	Thread.__init__(self)
	self.done=False
	self.ser = serial.Serial('/dev/ttyUSB0', 19200, bytesize=8, parity='N', stopbits=1, timeout=None, xonxoff=0, rtscts=0)
	self.load_mNm=0
    def get_load_mNm(self):
	return self.load_mNm
    def stop(self):
	self.done=True
	time.sleep(1.0)
    def run(self):
	print 'Starting DPM3 thread'
	start=time.time()
	while not self.done:
	    self.line=self.ser.readline()
	    try:
		#With XXXX.X resolution, divide by 10
		self.load_mNm = m3u.inLb2mNm(float(self.line)/10.0)   # read a '\n' terminated line
	    except ValueError:
		print 'Bad DPM3 line: ',self.line
		self.load_mNm=0
	print 'Exiting DPM3 thread'
# ###############################################################################################################


	
class M3CalibrateActuatorEcA1R1(M3Calibrate):
	def __init__(self):
		M3Calibrate.__init__(self)

		
	def start(self,ctype):
		if not M3Calibrate.start(self,ctype):
			return False
		self.ext_temp=M3TempSensor(self.comp_rt.config['calib']['ext_temp']['type'])
		self.amp_temp=M3TempSensor(self.comp_rt.config['calib']['amp_temp']['type'])
		self.current=M3CurrentSensor(self.comp_rt.config['calib']['current']['type'])
		self.theta=M3AngleSensor(self.comp_rt.config['calib']['theta']['type'])
		self.torque=M3TorqueSensor(self.comp_rt.config['calib']['torque']['type'])
		self.name_j=self.name_rt.replace(ctype['comp_rt'],ctype['comp_j'])
		self.comp_j=m3f.create_component(self.name_j)
				
		if self.comp_j is None:
			print 'Joint component not found. Proceeding without it...'
		self.comp_l=None
		print 'Select component'
		ac=self.proxy.get_available_components('m3loadx1')
		if len(ac)==0:
			print 'Required loadx1 components not available. Proceeding without it'
			return True
		self.name_load=m3t.user_select_components_interactive(ac,single=True)[0]
		self.comp_l=m3f.create_component(self.name_load)
		self.proxy.subscribe_status(self.comp_l)
		return True
		
	def zero_temp(self,sensor,adc_sensor):
		print 'Zero sensor manually [y]?'
		if m3t.get_yes_no('y'):
			avg_adc=int(self.get_sensor_list_avg([adc_sensor],1.0)[adc_sensor])
			print 'Enter temp (C)'
			cm=m3t.get_float()
			if sensor=='ext_temp':
				cs=self.ext_temp.raw_2_C(self.comp_rt.config['calib'][sensor],avg_adc)
			if sensor=='amp_temp':
				cs=self.amp_temp.raw_2_C(self.comp_rt.config['calib'][sensor],avg_adc)
			print 'Old calibrated temp',cs
			print 'Delta',cm-cs
			self.comp_rt.config['calib'][sensor]['cb_bias']+=cm-cs
			if sensor=='ext_temp':
				print 'New calibrated temp',self.ext_temp.raw_2_C(self.comp_rt.config['calib'][sensor],avg_adc)
			if sensor=='amp_temp':
				print 'New calibrated temp',self.amp_temp.raw_2_C(self.comp_rt.config['calib'][sensor],avg_adc)
				
	def zero_current(self):
		print 'Zero sensor [y]?'
		if m3t.get_yes_no('y'):
			print 'Hit enter when motor power is off'
			raw_input()
			avg=self.get_sensor_list_avg(['adc_current_a','adc_current_b'],1.0)
			avg_a=int(avg['adc_current_a'])
			avg_b=int(avg['adc_current_b'])
			io=self.current.raw_2_mA(self.comp_rt.config['calib']['current'],avg_a,avg_b)
			print 'Old calibrated current',io
			self.comp_rt.config['calib']['current']['cb_ticks_at_zero_a']=avg_a
			self.comp_rt.config['calib']['current']['cb_ticks_at_zero_b']=avg_b
			print 'New calibrated current',self.current.raw_2_mA(self.comp_rt.config['calib']['current'],avg_a,avg_b)
		
	def zero_torque(self):
		print 'Zeroing torque'
		print 'Place actuator in unloaded configuration'
		print 'Give a small impulse in first direction, let return to zero'
		print 'Hit any key when ready'
		raw_input()
		raw_a=int(self.get_sensor_list_avg(['adc_torque'],1.0)['adc_torque'])
		
		print 'Give a small impulse in second direction, let return to zero'
		print 'Hit any key when ready'
		raw_input()
		raw_b=int(self.get_sensor_list_avg(['adc_torque'],1.0)['adc_torque'])
		raw_c=raw_a+(raw_b-raw_a)/2.0
		print 'Delta',raw_b-raw_a,'Mid',raw_c
		tqo=self.torque.raw_2_mNm(self.comp_rt.config['calib']['torque'],raw_c)
		print 'Old torque (mNm):',tqo
		print 'Old bias',self.comp_rt.config['calib']['torque']['cb_bias']
		self.comp_rt.config['calib']['torque']['cb_bias']-=tqo
		print 'New torque (mNm):',self.torque.raw_2_mNm(self.comp_rt.config['calib']['torque'],raw_c)
		print 'New bias',self.comp_rt.config['calib']['torque']['cb_bias']

	def zero_theta(self):
		print 'Zeroing theta. This assumes that sensor is calibrated but need bias adjustment'
		print 'Proceed [y]?'
		if not m3t.get_yes_no('y'):
			return
		print 'Manually place actuator at zero hard-stop'
		print 'Hit any key when ready'
		raw_input()
		
		q_on=int(self.get_sensor_list_avg(['qei_on'],1.0)['qei_on'])
		q_p=self.comp_ec.status.qei_period
		q_r=self.comp_ec.status.qei_rollover
		bb=self.theta.raw_2_deg(self.comp_rt.config['calib']['theta'],q_on,q_p,q_r)
		
		print 'Old bias',self.comp_rt.config['calib']['theta']['cb_bias']
		self.comp_rt.config['calib']['theta']['cb_bias']-=bb
		self.proxy.step()
		q_on=self.comp_ec.status.qei_on
		q_p=self.comp_ec.status.qei_period
		q_r=self.comp_ec.status.qei_rollover
		bb=self.theta.raw_2_deg(self.comp_rt.config['calib']['theta'],q_on,q_p,q_r)
		print 'New theta (deg):',bb
		print 'New bias',self.comp_rt.config['calib']['theta']['cb_bias']
			
			
	def calibrate_theta(self, use_pwm=False, use_brake=False, disable_qei_lim=False):
		print 'Calibrating theta...'
		if use_pwm:
		    print 'Joint will be pwm driven to limits. Proceed [y]?'
		    if not m3t.get_yes_no('y'):
			return 
		else:
		    print 'This routine requires the ability to manually backdrive the joint to both extremes.'
		    print 'Continue? [y]'
		    if not m3t.get_yes_no('y'):
			return
		    
		if disable_qei_lim:
		    pconfig=self.comp_ec.param.config #disable qei limits
		    self.comp_ec.param.config=0 
		    
		self.proxy.publish_command(self.comp_rt)
		self.proxy.publish_param(self.comp_rt)
		self.proxy.make_operational(self.name_rt)
		self.step()
		
		if not use_pwm:
		    print 'Move joint to one extreme. Hit any key when done'
		    raw_input()
		    self.step()
		else:
		    print 'Moving joint to first limit. Hit any key when ready'
		    raw_input()
		    self.comp_rt.set_mode_pwm()
		    if use_brake:
			self.comp_rt.set_brake_off()
		    print 'Desired pwm? [',self.param_internal['pwm_theta'][0],']?'
		    p=int(m3t.get_float(self.param_internal['pwm_theta'][0]))
		    self.comp_rt.set_pwm(p)
		    self.step()
		    print 'Hit any key when motion done'
		    raw_input()
		    self.step()
		q_on=self.comp_ec.status.qei_on
		q_p=self.comp_ec.status.qei_period
		q_r=self.comp_ec.status.qei_rollover
		theta_as=self.theta.raw_2_deg(self.comp_rt.config['calib']['theta'],q_on,q_p,q_r)
		print 'Raw (Deg)',theta_as
		print 'Expected joint limits: ',self.param_internal['joint_limits']
		print 'Enter theta (Deg)'
		theta_a=m3t.get_float()
		
		if not use_pwm:
		    print 'Move joint to other extreme. Hit any key when done'
		    raw_input()
		    self.step()
		else:
		    print 'Moving joint to second limit. Hit any key when ready'
		    raw_input()
		    print 'Desired pwm? [',self.param_internal['pwm_theta'][1],']?'
		    p=int(m3t.get_float(self.param_internal['pwm_theta'][1]))
		    self.comp_rt.set_pwm(p)
		    self.step()
		    print 'Hit any key when motion done'
		    raw_input()
		    self.step()
		    
		q_on2=self.comp_ec.status.qei_on
		q_p2=self.comp_ec.status.qei_period
		q_r2=self.comp_ec.status.qei_rollover
		theta_bs=self.theta.raw_2_deg(self.comp_rt.config['calib']['theta'],q_on2,q_p2,q_r2)
		print 'Raw (Deg)',theta_bs
		
		print 'Expected joint limits: ',self.param_internal['joint_limits']
		print 'Enter theta (Deg)'
		theta_b=m3t.get_float()
		
		self.comp_rt.set_mode_off()
		if disable_qei_lim:
		    self.comp_ec.param.config=pconfig #enable qei limits
		self.step()
		self.proxy.make_safe_operational(self.name_rt)
		self.step()
		
		if (theta_a>theta_b and theta_as<theta_bs) or (theta_a<theta_b and theta_as>theta_bs):
			self.comp_rt.config['calib']['theta']['cb_scale']*=-1.0
			theta_as=self.theta.raw_2_deg(self.comp_rt.config['calib']['theta'],q_on,q_p,q_r)
			theta_bs=self.theta.raw_2_deg(self.comp_rt.config['calib']['theta'],q_on2,q_p2,q_r2)
		
		print 'Sensed range of', theta_bs-theta_as, 'Given range of',theta_b-theta_a
		e=abs((theta_bs-theta_as))-abs((theta_b-theta_a))
		print 'Sensing > Given by error of: ',e
		print 'Adjusting given to match sensing'
		
		if theta_a>=theta_b and e>=0:
			taa=theta_a+abs(e/2.0)
			tbb=theta_b-abs(e/2.0)
		if theta_b>=theta_a and e>=0:
			taa=theta_a-abs(e/2.0)
			tbb=theta_b+abs(e/2.0)
		if theta_a>=theta_b and e<=0:
			taa=theta_a-abs(e/2.0)
			tbb=theta_b+abs(e/2.0)
		if theta_b>=theta_a and e<=0:
			taa=theta_a+abs(e/2.0)
			tbb=theta_b-abs(e/2.0)
			
		print 'Spliting error to give range:',taa-tbb,'for ',taa,' to ',tbb 
		m=(tbb-taa)/(theta_bs-theta_as)
		print 'Slope',m
		bias=taa-m*theta_as
		self.comp_rt.config['calib']['theta']['cb_bias']=bias
		print 'Zero bias',bias
		theta_as=self.theta.raw_2_deg(self.comp_rt.config['calib']['theta'],q_on,q_p,q_r)
		theta_bs=self.theta.raw_2_deg(self.comp_rt.config['calib']['theta'],q_on2,q_p2,q_r2)
		print 'New calibrated range',theta_as,theta_bs
		
		max_q=max(taa,tbb)
		min_q=min(taa,tbb)
		if self.comp_j is not None:
			print 'Setting joint limits to',min_q,max_q
			print 'Expected joint limits: ',self.param_internal['joint_limits']
			self.comp_j.param.max_q=float(max_q) 
			self.comp_j.param.min_q=float(min_q)
		else:
			print 'Joint component missing. Unable to set joint limits to',min_q,max_q
			
			
			
	def display_sensors(self):
		M3Calibrate.display_sensors(self)
		raw=self.comp_ec.status.adc_ext_temp
		c=self.ext_temp.raw_2_C(self.comp_rt.config['calib']['ext_temp'],raw)
		print 'Ext Temp: (F) : '+'%3.2f'%m3u.C2F(c)+' (C): '+'%d'%c+' (ADC) '+'%d'%raw
		raw=self.comp_ec.status.adc_amp_temp
		c=self.amp_temp.raw_2_C(self.comp_rt.config['calib']['amp_temp'],raw)
		print 'Amp Temp:   (F) : '+'%3.2f'%m3u.C2F(c)+' (C): '+'%3.2f'%c+' (ADC) '+'%d'%raw
		raw_a=self.comp_ec.status.adc_current_a
		raw_b=self.comp_ec.status.adc_current_b
		c=self.current.raw_2_mA(self.comp_rt.config['calib']['current'],raw_a,raw_b)
		print 'Current:   (mA) : '+'%3.2f'%c+' (ADC_A): '+'%d'%raw_a+' (ADC_B) '+'%d'%raw_b
		q_on=self.comp_ec.status.qei_on
		q_p=self.comp_ec.status.qei_period
		q_r=self.comp_ec.status.qei_rollover
		c=self.theta.raw_2_deg(self.comp_rt.config['calib']['theta'],q_on,q_p,q_r)
		print 'Theta:    (Deg) : '+'%3.3f'%c+' Qei On '+'%d'%q_on+' Qei Period '+'%d'%q_p+' Qei Rollover '+'%d'%q_r
		raw=self.comp_ec.status.adc_torque
		c=self.torque.raw_2_mNm(self.comp_rt.config['calib']['torque'],raw)
		print 'Torque:   (mNm) : '+'%3.2f'%c+' (inLb): '+'%3.2f'%m3u.mNm2inLb(c)+' (ADC) '+'%d'%raw
		if self.comp_l is not None:
			print 'Load cell (mNm)',self.comp_l.get_torque_mNm()
			
	def do_task(self,ct):
		if ct=='ii':
			self.reset_sensor('current')
			self.zero_current()
			self.write_config()
			return True
		if ct=='et' :
			self.reset_sensor('ext_temp')
			self.zero_temp('ext_temp','adc_ext_temp')
			self.write_config()
			return True
		if ct=='at' :
			self.reset_sensor('amp_temp')
			self.zero_temp('amp_temp','adc_amp_temp')
			self.write_config()
			return True
		if ct=='sa':
			sensors=['qei_on','adc_torque','adc_current_a','adc_current_b','adc_amp_temp','adc_ext_temp']
			self.get_sensor_analysis(sensors,5.0)
			return True
		if ct=='tt':
			self.reset_sensor('theta')
			self.calibrate_theta()
			self.write_config()
		if ct=='zq':
			self.zero_torque()
			self.write_config()
			return True
		if ct=='zt':
			self.zero_theta()
			self.write_config()
			return True
		if M3Calibrate.do_task(self,ct):
			return True
		return False
	
	def print_tasks(self):
		M3Calibrate.print_tasks(self)
		print 'et: ext_temp'
		print 'at: amp_temp'
		print 'sa: sensor analyze'
		print 'ii: calibrate current'
		print 'tt: calibrate theta'
		print 'zt: zero theta'
		print 'zq: zero torque'
		
	def write_config(self):
		print 'Save calibration (y/n)?'
		save=raw_input()
		if save=='y':
			self.comp_rt.config['calibration_date']=time.asctime()
			self.comp_rt.write_config()
			if self.comp_ec is not None:
				self.comp_ec.write_config()
			if self.comp_j is not None:
				self.comp_j.config['calibration_date']=time.asctime()
				self.comp_j.write_config()
				

				
