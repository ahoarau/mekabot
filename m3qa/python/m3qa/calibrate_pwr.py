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
import m3qa.calibrate_sensors as m3s


class M3CalibratePwrEc_V0_3(M3Calibrate):
	def __init__(self):
		M3Calibrate.__init__(self)
		self.param_default={'max_bus_voltage': 36.0,
				    'max_current_digital': 5000,
				    'min_bus_voltage': 18.0}
		self.calib_default={
			'voltage':{
				'type': 'adc_poly',
				'cb_inv_voltage': [0,0],
				'cb_voltage': [0,0],
				'cb_scale': 1.0,
				'cb_bias': 0.0},
			'current':{
				'type': 'adc_linear_5V',
				'name': 'Allegro ACS712-05',
				'cb_mV_per_A': -185.0, #Power board inverts signal
				'cb_ticks_at_zero_a': 2048.0,
				'cb_ticks_at_zero_b': 2048.0,
				'cb_scale': 1.0,
				'cb_bias': 0.0}}
	
	def start(self,ctype):
		if not M3Calibrate.start(self,ctype):
			return False
		self.current=m3s.M3CurrentSensor(self.comp_rt.config['calib']['current']['type'])
		self.voltage=m3s.M3VoltageSensor(self.comp_rt.config['calib']['voltage']['type'])
		return True
	
	def calibrate_bus_voltage(self):
		adc_list=[]
		v_list=[]
		v_des=[18,20,25,30]
		for vd in v_des:
			print 'Set Bus Voltage to',vd
			print 'Current Bus Voltage (V): '
			v=m3t.get_float()
			adc=self.get_sensor_list_avg(['adc_bus_voltage'],1.0)['adc_bus_voltage']
			print 'Adc bus voltage',adc
			adc_list.append(adc)
			v_list.append(v)
		poly,inv_poly=self.get_polyfit_to_data(x=adc_list,y=v_list,n=1)
		self.comp_rt.config['calib']['voltage']['cb_voltage']=poly
		self.comp_rt.config['calib']['voltage']['cb_inv_voltage']=inv_poly
	
	def calibrate_current_digital(self):
		print 'Enter input bus voltage (V)'
		v=m3t.get_float()	
		print 'Enter input bus current (mA)'
		ma=m3t.get_float()
		ma=v*ma/15.0 #15V system, scale input wattage to get the current reading
		raw=self.get_sensor_list_avg(['adc_current_digital'],1.0)['adc_current_digital']
		ma_p=self.current.raw_2_mA(self.comp_rt.config['calib']['current'],raw,raw)
		ma_err=ma-ma_p
		print 'Estimated mA',ma
		print 'Previous calibration mA',ma_p
		print 'Error of',ma_err
		self.comp_rt.config['calib']['current']['cb_bias']=ma_err
		print 'New calibration mA',self.current.raw_2_mA(self.comp_rt.config['calib']['current'],raw,raw)
		
		
	def do_task(self,ct):
		if ct=='ii':
			self.reset_sensor('current')
			self.calibrate_current_digital()
			self.write_config()
			return True
		if ct=='bv':
			self.reset_sensor('voltage')
			self.calibrate_bus_voltage()
			self.write_config()
			return True
		if M3Calibrate.do_task(self,ct):
			return True
		return False
			
	def print_tasks(self):
		M3Calibrate.print_tasks(self)
		print 'ii: current'
		print 'bv: bus voltage'
		
	def display_sensors(self):
		M3Calibrate.display_sensors(self)
		raw=self.comp_ec.status.adc_bus_voltage
		v=self.voltage.raw_2_V(self.comp_rt.config['calib']['voltage'],raw)
		print 'Bus Voltage: (V) : '+'%3.2f'%v+' (ADC) '+'%d'%raw
		
		raw=self.comp_ec.status.adc_current_digital
		c=self.current.raw_2_mA(self.comp_rt.config['calib']['current'],raw,raw)
		print 'Current:   (mA) : '+'%3.2f'%c+' (ADC): '+'%d'%raw
		

# #################################################################################################

class M3CalibratePwrEc_V0_4(M3Calibrate):
	def __init__(self):
		M3Calibrate.__init__(self)
		self.param_default={'max_bus_voltage': 36.0,
				    'max_current_digital': 5000,
				    'min_bus_voltage': 18.0}
		self.calib_default={
			'voltage':{
				'type': 'adc_poly',
				'cb_inv_voltage': [0,0],
				'cb_voltage': [0,0],
				'cb_scale': 1.0,
				'cb_bias': 0.0},
			'current':{
				'type': 'adc_linear_5V',
				'name': 'Allegro ACS712-05',
				'cb_mV_per_A': -185.0, #Power board inverts signal
				'cb_ticks_at_zero_a': 2048.0,
				'cb_ticks_at_zero_b': 2048.0,
				'cb_scale': 1.0,
				'cb_bias': 0.0}}
	
	def start(self,ctype):
		if not M3Calibrate.start(self,ctype):
			return False
		self.current=m3s.M3CurrentSensor(self.comp_rt.config['calib']['current']['type'])
		self.voltage=m3s.M3VoltageSensor(self.comp_rt.config['calib']['voltage']['type'])
		return True
	
	def calibrate_bus_voltage(self):
		adc_list=[]
		v_list=[]
		v_des=[18,20,25,30]
		#diode_drop=0.7
		for vd in v_des:
			print 'Set Bus Voltage to',vd
			print 'Current Bus Voltage (V): '
			v=m3t.get_float()
			adc=self.get_sensor_list_avg(['adc_bus_voltage'],1.0)['adc_bus_voltage']
			print 'Adc bus voltage',adc
			adc_list.append(adc)
			v_list.append(v)
		poly,inv_poly=self.get_polyfit_to_data(x=adc_list,y=v_list,n=1)
		self.comp_rt.config['calib']['voltage']['cb_voltage']=poly
		self.comp_rt.config['calib']['voltage']['cb_inv_voltage']=inv_poly
	
	def calibrate_current_digital(self):
		print 'Enter input bus voltage (V)'
		v=m3t.get_float()	
		print 'Enter input bus current (mA)'
		ma=m3t.get_float()
		ma=v*ma/15.0 #15V system, scale input wattage to get the current reading
		raw=self.get_sensor_list_avg(['adc_current_digital'],1.0)['adc_current_digital']
		ma_p=self.current.raw_2_mA(self.comp_rt.config['calib']['current'],raw,raw)
		ma_err=ma-ma_p
		print 'Estimated mA',ma
		print 'Previous calibration mA',ma_p
		print 'Error of',ma_err
		self.comp_rt.config['calib']['current']['cb_bias']=ma_err
		print 'New calibration mA',self.current.raw_2_mA(self.comp_rt.config['calib']['current'],raw,raw)
		
		
	def do_task(self,ct):
		if ct=='ii':
			self.reset_sensor('current')
			self.calibrate_current_digital()
			self.write_config()
			return True
		if ct=='bv':
			self.reset_sensor('voltage')
			self.calibrate_bus_voltage()
			self.write_config()
			return True
		if M3Calibrate.do_task(self,ct):
			return True
		return False
			
	def print_tasks(self):
		M3Calibrate.print_tasks(self)
		print 'ii: current'
		print 'bv: bus voltage'
		
	def display_sensors(self):
		M3Calibrate.display_sensors(self)
		raw=self.comp_ec.status.adc_bus_voltage
		v=self.voltage.raw_2_V(self.comp_rt.config['calib']['voltage'],raw)
		print 'Bus Voltage: (V) : '+'%3.2f'%v+' (ADC) '+'%d'%raw
		
		raw=self.comp_ec.status.adc_current_digital
		c=self.current.raw_2_mA(self.comp_rt.config['calib']['current'],raw,raw)
		print 'Current:   (mA) : '+'%3.2f'%c+' (ADC): '+'%d'%raw

# #################################################################################################

class M3CalibratePwrEc_V0_5(M3CalibratePwrEc_V0_4):
	def __init__(self):
		M3CalibratePwrEc_V0_4.__init__(self)
