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

# ###############################################################################################################

class M3CalibrateLoadX1Ec(M3Calibrate):
	def __init__(self):
		M3Calibrate.__init__(self)
		self.calib_default={
			'torque':{
				'type': 'adc_poly',
				'name': 'Linear torque-load cell',
				'cb_inv_torque': [1,0],
				'cb_torque': [1,0],
				'cb_scale': 1.0,
				'cb_bias': 0.0}
			}
	
	def start(self,ctype):
		if not M3Calibrate.start(self,ctype):
			return False
		self.torque=M3TorqueSensor(self.comp_rt.config['calib']['torque']['type'])
		return True
	
	def zero_torque(self):
		print 'Place load cell in unloaded configuration. Hit enter when ready'
		raw_input()
		avg_adc=float(self.get_sensor_list_avg(['adc_torque'],1.0)['adc_torque'])
		cs=self.torque.raw_2_mNm(self.comp_rt.config['calib']['torque'],avg_adc)
		print 'Old calibrated zero',cs
		self.comp_rt.config['calib']['torque']['cb_bias']-=cs
		print 'New calibrated zero',self.torque.raw_2_mNm(self.comp_rt.config['calib']['torque'],avg_adc)
				
		
	def calibrate_torque(self):
		print 'Place load cell in unloaded configuration. Hit enter when ready'
		raw_input()
		self.step()
		adc_zero=float(self.get_sensor_list_avg(['adc_torque'],1.0)['adc_torque'])
		print 'adc_zero',adc_zero
		print 'Number of calibration weights [3]?'
		nw=m3t.get_int(3)
		print 'Calibration hub diameter (mm)[70]?'
		ch=m3t.get_float(70)
		adc_act=[adc_zero]
		load_mNm=[0.0]
		for i in range(nw):
			print 'Place positive load ',i,'. Hit enter when ready'
			raw_input()
			self.step()
			adc_act.append(float(self.get_sensor_list_avg(['adc_torque'],1.0)['adc_torque']))
			print 'adc_act positive',adc_act[-1]
			
			print 'Place negative load ',i,'. Hit enter when ready'
			raw_input()
			self.step()
			adc_act.append(float(self.get_sensor_list_avg(['adc_torque'],1.0)['adc_torque']))
			print 'adc_act negative',adc_act[-1]
			
			print 'Enter load weight (g)'
			w=m3u.g2mN(m3t.get_float())*(ch/1000.0)
			load_mNm.append(w)
			load_mNm.append(-w)
			print 'Calibration load (mNm)',load_mNm[-1]
		print adc_act
		print load_mNm
		poly,inv_poly=self.get_polyfit_to_data(adc_act,load_mNm,n=1)
		self.comp_rt.config['calib']['torque']['cb_torque']=list(poly)
		self.comp_rt.config['calib']['torque']['cb_inv_torque']=list(inv_poly)

	def analyze_torque(self):
		print 'Calibration hub diameter (mm)[70]?'
		ch=m3t.get_float(70)
		while True:
			print 'Add calibration weight'
			print 'Continue [y]?'
			if m3t.get_yes_no('y'):
				for i in range(100):
					self.step(time_sleep=0.1)
					raw=self.comp_ec.status.adc_torque
					c=self.torque.raw_2_mNm(self.comp_rt.config['calib']['torque'],raw)
					c=m3u.mN2g(c/(ch/1000.0))
					print 'Estimated weight (g)',c	
			else:
				break
	
	def display_sensors(self):
		M3Calibrate.display_sensors(self)
		raw=self.comp_ec.status.adc_torque
		c=self.torque.raw_2_mNm(self.comp_rt.config['calib']['torque'],raw)
		print 'Torque:   (mNm) : '+'%3.2f'%c+' (inLb): '+'%3.2f'%m3u.mNm2inLb(c)+' (ADC) '+'%d'%raw
		
	def do_task(self,ct):
		if M3Calibrate.do_task(self,ct):
			return True
		if ct=='sa':
			sensors=['adc_torque']
			self.get_sensor_analysis(sensors,5.0)
			return True
		if ct=='zq':
			self.zero_torque()
			self.write_config()
			return True
		if ct=='ct':
			self.reset_sensor('torque')
			self.calibrate_torque()
			self.write_config()
			return True
		if ct=='aq':
			self.analyze_torque()
		return False
	
	def print_tasks(self):
		M3Calibrate.print_tasks(self)
		print 'sa: sensor analyze'
		print 'zq: zero torque'
		print 'ct: calibrate torque'
		print 'aq: analyze torque'
	
# ###############################################################################################################

class M3CalibrateLoadX6Ec(M3Calibrate):
	def __init__(self):
		M3Calibrate.__init__(self)
		self.calib_default={
			'wrench':{
				'type': '6x6_linear',
				'name': 'ATI 6ch Force-Torque sensor',
				'cb_fx': [1,0,0, 0,0,0],
				'cb_fy': [0,1,0, 0,0,0],
				'cb_fz': [0,0,1, 0,0,0],
				'cb_tx': [0,0,0, 1,0,0],
				'cb_ty': [0,0,0, 0,1,0],
				'cb_tz': [0,0,0, 0,0,1],
				'cb_adc_bias': [0,0,0, 0,0,0],
				'cb_scale': 1.0,
				'cb_bias': [0,0,0, 0,0,0]}
			}
	
	def start(self,ctype):
		if not M3Calibrate.start(self,ctype):
			return False
		self.wrench=M3WrenchSensor(self.comp_rt.config['calib']['wrench']['type'])
		return True

	def zero_load_cell(self):
		print 'Place sensor in zero load configuration'
		print 'Hit any key to continue'
		raw_input()
		sensors=['adc_load_0','adc_load_1','adc_load_2','adc_load_3','adc_load_4','adc_load_5']
		self.step()
		z=self.get_sensor_list_avg(sensors,3.0)
		w=self.wrench.raw_2_mNm(self.comp_rt.config['calib']['wrench'],
					z['adc_load_0'],
					z['adc_load_1'],
					z['adc_load_2'],
					z['adc_load_3'],
					z['adc_load_4'],
					z['adc_load_5'])
		bc=nu.array(self.comp_rt.config['calib']['wrench']['cb_bias'])
		self.comp_rt.config['calib']['wrench']['cb_bias']=list(bc-w)
		print 'New zero: '
		print self.wrench.raw_2_mNm(self.comp_rt.config['calib']['wrench'],
					self.comp_ec.status.adc_load_0,
					self.comp_ec.status.adc_load_1,
					self.comp_ec.status.adc_load_2,
					self.comp_ec.status.adc_load_3,
					self.comp_ec.status.adc_load_4,
					self.comp_ec.status.adc_load_5)
	
	def zero_load_cell_with_payload(self):
		print 'Place arm with mass pulling straight down on FTS'
		print 'Enter payload mass (g) [834g=H2R2wFTS]'
		payload=m3u.g2mN(m3t.get_float(834))
		sensors=['adc_load_0','adc_load_1','adc_load_2','adc_load_3','adc_load_4','adc_load_5']
		self.step()
		z=self.get_sensor_list_avg(sensors,3.0)
		print 'Sensor readings: ',z
		w=self.wrench.raw_2_mNm(self.comp_rt.config['calib']['wrench'],
					z['adc_load_0'],
					z['adc_load_1'],
					z['adc_load_2'],
					z['adc_load_3'],
					z['adc_load_4'],
					z['adc_load_5'])
		print 'Old payload',m3u.mN2g(w[2])
		w[2]=w[2]-payload
		bc=nu.array(self.comp_rt.config['calib']['wrench']['cb_bias'])
		self.comp_rt.config['calib']['wrench']['cb_bias']=list(bc-w)
		w=self.wrench.raw_2_mNm(self.comp_rt.config['calib']['wrench'],
					z['adc_load_0'],
					z['adc_load_1'],
					z['adc_load_2'],
					z['adc_load_3'],
					z['adc_load_4'],
					z['adc_load_5'])
		print 'New payload: ',m3u.mN2g(w[2])
		print 'New wrench',w
		
	def calibrate_load_cell(self):
		print 'Place sensor in zero load configuration'
		print 'Hit any key to continue'
		raw_input()
		sensors=['adc_load_0','adc_load_1','adc_load_2','adc_load_3','adc_load_4','adc_load_5']
		self.step()
		z=self.get_sensor_list_avg(sensors,3.0)
		print 'Zero'
		print z
		self.comp_rt.config['calib']['wrench']['cb_adc_bias'][0]=z['adc_load_0']
		self.comp_rt.config['calib']['wrench']['cb_adc_bias'][1]=z['adc_load_1']
		self.comp_rt.config['calib']['wrench']['cb_adc_bias'][2]=z['adc_load_2']
		self.comp_rt.config['calib']['wrench']['cb_adc_bias'][3]=z['adc_load_3']
		self.comp_rt.config['calib']['wrench']['cb_adc_bias'][4]=z['adc_load_4']
		self.comp_rt.config['calib']['wrench']['cb_adc_bias'][5]=z['adc_load_5']
		m=[500.0,1000.0,2000.0]
		log_mN=[]
		log_fZ=[]
		for g in m:
			print 'Place ',g,'(g) load on sensor in negative Fz direction'
			print 'Hit any key to continue'
			raw_input()
			sensors=['adc_load_0','adc_load_1','adc_load_2','adc_load_3','adc_load_4','adc_load_5']
			self.step()
			z=self.get_sensor_list_avg(sensors,2.0)
			self.comp_rt.config['calib']['wrench']['cb_scale']=1.0 #just determin this and zero
			w=self.wrench.raw_2_mNm(self.comp_rt.config['calib']['wrench'],
						z['adc_load_0'],
						z['adc_load_1'],
						z['adc_load_2'],
						z['adc_load_3'],
						z['adc_load_4'],
						z['adc_load_5'])
			#print 'Enter weight (g)'
			mN=-1.0*m3u.g2mN(g)#m3u.g2mN(m3t.get_float())
			log_mN.append(mN)
			est=w[2] #current Fz estimation, N
			log_fZ.append(est)
			print 'Measured (g)',m3u.mN2g(mN)
			print 'Estimated (g)',m3u.mN2g(est)
			scale=mN/est
			print 'Scale',scale
		poly,inv_poly=self.get_polyfit_to_data(x=log_fZ,y=log_mN,n=1)
		print 'Estimated scale',poly[0]
		self.comp_rt.config['calib']['wrench']['cb_scale']=poly[0]
		self.comp_rt.config['calib']['wrench']['cb_bias']=[0.0]*6
		w=self.wrench.raw_2_mNm(self.comp_rt.config['calib']['wrench'],
					z['adc_load_0'],
					z['adc_load_1'],
					z['adc_load_2'],
					z['adc_load_3'],
					z['adc_load_4'],
					z['adc_load_5'])
		#print 'New Estimated (g)',m3u.mN2g(w[2])
		
	def display_sensors(self):
		M3Calibrate.display_sensors(self)
		w=self.wrench.raw_2_mNm(self.comp_rt.config['calib']['wrench'],
					self.comp_ec.status.adc_load_0,
					self.comp_ec.status.adc_load_1,
					self.comp_ec.status.adc_load_2,
					self.comp_ec.status.adc_load_3,
					self.comp_ec.status.adc_load_4,
					self.comp_ec.status.adc_load_5)
		print '--------------Ec--------------------'
		print 0,self.comp_ec.status.adc_load_0
		print 1,self.comp_ec.status.adc_load_1
		print 2,self.comp_ec.status.adc_load_2
		print 3,self.comp_ec.status.adc_load_3
		print 4,self.comp_ec.status.adc_load_4
		print 5,self.comp_ec.status.adc_load_5
		print '--------------Calib-----------------'
		print 'Fx (g)',m3u.mN2g(w[0])
		print 'Fy (g)',m3u.mN2g(w[1])
		print 'Fz (g)',m3u.mN2g(w[2])
		print 'Tx (mNm)',w[3]
		print 'Ty (mNm)',w[4]
		print 'Tz (mNm)',w[5]
		print '---------------Rt-------------------'
		print 'Fx (g)',self.comp_rt.get_Fx_Kg()*1000.0
		print 'Fy (g)',self.comp_rt.get_Fy_Kg()*1000.0
		print 'Fz (g)',self.comp_rt.get_Fz_Kg()*1000.0
		print 'Tx (mNm)',self.comp_rt.get_Tx_mNm()
		print 'Ty (mNm)',self.comp_rt.get_Ty_mNm()
		print 'Tz (mNm)',self.comp_rt.get_Tz_mNm()
		

	def do_task(self,ct):
		if M3Calibrate.do_task(self,ct):
			return True
		if ct=='sa':
			sensors=['adc_load_0','adc_load_1','adc_load_2','adc_load_3','adc_load_4','adc_load_5']
			self.get_sensor_analysis(sensors,5.0)
			return True
		if ct=='zl':
			self.zero_load_cell()
			self.write_config()
			return True
		if ct=='cl':
			self.calibrate_load_cell()
			self.write_config()
			return True
		if ct=='zp':
			self.zero_load_cell_with_payload()
			self.write_config()
			return True
		return False
	
	def print_tasks(self):
		M3Calibrate.print_tasks(self)
		print 'sa: sensor analyze'
		print 'zl: zero load cell'
		print 'cl: calibrate load cell'
		print 'zp: zero load cell with payload'		
