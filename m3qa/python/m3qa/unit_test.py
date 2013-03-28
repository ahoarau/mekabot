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
import numarray as na
import Numeric as nu
from scipy import linspace, polyval, polyfit, sqrt, stats, randn
from pylab import plot, title, show , legend
import math
import os 
import sys
import yaml
import pylab as pyl

import m3.toolbox as m3t
import m3.toolbox_ctrl as m3tc
import m3.rt_proxy as m3p
import m3.component_factory as m3f
import m3.unit_conversion as m3u

# ###############################################################################################################		
class M3UnitTest:
	def __init__(self):
		self.proxy=None
		self.t_last=time.time()
	def start(self,ctype):
		#Return true if success
		#ctype. e.g.,: {'comp_ec':'m3actuator_ec','comp_rt':'m3actuator'}
		self.proxy=m3p.M3RtProxy()
		self.proxy.start()
		print 'Select component'
		ac=self.proxy.get_available_components(ctype['comp_rt'])
		if len(ac)==0:
			print 'Required components not available'
			return False
		self.name_rt=m3t.user_select_components_interactive(ac,single=True)[0]
		self.name_ec=self.name_rt.replace(ctype['comp_rt'],ctype['comp_ec'])
		self.comp_ec=m3f.create_component(self.name_ec)
		self.comp_rt=m3f.create_component(self.name_rt)
		if self.proxy.is_component_available(self.name_ec):
			self.proxy.subscribe_status(self.comp_ec)
			self.proxy.publish_command(self.comp_ec)
			self.proxy.publish_param(self.comp_ec)
			self.proxy.make_operational(self.name_ec)
		if self.proxy.is_component_available(self.name_rt):
			self.proxy.subscribe_status(self.comp_rt)
		pwr_ec=self.proxy.get_available_components('m3pwr_ec')
		pwr_rt=self.proxy.get_available_components('m3pwr')
		if len(pwr_rt):
			pr=m3f.create_component(pwr_rt[0])
			self.proxy.publish_command(pr)
			self.proxy.make_operational(pwr_rt[0])
			pr.set_motor_power_on()
		if len(pwr_ec):
			self.proxy.make_operational(pwr_ec[0])
		self.step()
		return True
	
	def stop(self):
		if self.proxy is not None:
			self.proxy.stop()
		
	def step(self,time_sleep=0.02):
		if time_sleep>0:
			t1=time.time()
			computation_time =t1-self.t_last
			time.sleep(max(time_sleep,time_sleep-computation_time))
			self.t_last=t1
		self.proxy.step()
		
	
	def get_test(self):
		while True:
			print 'Unit Test:?'
			print '----------------------'
			self.print_tests()
			ct=raw_input()
			if ct=='q':
				return
			if not self.do_test(ct):
				print 'Invalid type'
			
	def print_tests(self):
		print 'q: quit'
		print 'ds: display sensors'
		
	def do_test(self,ct):
		if ct=='ds':
			while True:
				print 'Hit q to quit, any other key to display'
				if m3t.get_keystroke()=='q':
					break
				ts=time.time()
				while(time.time()-ts<5.0):
					self.display_sensors()
					self.step(0.1)
		return False
	
	def get_test_results(self):
		fn=m3t.get_m3_config_path()+'data/unit_test_results_'+self.comp_rt.name+'.yml'
		try:
			f=file(fn,'r')
			raw_data= yaml.safe_load(f.read())
			f.close()
			return raw_data
		except (IOError, EOFError):
			raw_data={}
			print 'Test results file',fn,'not available'
			return None
	
	def write_test_results(self,data):
		print 'Save test data [y]?'
		if m3t.get_yes_no('y'):
			fn=m3t.get_m3_config_path()+'data/unit_test_results_'+self.comp_rt.name+'.yml'
			try:
				f=file(fn,'r')
				raw_data= yaml.safe_load(f.read())
				f.close()
			except (IOError, EOFError):
				raw_data={}
				pass
			f=file(fn,'w')
			for k in data.keys():
				raw_data[k]=data[k]
			print 'Saving...',fn
			f.write(yaml.safe_dump(data, default_flow_style=False,width=200))
			f.close()
			
	def display_sensors(self):
		print '--------------------'
		
	def get_sensor_list_avg(self,sensor_list,trial_time,auto=False,verbose=True):
		# sensor_list e.g., ['qei_on','qei_period']
		# trial_time: seconds
		# return {'qei_on':1.0,'qei_period: 4.0}
		if not auto:
			print 'Measuring sensors',sensor_list
			print 'Hit any key to begin'
			raw_input()
		dt=0
		ts=time.time()
		log=[[] for x in sensor_list]
		self.step()
		while dt<trial_time:
			self.step()
			data=m3t.GetDictFromMsg(self.comp_ec.status)
			sample=[]
			for idx in range(len(sensor_list)):
				if verbose:
					print sensor_list[idx],':',data[sensor_list[idx]]
				log[idx].append(data[sensor_list[idx]])
			if verbose:
				print
			dt=time.time()-ts
		ret={}
		for idx in range(len(sensor_list)):
			ret[sensor_list[idx]]=na.array(log[idx],na.Float32).mean()
		return ret
	
	def get_sensor_analysis(self,sensors,trial_time):
		# sensors e.g., ['qei_on','qei_period']
		# trial_time: seconds
		sensor_list=[]
		while True:
			print '---------- Available Sensors----------'
			for idx in range(len(sensors)):
				print idx,' : ',sensors[idx]
			print 'Add sensor? '
			id=m3t.get_int()
			if id>=0 and id<len(sensors):
				sensor_list.append(sensors[id])
				break
		print 'Measuring sensor',sensor_list
		print 'Hit any key to begin'
		raw_input()
		dt=0
		ts=time.time()
		log=[[] for x in sensor_list]
		self.step()
		while dt<trial_time:
			self.step()
			data=m3t.GetDictFromMsg(self.comp_ec.status)
			sample=[]
			for idx in range(len(sensor_list)):
				print sensor_list[idx],data[sensor_list[idx]]
				print 'Timestamp',self.comp_ec.status.base.timestamp
				log[idx].append(data[sensor_list[idx]])
			print
			dt=time.time()-ts
		ret={}
		for idx in range(len(sensor_list)):
			a=na.array(log[idx],na.Float32)
			avg=a.mean()
			std=a.stddev()
			p2p=max(a)-min(a)
			print '-----------',sensor_list[idx],'--------------'
			print 'Mean:',avg
			print 'Std Dev:',std
			print 'Peak To Peak:',p2p
			pyl.title('Noise Distribution for '+sensor_list[idx])
			pyl.hist(x=a-avg,bins=pyl.array(range(-10,10,1)))
			pyl.show()
			ret[sensor_list[idx]]={'avg':avg,'std':std,'p2p':p2p}
			print 'Hit enter to continue'
			raw_input()
		return ret

