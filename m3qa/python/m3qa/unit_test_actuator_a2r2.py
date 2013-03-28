#Copyright  2010, Meka Robotics
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

import m3.toolbox as m3t
import m3.actuator_ec_pb2 as aec
import numpy.numarray as na
from m3qa.unit_test import *
import math
from calibrate_actuator_ec_r2 import dpm3_thread
import unit_test_toolbox as utt

# ###################################### A2 J0 ##############################################################
test_default_a2_j0={
	'test_torques':{'small':1000.0,'medium':4000.0,'large':10000.0} 
}

# ######################################## A2 J1 ############################################################
test_default_a2_j1={
	'test_torques':{'large':10000.0} 
}
# ########################################## A2 J2 ##########################################################
test_default_a2_j2={

}
# ############################################# A2 J3 #######################################################
test_default_a2_j3={

}
# ############################################# A2 J4 #######################################################
test_default_a2_j4={
	
}
# ############################################# A2 J5 #######################################################
test_default_a2_j5={

}
# ############################################# A2 J6 #######################################################
test_default_a2_j6={

}


		
class M3UnitTest_Actuator_A2R2(M3UnitTest):
	def __init__(self):
		M3UnitTest.__init__(self)
	def start(self,ctype):
		if not M3UnitTest.start(self,ctype):
			return False
		self.jid=int(self.comp_ec.name[self.comp_ec.name.find('_j')+2:])
		self.test_default=[
			test_default_a2_j0,
			test_default_a2_j1,
			test_default_a2_j2,
			test_default_a2_j3,
			test_default_a2_j4,
			test_default_a2_j5,
			test_default_a2_j6]
		return True
	
	def do_test(self,ct):
		if ct=='ht':
			self.test_torque_hysterisis()
			return True
		if ct=='bn':
			self.test_torque_banger()
		if ct=='tt':
			self.test_torque_tracking()
			return True
		if M3UnitTest.do_test(self,ct):
			return True
		return False
	
	def print_tests(self):
		M3UnitTest.print_tests(self)
		print 'ht: test torque hystersis'
		print 'bn: test torque banger'
		print 'tt: test torque tracking'
		
	# ##################################################################
	def is_dpm3_ready(self):
		print 'This routine requires the actuator torque to be calibrated'
		print 'This routine requires the actuator to be locked with a DPM3 load-cell in-line'
		print 'Is DPM3 Loadcell present? [y]'
		if not m3t.get_yes_no('y'):
			return False
		print 'Is the DPM3 resolution set at XXXX.X? [y]'
		if not m3t.get_yes_no('y'):
			return False
		print 'Place actuator in unloaded configuration. Enable Power. Hit enter when ready'
		raw_input()
		return True
		
	# ##################################################################		
	def test_torque_tracking(self,auto=False):
		if not auto and not self.is_dpm3_ready():
			return 
		res={}
		for k in self.test_default[self.jid]['test_torques'].keys():
			amp=self.test_default[self.jid]['test_torques'][k]
			print '------------------'
			print 'Tracking test:',k,':',amp
			log_torque_mNm, log_load_mNm=self.sweep_torque_sine(amp,zero=True,loadcell=True)
			err=utt.measure_avg_error(log_torque_mNm, log_load_mNm)
			acc=utt.measure_accuracy(log_torque_mNm, log_load_mNm)
			print 'Average err',err
			print 'Accuracy (%)',acc
			res[k]={'tracking_err_mNm':err,'tracking_accuracy_pct':acc,'log_torque_mNm':log_torque_mNm,'log_load_mNm':log_load_mNm}
			m3t.mplot2(range(len(log_torque_mNm)),log_load_mNm,log_torque_mNm,xlabel='Samples',ylabel='Torque (mNm)',
				   y1name='loadcell',y2name='actuator')
		self.write_test_results({'test_torque_tracking':res})
	# ##################################################################	
	def test_torque_hysterisis(self,auto=False):
		if not auto and not self.is_dpm3_ready():
			return 
		res={}
		for k in self.test_default[self.jid]['test_torques'].keys():
			amp=self.test_default[self.jid]['test_torques'][k]
			print '------------------'
			print 'Hysterisis test:',k,':',amp
			log_torque_mNm_a, log_load_mNm_a=self.slew_to_torque(amp,3.0,zero=False,loadcell=True)
			log_torque_mNm_b, log_load_mNm_b=self.slew_to_torque(-1*amp,3.0,zero=False,loadcell=True)
			p=na.array(log_torque_mNm_a,na.Float32).mean()
			n=na.array(log_torque_mNm_b,na.Float32).mean()
			pl=na.array(log_load_mNm_a,na.Float32).mean()
			nl=na.array(log_load_mNm_b,na.Float32).mean()
			res[k]={'sea':{'avg_pos':p,'avg_neg':n,'hystersis':abs(p-n), 'zero':(p+n)/2.0},
				'loadcell':{'avg_pos':pl,'avg_neg':nl,'hystersis':abs(pl-nl), 'zero':(pl+nl)/2.0}}
			print 'Hystersis SEA',res[k]['sea']['hystersis'],'mNm'
			print 'Hystersis Loadcell',res[k]['loadcell']['hystersis'],'mNm'
		self.write_test_results({'test_torque_tracking':res})
		
		# ##################################################################	
	def test_torque_precision(self,auto=False):
		if not auto and not self.is_dpm3_ready():
			return 
		res={}
		amp=self.test_default[self.jid]['test_torques']['medium']
		print '------------------'
		print 'Hysterisis test:',k,':',amp
		log_torque_mNm_a, log_load_mNm_a=self.slew_to_torque(amp,3.0,zero=False,loadcell=True)
		log_torque_mNm_b, log_load_mNm_b=self.slew_to_torque(-1*amp,3.0,zero=False,loadcell=True)
		p=na.array(log_torque_mNm_a,na.Float32).mean()
		n=na.array(log_torque_mNm_b,na.Float32).mean()
		pl=na.array(log_load_mNm_a,na.Float32).mean()
		nl=na.array(log_load_mNm_b,na.Float32).mean()
		res[k]={'sea':{'avg_pos':p,'avg_neg':n,'hystersis':abs(p-n), 'zero':(p+n)/2.0},
			'loadcell':{'avg_pos':pl,'avg_neg':nl,'hystersis':abs(pl-nl), 'zero':(pl+nl)/2.0}}
		print 'Hystersis SEA',res[k]['sea']['hystersis'],'mNm'
		print 'Hystersis Loadcell',res[k]['loadcell']['hystersis'],'mNm'
		self.write_test_results({'test_torque_tracking':res})
	# ##################################################################
	
	def slew_to_torque(self,amp,slew_time,zero=True,loadcell=False):
		if loadcell:
			dpm3=dpm3_thread()
			dpm3.start()
		self.proxy.publish_command(self.comp_rt)
		self.proxy.publish_param(self.comp_rt)
		self.proxy.make_operational(self.name_rt)
		self.step()
		load_zero=0
		torque_zero=0
		if zero:
			self.step()
			load_zero=dpm3.get_load_mNm()
			torque_zero=self.comp_rt.get_torque_mNm()
			print 'Load zero',load_zero
			print 'Actuator zero',torque_zero
		ns=slew_time/0.1
		t=nu.arange(0,1+1/ns,1/ns)
		self.comp_rt.set_mode_torque()
		for idx in range(len(t)):
			des=amp*math.sin(math.pi*0.5*t[idx])
			self.comp_rt.set_torque_mNm(des)
			self.step(time_sleep=0.1)
			lmNm=0
			if loadcell:
				lmNm=dpm3.get_load_mNm()-load_zero
			tq=self.comp_rt.get_torque_mNm()-torque_zero
			print '------------------------'
			print 'Des',idx,'/',len(t),': ',t[idx]
			print 'mNm Des',des
			print 'mNm Actuator',tq
			print 'mNm Load Cell',lmNm
		self.step(0.5)
		log_torque_mNm=[]
		log_load_mNm=[]
		for i in range(10):
			lmNm=0
			if loadcell:
				lmNm=dpm3.get_load_mNm()-load_zero
			log_load_mNm.append(lmNm)
			log_torque_mNm.append(self.comp_rt.get_torque_mNm()-torque_zero)
		self.comp_rt.set_mode_off()
		self.step()
		self.proxy.make_safe_operational(self.name_rt)
		if loadcell:
			dpm3.stop()
		return log_torque_mNm, log_load_mNm
		
	def sweep_torque_sine(self,amp,zero=True,loadcell=False):
		if loadcell:
			dpm3=dpm3_thread()
			dpm3.start()
		self.proxy.publish_command(self.comp_rt)
		self.proxy.publish_param(self.comp_rt)
		self.proxy.make_operational(self.name_rt)
		self.step()
		load_zero=0
		torque_zero=0
		if zero:
			self.step()
			if loadcell:
				load_zero=dpm3.get_load_mNm()
			torque_zero=self.comp_rt.get_torque_mNm()
			print 'Load zero',load_zero
			print 'Actuator zero',torque_zero
		ns=100.0
		t=nu.arange(-1,1+1/ns,1/ns)
		self.comp_rt.set_mode_torque()
		log_torque_mNm=[]
		log_load_mNm=[]
		for idx in range(len(t)):
			des=amp*math.sin(math.pi*2*t[idx])
			self.comp_rt.set_torque_mNm(des)
			self.step(time_sleep=0.1)
			lmNm=0
			if loadcell:
				lmNm=dpm3.get_load_mNm()-load_zero
			tq=self.comp_rt.get_torque_mNm()-torque_zero
			print '------------------------'
			print 'Des',idx,'/',len(t),': ',t[idx]
			print 'mNm Des',des
			print 'mNm Actuator',tq
			print 'mNm Load Cell',lmNm
			log_torque_mNm.append(tq)
			log_load_mNm.append(lmNm)
		self.comp_rt.set_mode_off()
		self.step()
		self.proxy.make_safe_operational(self.name_rt)
		if loadcell:
			dpm3.stop()
		return log_torque_mNm, log_load_mNm
		
		

		
	def test_torque_banger(self,auto=False):
		d=30.0
		if not auto:
			print '------------- Test Conditions --------------'
			print '1. Actuator on test-plate'
			print '2. Rocker arm installed'
			print '3. PID tuning complete for actuator_ec'
			print '4. Rocker arm unloaded'
			print 'Continue? [y]'
			if not m3t.get_yes_no('y'):
				return
			print 'Enter run duration (s) [30.0]'
			d=m3t.get_float(30.0)
			print 'Enable power. Hit enter to continue'
			raw_input()
		self.step()
		amplitude=self.test_default[self.jid]['test_banger_amp']
		adc_zero=float(self.get_sensor_list_avg(['adc_torque'],1.0,auto=True,verbose=False)['adc_torque'])
		print 'Zero of ',adc_zero
		self.comp_ec.command.mode=aec.ACTUATOR_EC_MODE_TORQUE
		self.comp_ec.command.t_desire=int(adc_zero)
		self.step(1.0)
		nc=2
		period=5.0
		start=time.time()
		dt=0
		self.comp_ec.command.mode=aec.ACTUATOR_EC_MODE_TORQUE
		while dt<d:
			dt=time.time()-start
			a=amplitude[self.jid]*math.sin(2*math.pi*dt/period)
			self.comp_ec.command.t_desire=int(adc_zero+a)
			self.step(time_sleep=0.1)
			print '----------'
			print 'DT',dt
			print 'Amplitude',a
			print 'Current (mA)',self.comp_rt.get_current_mA()
		self.comp_ec.command.mode=aec.ACTUATOR_EC_MODE_OFF
		self.step()
	

		
	
	
	
	
	