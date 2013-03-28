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


	
class M3CalibrateActuatorEcR4(M3Calibrate):
	def __init__(self):
		M3Calibrate.__init__(self)
		#self.param_internal={
			#'calib_tq_lc_amp':100.0,
			#'analyze_tq_lc_amp':1000.0,
			#'calib_lever_mass':327.6,
			#'calib_lever_com':157.5,
			#'calib_lever_len':304.8,
			#'calib_tq_degree':1,
			#'calib_hub_diam':70,
			#'joint_limits': [-10,10]
			#}
		
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
		
	def rescale_torque(self):
		print 'Rescaling torque'
		if self.comp_rt.config['calib']['torque']['type']=='sea_vertx_14bit':
			scale=self.comp_rt.config['calib']['torque']['cb_scale']
			bias=self.comp_rt.config['calib']['torque']['cb_bias']
			print 'Old',self.comp_rt.config['calib']['torque']
			print '--------------'
			tq=self.torque.raw_2_mNm(self.comp_rt.config['calib']['torque'],2048.0)
			adc=self.torque.mNm_2_raw(self.comp_rt.config['calib']['torque'],0.0)
			print 'Old Torque(2048)=',tq
			print 'Old InvTorque(',tq,'): ',self.torque.mNm_2_raw(self.comp_rt.config['calib']['torque'],tq)
			print 'Old InvTorque(0)=',adc
			print 'Old Torque (',adc,'): ',self.torque.raw_2_mNm(self.comp_rt.config['calib']['torque'],adc)
			
			self.comp_rt.config['calib']['torque']['cb_torque'][0]=self.comp_rt.config['calib']['torque']['cb_torque'][0]*scale
			self.comp_rt.config['calib']['torque']['cb_torque'][1]=self.comp_rt.config['calib']['torque']['cb_torque'][1]*scale
			self.comp_rt.config['calib']['torque']['cb_inv_torque'][0]=self.comp_rt.config['calib']['torque']['cb_inv_torque'][0]/scale
			self.comp_rt.config['calib']['torque']['cb_scale']=1.0
			print 'New',self.comp_rt.config['calib']['torque']
			print '--------------'
			tq=self.torque.raw_2_mNm(self.comp_rt.config['calib']['torque'],2048.0)
			adc=self.torque.mNm_2_raw(self.comp_rt.config['calib']['torque'],0.0)
			print 'New Torque(2048)=',tq
			print 'New InvTorque(',tq,'): ',self.torque.mNm_2_raw(self.comp_rt.config['calib']['torque'],tq)
			print 'New InvTorque(0)=',adc
			print 'New Torque (',adc,'): ',self.torque.raw_2_mNm(self.comp_rt.config['calib']['torque'],adc)
		else:
			print 'Incorrect calibration type',self.comp_rt.config['calib']['torque']['type']	
			
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
		  
		qs_to_qj = 1.0
		if self.comp_j is not None:
		    qs_to_qj = self.comp_j.config['transmission']['qs_to_qj'][0]
		    
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
		theta_as=self.theta.raw_2_deg(self.comp_rt.config['calib']['theta'],q_on,q_p,q_r, qs_to_qj)
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
		theta_bs=self.theta.raw_2_deg(self.comp_rt.config['calib']['theta'],q_on2,q_p2,q_r2, qs_to_qj)
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
			theta_as=self.theta.raw_2_deg(self.comp_rt.config['calib']['theta'],q_on,q_p,q_r, qs_to_qj)
			theta_bs=self.theta.raw_2_deg(self.comp_rt.config['calib']['theta'],q_on2,q_p2,q_r2, qs_to_qj)
		
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
		adj=bias*(1-qs_to_qj)/qs_to_qj ##This is too complicated. qs_to_j should be 1.0 and cb_bias should capture encoder gearing in r4.
		#TODO: Fix how qs_to_qj is handled in calibrate_sensors.py in R4. Done on wrong side of cb_scale
		self.comp_rt.config['calib']['theta']['cb_bias']=bias+adj
		print 'Zero bias',self.comp_rt.config['calib']['theta']['cb_bias']
		theta_as=self.theta.raw_2_deg(self.comp_rt.config['calib']['theta'],q_on,q_p,q_r, qs_to_qj)
		theta_bs=self.theta.raw_2_deg(self.comp_rt.config['calib']['theta'],q_on2,q_p2,q_r2, qs_to_qj)
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
			
			
	def calibrate_torque_loadcell_from_file(self):
	    data=self.get_raw_calibration()
	    print 'Enter degree [',self.param_internal['calib_tq_degree'],']?'
	    n=m3t.get_int(self.param_internal['calib_tq_degree'])
	    poly,inv_poly=self.get_polyfit_to_data(x=data['log_adc_torque'],y=data['log_load_mNm'],n=n)
	    self.write_raw_calibration({'log_adc_torque':data['log_adc_torque'],'log_load_mNm':data['log_load_mNm'],
					'cb_torque':poly,'cb_inv_torque':inv_poly})
	    self.comp_rt.config['calib']['torque']['cb_torque']=poly
	    self.comp_rt.config['calib']['torque']['cb_inv_torque']=inv_poly
	    print 'Poly',poly
	    s=m3tc.PolyEval(poly,data['log_adc_torque'])
	    m3t.mplot2(range(len(data['log_adc_torque'])),data['log_load_mNm'],s,xlabel='Samples',ylabel='Torque (mNm)',
			   y1name='loadcell',y2name='actuator')
	def calibrate_torque_loadcell(self):
		print 'This calibration routine requires the actuator to be locked with a DPM3 load-cell in-line'
		print 'Is DPM3 Loadcell present? [y]'
		if not m3t.get_yes_no('y'):
			return
		print 'Is the DPM3 resolution set at XXXX.X? [y]'
		if not m3t.get_yes_no('y'):
			return
		dpm3=dpm3_thread()
		dpm3.start()
		print 'Zero load-cell'
		print 'Place actuator in unloaded configuration. Hit enter when ready'
		raw_input()
		self.step()
		adc_zero=float(self.get_sensor_list_avg(['adc_torque'],1.0)['adc_torque'])
		#load_zero=self.comp_l.get_torque_mNm()
		print 'Enable power'
		print 'Enter amplitude [',self.param_internal['calib_tq_lc_amp'],']'
		amp=m3t.get_float(self.param_internal['calib_tq_lc_amp'])
		ampx=[amp,0.5*amp]
		ns=500.0
		t=nu.arange(-1,1+1/ns,1/ns)
		self.comp_ec.command.mode=aec.ACTUATOR_EC_MODE_TORQUE
		log_adc_torque=[]
		log_load_mNm=[]
		#Init
		des=amp*math.sin(math.pi*2*-1.0)+adc_zero
		self.comp_ec.command.t_desire=int(des)
		self.step(time_sleep=1.0)
		for a in ampx:
		    for idx in range(len(t)):
			    des=a*math.sin(math.pi*2*t[idx])+adc_zero #amp*t[idx]+adc_zero#
			    self.comp_ec.command.t_desire=int(des)
			    self.step(time_sleep=0.02)
			    self.step(time_sleep=0.02)
			    lmNm=dpm3.get_load_mNm()#-load_zero
			    tq=self.comp_ec.status.adc_torque
			    print '------------------------'
			    print 'Des',idx,'/',len(t),': ',t[idx]
			    print 'Adc SEA',tq
			    print 'mNm Load Cell',lmNm
			    log_adc_torque.append(tq)
			    log_load_mNm.append(lmNm)
		
		log_adc_torque=log_adc_torque[10:]
		log_load_mNm=log_load_mNm[10:]
		self.comp_ec.command.mode=aec.ACTUATOR_EC_MODE_OFF
		self.step()
		dpm3.stop()
		
		print 'Enter degree [',self.param_internal['calib_tq_degree'],']?'
		n=m3t.get_int(self.param_internal['calib_tq_degree'])
		poly,inv_poly=self.get_polyfit_to_data(x=log_adc_torque,y=log_load_mNm,n=n)
		self.write_raw_calibration({'log_adc_torque':log_adc_torque,'log_load_mNm':log_load_mNm,
					    'cb_torque':poly,'cb_inv_torque':inv_poly})
		self.comp_rt.config['calib']['torque']['cb_torque']=poly
		self.comp_rt.config['calib']['torque']['cb_inv_torque']=inv_poly
		print 'Poly',poly
		s=m3tc.PolyEval(poly,log_adc_torque)
		m3t.mplot2(range(len(log_adc_torque)),log_load_mNm,s,xlabel='Samples',ylabel='Torque (mNm)',
			   y1name='loadcell',y2name='actuator')
		
	def calibrate_torque_weighted_lever(self):
		print 'This routine requires M3Joint Theta mode control of a weighted lever'
		print 'Theta = 0 should be calibrated to vertical'
		print 'Theta = 90 should be calibrated to horizontal (pointing right when facing actuator output).'
		print 'Proceed [y]?'
		if not m3t.get_yes_no('y'):
			return

		print 'Calibration lever COM (mm)[',self.param_internal['calib_lever_com'],']?'
		com=m3t.get_float(self.param_internal['calib_lever_com'])
		print 'Calibration lever mass (g)[',self.param_internal['calib_lever_mass'],']?'
		cm=m3t.get_float(self.param_internal['calib_lever_mass'])
		clq = m3u.g2mN(cm)*(com/1000.0)
		print 'Calibration lever len (mm)[',self.param_internal['calib_lever_len'],']?'
		cl=m3t.get_float(self.param_internal['calib_lever_len'])
		print 'Calibration lever torque (mNm): ',clq
		
		print 'Number of calibration weights [3]?'
		nw=m3t.get_int(3)
		
		print 'Place actuator in unloaded configuration'
		print 'Give a small impulse in first direction, let return to zero'
		print 'Hit any key when ready'
		raw_input()
		raw_a=int(self.get_sensor_list_avg(['adc_torque'],1.0)['adc_torque'])
		print 'Give a small impulse in second direction, let return to zero'
		print 'Hit any key when ready'
		raw_input()
		raw_b=int(self.get_sensor_list_avg(['adc_torque'],1.0)['adc_torque'])
		adc_zero=raw_a+(raw_b-raw_a)/2.0
		print 'adc_zero',adc_zero
		
		print 'Turn on power. Hit enter when ready'
		raw_input()
		
		poses=[90,45,-45,-90]
		self.proxy.subscribe_status(self.comp_j)
		self.proxy.publish_command(self.comp_j)
		self.proxy.publish_param(self.comp_j)
		self.proxy.make_operational(self.name_j)
		self.proxy.make_operational(self.name_rt)
		self.comp_j.set_mode_theta()
		self.comp_j.set_slew_rate(25.0)
		self.step()
		
		adc_act=[adc_zero]
		load_mNm=[0.0]
		for i in range(nw):
			print 'Enter load weight (g)'
			w=m3u.g2mN(m3t.get_float())*(cl/1000.0)
			for pose in poses:
				print 'Posing arm. Hit enter when ready'
				raw_input()
				self.comp_j.set_theta_deg(pose)
				self.step(time_sleep=3.0)
				print 'Acquiring torque. Hit enter when ready'
				raw_input()
				self.step()
				adc_act.append(float(self.get_sensor_list_avg(['adc_torque'],1.0)['adc_torque']))
				print 'adc_act',adc_act[-1]
				gc=math.sin(self.comp_j.get_theta_rad())*(w+clq)
				print 'Theta',self.comp_j.get_theta_rad(),'TorqueV',(w+clq)
				load_mNm.append(gc)
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
	def analyze_torque_gravity_comp(self):
		print 'This routine requires Torque mode control of a known weighted lever'
		print 'Theta = 0 should be calibrated to vertical'
		print 'Theta = 90 should be calibrated to horizontal (pointing right when facing actuator output).'
		print 'Proceed [y]?'
		if not m3t.get_yes_no('y'):
			return 
		self.proxy.publish_command(self.comp_rt)
		self.proxy.publish_param(self.comp_rt)
		self.proxy.subscribe_status(self.comp_rt)
		self.proxy.make_operational(self.name_rt)
		print 'Enter test lever mass (g) [',self.param_internal['calib_lever_mass'],']'
		tlm=m3t.get_float(self.param_internal['calib_lever_mass'])
		print 'Enter test lever COM (mm) [',self.param_internal['calib_lever_com'],']'
		com=m3t.get_float(self.param_internal['calib_lever_com'])
		print 'Enter test lever length (mm) [',self.param_internal['calib_lever_len'],']'
		tll=m3t.get_float(self.param_internal['calib_lever_len'])
		print 'Enter payload (g)[500]'
		tp=m3t.get_float(500)
		self.comp_rt.set_mode_torque()
		tq_tl=m3u.g2mN(tlm)*com/1000.0
		tq_p= m3u.g2mN(tp)*tll/1000.0
		print 'Test lever torque (at 90 deg) (mNm)',tq_tl
		print 'Payload torque (at 90 deg) (mNm)',tq_p
		ts=time.time()
		while True:
			tqg=1.0*math.sin(self.comp_rt.get_theta_rad())*(tq_tl+tq_p)
			self.comp_rt.set_torque_mNm(tqg)
			self.step()
			e=tqg-self.comp_rt.get_torque_mNm()
			
			print 'Tqg',tqg,'Dt',time.time()-ts, 'Error',e
			if time.time()-ts>20.0:
				print 'Continue [y]?'
				if m3t.get_yes_no('y'):
					print 'Enter payload (g)[',tp,']'
					tp=m3t.get_float(tp)
					self.comp_rt.set_mode_torque()
					tq_tl=m3u.g2mN(tlm)*tll/1000.0
					tq_p= m3u.g2mN(tp)*tll/1000.0
					ts=time.time()
				else:
					break
		self.comp_rt.set_mode_off()
		self.proxy.make_safe_operational(self.name_rt)	
		self.step()
		
	def calibrate_torque_hub(self,run_neg=True):
		print 'This calibration requires a hub with known diameter loaded by a weight'
		print 'Place actuator in unloaded configuration. Hit enter when ready'
		raw_input()
		self.step()
		adc_zero=float(self.get_sensor_list_avg(['adc_torque'],1.0)['adc_torque'])
		print 'adc_zero',adc_zero
		print 'Number of calibration weights [3]?'
		nw=m3t.get_int(3)
		print 'Calibration hub diameter (mm)[',self.param_internal['calib_hub_diam'],']?'
		ch=m3t.get_float(self.param_internal['calib_hub_diam'])/2.0
		adc_act=[adc_zero]
		load_mNm=[0.0]
		for i in range(nw):
			print 'Place positive load ',i,'. Hit enter when ready'
			raw_input()
			self.step()
			adc_act.append(float(self.get_sensor_list_avg(['adc_torque'],1.0)['adc_torque']))
			print 'adc_act positive',adc_act[-1]
			
			if run_neg:
				print 'Place negative load ',i,'. Hit enter when ready'
				raw_input()
				self.step()
				adc_act.append(float(self.get_sensor_list_avg(['adc_torque'],1.0)['adc_torque']))
				print 'adc_act negative',adc_act[-1]
			
			print 'Enter load weight (g)'
			w=m3u.g2mN(m3t.get_float())*(ch/1000.0)
			load_mNm.append(w)
			if run_neg:
				load_mNm.append(-w)
			print 'Calibration load (mNm)',load_mNm[-1]
		print adc_act
		print load_mNm
		print 'Enter degree [',self.param_internal['calib_tq_degree'],']?'
		n=m3t.get_int(self.param_internal['calib_tq_degree'])
		poly,inv_poly=self.get_polyfit_to_data(adc_act,load_mNm,n=n)
		self.comp_rt.config['calib']['torque']['cb_torque']=list(poly)
		self.comp_rt.config['calib']['torque']['cb_inv_torque']=list(inv_poly)
		
	def analyze_torque_hub(self):
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
			
	def analyze_torque_loadcell(self):
		print 'This routine requires the actuator to be locked with a load-cell in-line'
		print 'Place actuator in unloaded configuration. Enable Power. Hit enter when ready'
		raw_input()
		self.proxy.publish_command(self.comp_rt)
		self.proxy.publish_param(self.comp_rt)
		self.proxy.make_operational(self.name_rt)
		self.step()
		print 'Zero sensors [n]?'
		load_zero=0
		torque_zero=0
		if m3t.get_yes_no('n'):
			self.step()
			load_zero=self.comp_l.get_torque_mNm()
			torque_zero=self.comp_rt.get_torque_mNm()
			print 'Load zero',load_zero
			print 'Actuator zero',torque_zero
		print 'Enter amplitude [',self.param_internal['calib_tq_lc_amp'],']'
		amp=m3t.get_float(self.param_internal['calib_tq_lc_amp'])
		ns=100.0
		t=nu.arange(-1,1+1/ns,1/ns)
		self.comp_rt.set_mode_torque()
		log_torque_mNm=[]
		log_load_mNm=[]
		log_des_mNm=[]
		for idx in range(len(t)):
			des=amp*math.sin(math.pi*2*t[idx])
			log_des_mNm.append(des)
			self.comp_rt.set_torque_mNm(des)
			self.step(time_sleep=0.1)
			self.step(time_sleep=0.1)
			lmNm=self.comp_l.get_torque_mNm()-load_zero
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
		tdelta=nu.array(log_des_mNm)-nu.array(log_torque_mNm)
		terr=nu.sqrt(nu.sum(tdelta**2))
		print 'Average tracking error (mNm)',terr, 'Pct',terr/self.param_internal['calib_tq_lc_amp'], 'Max',nu.maximum(nu.absolute(tdelta))
		
		print '0: Plot torque_sensor vs loadcell'
		print '1: Plot torque_sensor vs desired '
		print 'Enter plot type [1]'
		pt=m3t.get_int(1)
		if pt==0:
		    m3t.mplot2(range(len(log_torque_mNm)),log_load_mNm,log_torque_mNm,xlabel='Samples',ylabel='Torque (mNm)',
			       y1name='loadcell',y2name='actuator')
		if pt==1:
		    m3t.mplot2(range(len(log_torque_mNm)),log_des_mNm,log_torque_mNm,xlabel='Samples',ylabel='Torque (mNm)',
			       y1name='desired',y2name='actuator')  
		#m3t.mplot(log_load_mNm,log_torque_mNm,xlabel='LoadCell (mNm)',ylabel='Actuator (mNm)')
		
	def display_torque_calibration(self):
		d=self.get_raw_calibration()
		if d is not None:
			n=len(d['cb_torque'])-1
			#self.get_polyfit_to_data(x=d['log_adc_torque'],y=d['log_load_mNm'],n=n)
			s=m3tc.PolyEval(d['cb_torque'],d['log_adc_torque'])
			m3t.mplot2(range(len(d['log_adc_torque'])),d['log_load_mNm'],s,xlabel='Samples',ylabel='Torque (mNm)')
			
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
		if ct=='gc':
			self.analyze_torque_gravity_comp()
			return True
		if ct=='cl':
			self.reset_sensor('torque')
			self.calibrate_torque_loadcell()
			self.write_config()
			return True
		if ct=='cf':
		    self.reset_sensor('torque')
		    self.calibrate_torque_loadcell_from_file()
		    self.write_config()
		    return True
		if ct=='cw':
			self.reset_sensor('torque')
			self.calibrate_torque_weighted_lever()
			self.write_config()
			return True
		if ct=='al':
			self.analyze_torque_loadcell()
			return True
		if ct=='ch':
			self.reset_sensor('torque')
			self.calibrate_torque_hub()
			self.write_config()
			return True
		if ct=='ah':
			self.analyze_torque_hub()
			return True
		if ct=='ch':
			self.reset_sensor('torque')
			self.calibrate_torque_hub()
			self.write_config()
			return True
		if ct=='ah':
			self.analyze_torque_hub()
			return True
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
		if ct=='rq':
			self.rescale_torque()
			self.write_config()
			return True
		if ct=='dt':
			self.display_torque_calibration()
			
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
		print 'cl: calibrate torque loadcell'
		print 'cf: calibrate torque loadcell from file'
		print 'al: analyze   torque loadcell'
		print 'cw: calibrate torque weighted lever'
		print 'dt: display torque calibration'
		print 'gc: analyze torque gravity comp'
		
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
				

				
