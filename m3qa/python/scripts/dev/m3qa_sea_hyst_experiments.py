#! /usr/bin/python
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


import time
import m3.rt_proxy as m3p
import m3.toolbox as m3t
import m3.toolbox_ctrl as m3tc
import m3.component_factory as m3f
import Numeric as nu
import m3qa.calibrate_actuator_ec_r2 as m3a
import math
from m3qa.calibrate import M3Calibrate
import m3.actuator_ec_pb2 as aec
	
class M3SeaHystExperiment(m3a.M3CalibrateActuatorEcR2):
	def __init__(self):
		m3a.M3CalibrateActuatorEcR2.__init__(self)
		self.dpm3=None
	def start(self,ctype):
		if not m3a.M3CalibrateActuatorEcR2.start(self,ctype):
			return False
		#print 'This routine requires the actuator to be locked with a DPM3 load-cell in-line'
		#print 'Is DPM3 Loadcell present? [y]'
		#if not m3t.get_yes_no('y'):
			#return False
		#print 'Is the DPM3 resolution set at XXXX.X? [y]'
		#if not m3t.get_yes_no('y'):
			#return False
		self.dpm3=m3a.dpm3_thread()
		self.dpm3.start()
		return True
	
	def stop(self):
		if self.dpm3 is not None:
			self.dpm3.stop()
		m3a.M3CalibrateActuatorEcR2.stop(self)
		
	def display_sensors(self):
		m3a.M3Calibrate.display_sensors(self)
		lmNm=self.dpm3.get_load_mNm()
		#raw=self.comp_ec.status.adc_torque
		#c=self.torque.raw_2_mNm(self.comp_rt.config['calib']['torque'],raw)
		#print 'Torque:   (mNm) : '+'%3.2f'%c+' (inLb): '+'%3.2f'%m3u.mNm2inLb(c)+' (ADC) '+'%d'%raw
		print 'Load cell (mNm)',lmNm

	def experiment_a(self):
		
		print 'Prepare experiment. Hit enter when ready'
		raw_input()
		print 'Enable power. Hit enter when ready'
		raw_input()
		self.step() #send current commands out to dsp and read sensors
		adc_zero=self.comp_ec.status.adc_torque
		
		print 'Enter amplitude (ticks)[100]'
		amp=m3t.get_float(100)
		ns=100.0
		#Sawtooth torque profile
		fwd=nu.arange(0,1.0,1/ns)*amp
		rev=nu.arange(0,-1.0,-1/ns)*amp
		zero=nu.zeros(int(ns))
		fwd=fwd.tolist()+zero.tolist()
		rev=rev.tolist()+zero.tolist()
		#Place actuator in torque mode
		self.comp_ec.command.mode=aec.ACTUATOR_EC_MODE_TORQUE
		log_adc_torque=[]
		log_load_mNm=[]
		self.comp_ec.command.t_desire=0 #Initial desired torque
		#Send out intial configuration (zero torque) and wait 1 second
		self.step(time_sleep=1.0)
		
		for f in fwd:
			self.comp_ec.command.t_desire=int(f+adc_zero) #send out command torque
			self.step(time_sleep=0.05) #send new desired out and wait
			lmNm=self.dpm3.get_load_mNm()#read load-cell
			tq=self.comp_ec.status.adc_torque #read raw torque sensor (ticks)
			print '------------------------'
			print 'Fwd',f
			print 'Adc SEA',tq
			print 'mNm Load Cell',lmNm
			log_adc_torque.append(tq) #log data
			log_load_mNm.append(lmNm)
		
		for r in rev:
			self.comp_ec.command.t_desire=int(r+adc_zero) #send out command torque
			self.step(time_sleep=0.05)
			lmNm=self.dpm3.get_load_mNm()#-load_zero
			tq=self.comp_ec.status.adc_torque
			print '------------------------'
			print 'Rev',r
			print 'Adc SEA',tq
			print 'mNm Load Cell',lmNm
			log_adc_torque.append(tq)
			log_load_mNm.append(lmNm)
		
		self.comp_ec.command.mode=aec.ACTUATOR_EC_MODE_OFF #Turn off actuator
		self.step()#send out command
		
		#Least squares fit to get nominal calibration
		poly,inv_poly=self.get_polyfit_to_data(x=log_adc_torque,y=log_load_mNm,n=1,draw=False)
		#Compute calibrated view of raw torque data
		s=m3tc.PolyEval(poly,log_adc_torque)
		#Draw the comparison between loadcell and torque sensor
		m3t.mplot2(range(len(log_adc_torque)),log_load_mNm,s,xlabel='Samples',ylabel='Torque (mNm)',
			   y1name='loadcell',y2name='actuator')
		#Save raw data
		self.write_raw_calibration({'log_adc_torque':log_adc_torque,\
		                            'log_load_mNm':log_load_mNm})
	def experiment_b(self):
		print 'Hello B!'
		# Command motor to run sine curvature
		print 'Prepare experiment. Hit enter when ready'
		raw_input()
		print 'Enable power. Hit enter when ready'
		raw_input()
		self.step() #send current commands out to dsp and read sensors
		adc_zero=self.comp_ec.status.adc_torque
		
		ns=100.0
		amp = 100;
		# Generate Sine curvature as
		x_temp=nu.arange(0,50.0,1/ns)
		sin_curvature = nu.sin(x_temp)*amp
		
		#Place actuator in torque mode
		self.comp_ec.command.mode=aec.ACTUATOR_EC_MODE_TORQUE
		log_adc_torque=[]
		log_load_mNm=[]
		self.comp_ec.command.t_desire=0 #Initial desired torque
		#Send out intial configuration (zero torque) and wait 1 second
		self.step(time_sleep=1.0)
		
		for f in sin_curvature:
			self.comp_ec.command.t_desire=int(f+adc_zero) #send out command torque
			self.step(time_sleep=0.05) #send new desired out and wait
			lmNm=self.dpm3.get_load_mNm()#read load-cell
			tq=self.comp_ec.status.adc_torque #read raw torque sensor (ticks)
			print '------------------------'
			print 'Fwd',f
			print 'Adc SEA',tq
			print 'mNm Load Cell',lmNm
			log_adc_torque.append(tq) #log data
			log_load_mNm.append(lmNm)
		self.comp_ec.command.mode=aec.ACTUATOR_EC_MODE_OFF #Turn off actuator
		self.step()#send out command
		
		#Least squares fit to get nominal calibration
		poly,inv_poly=self.get_polyfit_to_data(x=log_adc_torque,y=log_load_mNm,n=1,draw=False)
		#Compute calibrated view of raw torque data
		s=m3tc.PolyEval(poly,log_adc_torque)
		#Draw the comparison between loadcell and torque sensor
		m3t.mplot2(range(len(log_adc_torque)),log_load_mNm,s,xlabel='Samples',ylabel='Torque (mNm)',
			   y1name='loadcell',y2name='actuator')
		#Save raw data
		self.write_raw_calibration({'log_adc_torque':log_adc_torque,\
		                            'log_load_mNm':log_load_mNm})
		
	def do_task(self,ct):
		if M3Calibrate.do_task(self,ct):
			return True
		if ct=='ea':
			self.experiment_a()
		if ct=='eb':
			self.experiment_b()
		return False
	
	def print_tasks(self):
		M3Calibrate.print_tasks(self)
		print 'ea: experiment-a'
		print 'eb: experiment-b'

		
calib=M3SeaHystExperiment()
try:
	if calib.start({'comp_ec':'m3actuator_ec','comp_rt':'m3actuator','comp_j':'m3joint'}): 
		calib.get_task()
except (KeyboardInterrupt,EOFError):
	pass
except m3t.M3Exception,e:
	print 'M3Exception',e
finally:
	print 'Shutting down system'
	calib.stop()
	
