#! /usr/bin/python
# -*- coding: utf-8 -*-
import pylab as pyl
import time
import m3.rt_proxy as m3p
import m3.toolbox as m3t
import m3.component_factory as mcf
import numpy.numarray as na
import math
from m3qa.calibrate_sensors import *

print 'This script will calibrate the zero-torque of the A2 arm and T2 torso based on the current torque_gravity estimate'
#print 'It will not calibrate joint J5,J6 of the A2 arm'
print 'The robot should be powered off in a neutral pose'
print '----------------------------------------------'
print 'Enter limb type (0: A2 arm, 1: T2 torso, 2: H2 hand)'
lt=m3t.get_int()
proxy = m3p.M3RtProxy()
proxy.start()

cnames=proxy.get_available_components()
comp={}
cnames=[q for q in cnames if q.find('actuator_ec')!=-1] 
if lt==0:
	cnames=[q for q in cnames if (q.find('_ma')!=-1)]
if lt==1:
	cnames=[q for q in cnames if (q.find('_mt')!=-1)]
if lt==2:
	cnames=[q for q in cnames if (q.find('_mh')!=-1)]

for c in cnames:
	comp[c]={'comp_rt':None,'comp_j':None,'torque_act':[],'torque_joint':[],'torque_gravity':[],'is_wrist':False}
	if (c.find('j5')>=0 or c.find('j6')>=0) and lt==0:
		comp[c]['is_wrist']=True

if len(cnames)==0:
	print 'No components found'
	exit()
	
print 'Using components: '
for i in range(len(cnames)):
	print i,':',cnames[i]

print 'Continue [y]?'
if not m3t.get_yes_no('y'):
	exit()
	
print 'Query to save each calibration [y]?'
do_query=m3t.get_yes_no('y')

for c in comp.keys():
	comp[c]['comp_rt']=mcf.create_component(c.replace('_ec',''))
	comp[c]['comp_j']=mcf.create_component(c.replace('actuator_ec','joint'))
	proxy.subscribe_status(comp[c]['comp_rt'])
	proxy.subscribe_status(comp[c]['comp_j'])
	proxy.publish_param(comp[c]['comp_j'])
proxy.make_operational_all()
proxy.step()

# ###########################
ns=30
for i in range(ns):
	proxy.step()
	print '---------'
	for c in comp.keys():
		tqj=comp[c]['comp_j'].get_torque_mNm()
		tqg=comp[c]['comp_j'].get_torque_gravity_mNm()/1000.0
		tqa=comp[c]['comp_rt'].get_torque_mNm()
		comp[c]['torque_act'].append(tqa)
		comp[c]['torque_joint'].append(tqj)
		comp[c]['torque_gravity'].append(tqg)
		if comp[c]['is_wrist']:
			print c,':joint',tqj,':gravity',tqg,':actuator',tqa
		else:
			print c,':joint',tqj,':gravity',tqg,
	time.sleep(0.05)

# ###########################
for c in comp.keys():
	print '--------',c,'---------'
	if lt==2:
		tqg=0.0
	else:
		tqg=float(na.array(comp[c]['torque_gravity'],na.Float32).mean())
	tqj=float(na.array(comp[c]['torque_joint'],na.Float32).mean())
	tqa=float(na.array(comp[c]['torque_act'],na.Float32).mean())
	if not comp[c]['is_wrist']:
		bias=tqa+tqg
		torque=M3TorqueSensor(comp[c]['comp_rt'].config['calib']['torque']['type'])
		if lt==2:
			print 'Previous zero torque'
			print 'Delta of',bias,'mNm'
		else:
			print 'Measured torque:',tqa,'Torque gravity:', tqg
			print 'Delta of',bias,'mNm'
		comp[c]['comp_rt'].config['calib']['torque']['cb_bias']=comp[c]['comp_rt'].config['calib']['torque']['cb_bias']-bias
		comp[c]['comp_rt'].config['calibration_date']=time.asctime()
		if do_query:
			print 'Save calibration? [y]'
			if m3t.get_yes_no('y'):
				comp[c]['comp_rt'].write_config()
		else:
			comp[c]['comp_rt'].write_config()
	else: 
		print 'Wrist joint...'
		if c.find('j5')!=-1: #do j5/j6 at once
			cc=None
			for x in comp.keys():
				if x.find('j6')!=-1:
					cc=x
			if cc is None:
				print 'Did not find coupled joint to',c
			tqg_c=float(na.array(comp[cc]['torque_gravity'],na.Float32).mean())
			tqj_c=float(na.array(comp[cc]['torque_joint'],na.Float32).mean())
			tqa_c=float(na.array(comp[cc]['torque_act'],na.Float32).mean())
			x=comp[c]['comp_j'].config['transmission']['tqj_to_tqa'][0] #Joint to actuator matrix
			y=comp[c]['comp_j'].config['transmission']['tqj_to_tqa'][1]
			m=comp[cc]['comp_j'].config['transmission']['tqj_to_tqa'][0]
			n=comp[cc]['comp_j'].config['transmission']['tqj_to_tqa'][1]
			tqg_a5= x*tqg+y*tqg_c
			tqg_a6= m*tqg_c+n*tqg
			bias_5=tqa+tqg_a5
			bias_6=tqa_c+tqg_a6
			torque_5=M3TorqueSensor(comp[c]['comp_rt'].config['calib']['torque']['type'])
			torque_6=M3TorqueSensor(comp[cc]['comp_rt'].config['calib']['torque']['type'])
			print '------------'
			print 'J5: Previous joint torque',tqj,'with joint torque gravity', tqg
			print 'J5: Previous actuator torque',tqa,'with actuator torque gravity', tqg_a5
			print 'J5: Actuator delta of',bias_5,'mNm'
			print '------------'
			print 'J6: Previous joint torque',tqj_c,'with joint torque gravity', tqg_c
			print 'J6: Previous actuator torque',tqa_c,'with actuator torque gravity', tqg_a6
			print 'J6: Actuator delta of',bias_6,'mNm'
			print '------------'
			comp[c]['comp_rt'].config['calib']['torque']['cb_bias']=comp[c]['comp_rt'].config['calib']['torque']['cb_bias']-bias_5
			comp[c]['comp_rt'].config['calibration_date']=time.asctime()
			comp[cc]['comp_rt'].config['calib']['torque']['cb_bias']=comp[cc]['comp_rt'].config['calib']['torque']['cb_bias']-bias_6
			comp[cc]['comp_rt'].config['calibration_date']=time.asctime()
			if do_query:
				print 'Save calibration? [y]'
				if m3t.get_yes_no('y'):
					comp[c]['comp_rt'].write_config()
					comp[cc]['comp_rt'].write_config()
			else:
				comp[c]['comp_rt'].write_config()
				comp[cc]['comp_rt'].write_config()
proxy.stop() 
