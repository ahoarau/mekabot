#! /usr/bin/python
import pylab as pyl
import time
import m3.rt_proxy as m3p
import m3.toolbox as m3t
import m3.component_factory as mcf
import numpy.numarray as na
import math
from m3qa.calibrate_sensors import *

# ###########################
fields=[['adc_ext_temp'],['adc_amp_temp'],['adc_current_a','adc_current_b']]
print 'Select sensor to zero'
for i in range(len(fields)):
	print i,' : ',fields[i]
sidx=m3t.get_int()
sensor=fields[sidx]
print 'This assumes ISS version config files'
print 'All boards should be at room temp'
print 'All motor powers should be off'
print 'Continue [y]?'
if not m3t.get_yes_no('y'):
	exit()
if sensor[0]=='adc_ext_temp' or sensor[0]=='adc_amp_temp':
	print 'Enter room temp (C)'
	ambient=m3t.get_float()

# ###########################
proxy = m3p.M3RtProxy()
proxy.start()

cnames=proxy.get_available_components()
comp_ec=[]
comp_rt=[]
cnames=[q for q in cnames if q.find('actuator_ec')!=-1]
for c in cnames:
	comp_ec.append(mcf.create_component(c))
	comp_rt.append(mcf.create_component(c.replace('_ec','')))
if len(comp_ec)==0:
	print 'No actuator_ec components found'
	exit()
log={}
for c in comp_ec:
	proxy.subscribe_status(c)
	log[c.name]={}
	for f in fields:
		for x in f:
			log[c.name][x]=[]
proxy.step()
time.sleep(0.5)
proxy.step()
# ###########################
ns=30
for i in range(ns):
	proxy.step()
	print '---------'
	for c in comp_ec:
		for s in sensor:
			v=m3t.get_msg_field_value(c.status,s)
			log[c.name][s].append(v)
			print c.name,':',s,':',v
	time.sleep(0.05)
print log
# ###########################
for idx in range(len(comp_ec)):
	print '--------',comp_ec[idx].name,'---------'
	c=comp_ec[idx]
	rt=comp_rt[idx]
	if sensor[0]=='adc_current_a':
		avg_a=float(na.array(log[c.name]['adc_current_a'][6:],na.Float32).mean())
		avg_b=float(na.array(log[c.name]['adc_current_b'][6:],na.Float32).mean())
		current=M3CurrentSensor(rt.config['calib']['current']['type'])
		if rt.config['calib']['current']['type']=='adc_linear_5V' or rt.config['calib']['current']['type']=='adc_linear_5V_ns':
			io=current.raw_2_mA(rt.config['calib']['current'],avg_a,avg_b)
			print 'Old calibrated current',io
			rt.config['calib']['current']['cb_ticks_at_zero_a']=avg_a
			rt.config['calib']['current']['cb_ticks_at_zero_b']=avg_b
			print 'New calibrated current',current.raw_2_mA(rt.config['calib']['current'],avg_a,avg_b)
	if sensor[0]=='adc_ext_temp':
		avg_e=float(na.array(log[c.name]['adc_ext_temp'][6:],na.Float32).mean())
		ext_temp=M3TempSensor(rt.config['calib']['ext_temp']['type'])
		#Zero ext_temp
		if rt.config['calib']['ext_temp']['type'] !='none' and rt.config['calib']['ext_temp']['type']!='temp_25C' :
			cs=ext_temp.raw_2_C(rt.config['calib']['ext_temp'],avg_e)
			print 'Old ext_temp',cs
			rt.config['calib']['ext_temp']['cb_bias']+=ambient-cs
			print 'New ext_temp',ext_temp.raw_2_C(rt.config['calib']['ext_temp'],avg_e)
	if sensor[0]=='adc_amp_temp':
		avg_m=float(na.array(log[c.name]['adc_amp_temp'][6:],na.Float32).mean())
		print 'Type',rt.config['calib']['amp_temp']['type']
		if rt.config['calib']['amp_temp']['type']!='none' and rt.config['calib']['amp_temp']['type']!='temp_25C' :
			amp_temp=M3TempSensor(rt.config['calib']['amp_temp']['type'])
			cs=amp_temp.raw_2_C(rt.config['calib']['amp_temp'],avg_m)
			print 'Old amp_temp',cs
			rt.config['calib']['amp_temp']['cb_bias']+=ambient-cs
			print 'New amp_temp',amp_temp.raw_2_C(rt.config['calib']['amp_temp'],avg_m)
	
print 'Save calibration? [y]'
if m3t.get_yes_no('y'):
	for rt in comp_rt:
		rt.config['calibration_date']=time.asctime()
		rt.write_config()
proxy.stop() 
