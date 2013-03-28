#! /usr/bin/python
import pylab as pyl
import time
import m3.rt_proxy as m3p
import m3.toolbox as m3t
import m3.component_factory as mcf
import numpy.numarray as na
import math

proxy = m3p.M3RtProxy()
proxy.start()
cnames=proxy.get_available_components()
cnames=[q for q in cnames if q.find('actuator_ec')!=-1]
cnames=[q for q in cnames if  q.find('mh')==-1]
comps=[]
for c in cnames:
	comps.append(mcf.create_component(c))
	proxy.subscribe_status(comps[-1])
last_period=[]
last_rollover=[]
offset_period=[]
offset_rollover=[]
for i in range(len(comps)):
	last_period.append(0)
	last_rollover.append(0)
	offset_period.append(0)
	offset_rollover.append(0)
try:
	ts=time.time()
	terr_q=0
	terr_tq=0
	while True:
		proxy.step()
		idx=0
		for c in comps:
			v=c.status.qei_rollover
			if v!=last_rollover[idx]:
				if v<last_rollover[idx]:
					offset_rollover[idx]=offset_rollover[idx]+32768
					v=v+offset_rollover[idx]
				terr_q=terr_q+(v-last_rollover[idx])
				print 'Error Theta',c.name,':',
			last_rollover[idx]=v
			v=c.status.qei_period
			if v!=last_period[idx]:
				if v<last_period[idx]:
					offset_period[idx]=offset_period[idx]+32768
					v=v+offset_period[idx]
				terr_tq=terr_tq+(v-last_period[idx])
				print 'Error Torque',c.name,':',v
			last_period[idx]=v
			idx=idx+1
		time.sleep(0.1)
		dt=time.time()-ts
		print 'DT: %3.2f'%(time.time()-ts),'Torque errors: ',terr_tq,' | Theta errors: ',terr_q
except (KeyboardInterrupt,EOFError):
		pass
proxy.stop(force_safeop=False)