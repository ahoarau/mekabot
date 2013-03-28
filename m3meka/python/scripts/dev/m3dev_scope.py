#! /usr/bin/python
from pylab import *
import time
import m3.rt_proxy as m3p
import m3.toolbox as m3t
import m3.component_factory as mcf


proxy = m3p.M3RtProxy()
proxy.start()
comps=proxy.get_available_components()
print '------- Components ------'
for i in range(len(comps)):
       print i,' : ',comps[i]
print '-------------------------'
print 'Enter component id'
cid=m3t.get_int()
print 'Select Y-Range? [n]'
yrange=None
if m3t.get_yes_no('n'):
       yrange=[]
       print 'Min?'
       yrange.append(m3t.get_int())
       print 'Max?'
       yrange.append(m3t.get_int())   
name=comps[cid]
comp=mcf.create_component(name)
proxy.subscribe_status(comp)
#proxy.publish_param(comp)
field=m3t.user_select_msg_field(comp.status)
repeated = False
idx = 0
if hasattr(m3t.get_msg_field_value(comp.status,field),'__len__'):
       repeated = True
       print 'Select index of repeated field to monitor: [0]'
       idx = m3t.get_int(0)
              
              
scope=m3t.M3Scope(xwidth=100,yrange=yrange)
try:
       ts=time.time()
       while True:
              proxy.step()
              if repeated:
                     v=m3t.get_msg_field_value(comp.status,field)[idx]
              else:
                     v=m3t.get_msg_field_value(comp.status,field)              
              scope.plot(v)
              #print 'Time: ',60.0-(time.time()-ts),field,':',v
	      print v
              time.sleep(0.1)
              if False:
		     if time.time()-ts>60.0:
			    print 'Continue [y]?'
			    if m3t.get_yes_no('y'):
				   ts=time.time()
			    else:
				   break
except (KeyboardInterrupt,EOFError):
       pass
proxy.stop(force_safeop=False) #allow other clients to continue running

