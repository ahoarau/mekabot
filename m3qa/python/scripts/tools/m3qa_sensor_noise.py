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

print 'All actuator_ec [n]?'
comps=[]
if m3t.get_yes_no('n'):
	cnames=[q for q in cnames if q.find('actuator_ec')!=-1]
	all_aec=True
	for c in cnames:
		comps.append(mcf.create_component(c))
else:
	all_aec=False
	print '------- Components ------'
	for i in range(len(cnames)):
		print i,' : ',cnames[i]
	print '-------------------------'
	print 'Enter component id'
	cid=m3t.get_int()
	name=cnames[cid]
	comps.append(mcf.create_component(name))
if len(comps)==0:
	print 'No components selected'
	exit()

for c in comps:
	proxy.subscribe_status(c)
	proxy.publish_command(c)

field=m3t.user_select_msg_field(comps[0].status)
print 'Number of samples [100]?'
ns=m3t.get_int(100)
log=[]
for i in range(len(comps)):
	log.append([])
try:
	ts=time.time()
	for i in range(ns):
		proxy.step()
		idx=0
		print '---------'
		for c in comps:
			v=m3t.get_msg_field_value(c.status,field)
			log[idx].append(v)
			idx=idx+1
			print i,':',v
		time.sleep(0.05)
except (KeyboardInterrupt,EOFError):
	pass
proxy.stop(force_safeop=False) #allow other clients to continue running
print log
res={}
p2ps=[]
for i in range(len(comps)):
	a=na.array(log[i],na.Float32)
	avg=float(a.mean())
	std=a.std()
	p2p=max(a)-min(a)
	if p2p>0:
		bits=math.log(p2p,2)
	else:
		bits=0
	print '-----------',comps[i].name,field,'--------------'
	print 'Mean:',avg
	print 'Std Dev:',std
	print 'Peak To Peak:',p2p
	print 'Bits',bits
	res[comps[i].name]={'avg':avg,'std':std,'p2p':p2p,'bits':bits}
	p2ps.append(p2p)
	if not all_aec:
		pyl.title('Noise Distribution for '+field)
		pyl.hist(x=a-avg,bins=pyl.array(range(-10,10,1)))
		pyl.show()
if all_aec:
	print '-----------------------'
	print 'Peak to Peak Disribution'
	print p2ps
	print '-------'
	pyl.title('Distribution of P2P noise for '+field)
	pyl.hist(p2ps,bins=range(25))
	pyl.show()
	for i in range(len(comps)):
		print comps[i].name,':',p2ps[i]

proxy.stop()

