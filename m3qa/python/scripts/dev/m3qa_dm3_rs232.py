#! /usr/bin/python


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

import m3.toolbox as m3t
import m3.toolbox_ctrl as m3tc
import time




proxy = m3p.M3RtProxy()
		
proxy.start()
name_ec=m3t.user_select_components_interactive(proxy.get_available_components('m3actuator_ec'),single=True)
if name_ec is None:
	exit()
comp=m3f.create_component(name_ec)
proxy.subscribe_status(comp)
proxy.make_operational(name_ec)
q_log=[]
tq_log=[]
print 'Ready to generate motion? Hit any key to start'
m3t.get_keystroke()
ts=time.time()
try:
	while time.time()-ts>5.0:
		proxy.step()
		q=comp.status.qei_on
		tq=comp.status.adc_torque
		time.sleep(0.25)
		print 'DT',time.time()-ts, 'Q',q,'TQ',tq
		q_log.append(q)
		tq_log.append(tq)
except (KeyboardInterrupt,EOFError):
	proxy.stop()		

poly,inv_poly=m3tc.get_polyfit_to_data(q_log,tq_log,n=1)	
print 'Poly',poly
s=m3t.PolyEval(poly,log_tq)
m3t.mplot2(range(len(log_q)),log_load_mNm,s,xlabel='Samples',ylabel='Torque (mNm)',
			   y1name='loadcell',y2name='actuator')



		
