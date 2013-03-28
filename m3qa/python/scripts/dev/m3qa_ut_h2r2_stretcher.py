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
import m3.component_factory as m3f
import Numeric as nu
import math

# ######################################################	
proxy = m3p.M3RtProxy()
proxy.start()
proxy.make_operational_all()
chain_names=proxy.get_available_components('m3hand')
if len(chain_names)>1:
    hand_name=m3t.user_select_components_interactive(chain_names,single=True)
else:
    hand_name=chain_names
pwr_name=proxy.get_available_components('m3pwr')[0]
hand=m3f.create_component(hand_name[0])
proxy.publish_command(hand)
proxy.subscribe_status(hand)

pwr=m3f.create_component(pwr_name)
proxy.publish_command(pwr)
pwr.set_motor_power_on()

hand.set_mode_torque()
print 'Enter duration (s) [30.0]'
d=m3t.get_float(30.0)
ts=time.time()
amp=nu.array([0.0,200.0,200.0,200.0,300.0])
period=nu.array([3,3.1,2.8,3.2,3.0])
nc=0
print 'Power up system. Hit enter to start'
raw_input()
try:
	while time.time()-ts<d:
		nc=nc+1
		t=time.time()-ts
		if math.fmod(nc,10)==0:
			print '---------------------------------------------'
			print 'Time: ',t,'/',d
		proxy.step()
		x=amp*nu.sin(2*nu.pi*t/period)
		if math.fmod(nc,10)==0:
		    print x
		hand.set_torque_mNm(x)
		time.sleep(0.01)
except (KeyboardInterrupt,EOFError):
	pass
hand.set_mode_off()
proxy.step()
proxy.stop()
	
	
	
	