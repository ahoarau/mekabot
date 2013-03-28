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
import m3.humanoid 
import math

# ######################################################	
proxy = m3p.M3RtProxy()
proxy.start()
bot_name=m3t.get_robot_name()
if bot_name == "":
	print 'Error: no robot components found:', bot_name
bot=m3f.create_component(bot_name)
proxy.publish_param(bot) #allow to set payload
proxy.subscribe_status(bot)
proxy.publish_command(bot)
proxy.make_operational_all()
bot.set_motor_power_on()
chains=bot.get_available_chains()
print 'Select chain'
chains=m3t.user_select_components_interactive(chains,single=False)
for c in chains:
	ndof=bot.get_num_dof(c)
	bot.set_mode_torque(c)
print 'Enter duration (s) [30.0]'
d=m3t.get_float(30.0)
ts=time.time()
amp=nu.array([5000.0,5000.0,2500.0,2500.0,0.0,0.0,0.0])
period=nu.array([3,3.1,2.8,3.2,3.0,3.0,3.0])
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
		for c in chains:
			x=amp*nu.sin(2*nu.pi*t/period)
			if math.fmod(nc,10)==0:
				print c,':',x
			bot.set_torque(c,x)
		time.sleep(0.01)
except (KeyboardInterrupt,EOFError):
	pass
for c in chains:
	bot.set_mode_off(c)
proxy.step()
proxy.stop()
	
	
	
	