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
from threading import Thread
import yaml

# ######################################################

class menu_thread(Thread):
    def __init__ (self,bot,chains):
	Thread.__init__(self)
	self.bot=bot
	self.done=False
	self.chains=chains
    def run(self):
	vias={}
	for c in self.chains:
		vias[c]=[]
	while not self.done:
		print '--------------'
		print 'r: record via'
		print 's: save via file'
		print 'p: print current pose'
		print 'q: quit'
		print '--------------'
		print
		k=m3t.get_keystroke()
		print 'Dbg',dbg
		if k=='r':
			print '-------------'
			for c in self.chains:
				vias[c].append(m3t.float_list(self.bot.get_theta_deg(c)))
				print 'Record of: ',vias[c][-1],'for',c
		if k=='p':
			print '----------------'
			for c in self.chains:
				print 'Chain:',c,' : ',self.bot.get_theta_deg(c)
			print '----------------'
		if k=='s':
			bot_name=m3t.get_robot_name()
			fn=bot_name+'_'+m3t.time_string()
			print 'Enter via file name [',fn,']'
			fn=m3t.get_string(fn)
			fn=m3t.get_m3_animation_path()+fn+'.via'
			print 'Writing file: ',fn
			f=file(fn,'w')
			d={}
			for c in self.chains:
			    ndof=self.bot.get_num_dof(c)
			    if c=='torso':
				param={'slew':[1.0]*ndof,'stiffness':[0.8]*ndof,'velocity':[25.0,15.0]}
			    else:
				param={'slew':[1.0]*ndof,'stiffness':[0.4]*ndof,'velocity':[25.0]*ndof}
			    d[c]={'postures':vias[c],'param':param} #safe defaults
			f.write(yaml.safe_dump(d))
			vias={}
			for c in self.chains:
				vias[c]=[]
		if k=='q':
			self.done=True




# ######################################################	

proxy = m3p.M3RtProxy()
proxy.start()
bot_name=m3t.get_robot_name()
if bot_name == "":
	print 'Error: no robot components found:', bot_name
	exit()
bot=m3f.create_component(bot_name)
proxy.publish_param(bot) #allow to set payload
proxy.subscribe_status(bot)
proxy.publish_command(bot)
proxy.make_operational_all()
bot.set_motor_power_on()
proxy.step()

chains=bot.get_available_chains()
print 'Select chains to pose'
chains=m3t.user_select_components_interactive(chains)
menu=menu_thread(bot,chains)
menu.start()


slew={}
qdes={}
fn=m3t.get_m3_animation_path()+m3t.get_robot_name()+'_poser_config.yml'
f=file(fn,'r')
config= yaml.safe_load(f.read())
f.close()
stiffness=config['stiffness']
delta_thresh=config['delta_thresh']
slew_rate=config['slew_rate']
proxy.step()
dbg=''
for c in chains:
	ndof=bot.get_num_dof(c)
	slew[c]=[]
	qdes[c]=[]
	for i in range(ndof):
		slew[c].append(m3t.M3Slew())
		qdes[c]=nu.array(bot.get_theta_deg(c),nu.Float)
	bot.set_mode_theta_gc(c)
	bot.set_theta_deg(c,qdes[c])
	bot.set_stiffness(c,stiffness[c])	
	bot.set_slew_rate_proportion(c,[1.0]*bot.get_num_dof(c))
try:
	while not menu.done:
		proxy.step()
		for c in chains:
			q=bot.get_theta_deg(c)
			ndof=bot.get_num_dof(c)
			delta=q-qdes[c]
			s=[0.0]*ndof
			for i in range(ndof):
				if abs(delta[i])>delta_thresh[c][i]: #deg
					qdes[c][i]=q[i]
				s[i]=qdes[c][i]#slew[c][i].step(qdes[c][i],slew_rate)
			bot.set_theta_deg(c,s)
		time.sleep(0.1)
except (KeyboardInterrupt,EOFError):
	pass

for c in chains:
	bot.set_mode_off(c)
proxy.step()
print 'Stopping poser...'
proxy.stop()

	
	
	
	