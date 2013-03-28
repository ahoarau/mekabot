#! /usr/bin/python
# -*- coding: utf-8 -*-

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

# ######################################################	
proxy = m3p.M3RtProxy()
proxy.start()
bot_name=m3t.get_robot_name()
if bot_name == "":
	print 'Error: no robot components found:', bot_name
	exit()
bot=m3f.create_component(bot_name)

hand_names=proxy.get_available_components('m3hand')
hands={}
for h in hand_names:
  hh=m3f.create_component(h)
  hands[m3t.get_chain_limb_name(h)]=hh #right_hand, left_hand
  proxy.subscribe_status(hh)
  proxy.publish_command(hh)
  proxy.publish_param(hh)   

proxy.publish_param(bot) #allow to set payload
proxy.subscribe_status(bot)
proxy.publish_command(bot)
proxy.make_operational_all()
bot.set_motor_power_on()
proxy.step()
chains=bot.get_available_chains()

des={}
postures={'right_hand':{},'right_arm':{},'torso':{}}
for c in chains:
	des[c]=[0.0]*bot.get_num_dof(c)
	postures[c]['zero']=[0.0]*bot.get_num_dof(c)
	
postures['right_hand']['close']=[90.0,80.0,80.0,80.0,80.0]
postures['right_hand']['open']=[30.0,0.0,0.0,0.0,0.0]

postures['right_arm']['stow']=[-26.5, 5.7, 12.0, 129.6, 80.6, -1.5, -7.6]


postures['right_arm']['reach']=[69.8, 13.1, -54.8, 53.0, 123.5, -2.3, 1.1]

postures['right_arm']['bow']=[129.9, 23.4, -43.3, 66.4, 132.7, 29.0, 18.0]
postures['torso']['bow']=[0.0,29.0,29.0]

#Hold torso at zero by default so doesn't fall over
bot.set_mode_theta_gc('torso')
des['torso']=postures['torso']['zero']
#Default hand settings
hands['right_hand'].set_stiffness([1.0]*5)
hands['right_hand'].set_slew_rate_proportion([0.75]*5)


stiffness=0.75
velocity={'torso':5.0,'right_arm':25.0}

print '--------------'
try:
	while True:    
		print '<enter>: pass'
		print 'q: quit'
		print '-------------'
		print 'a: zero'
		print 'b: reach right'
		print 'd: bow'
		print 'e: right hand open'
		print 'f: right hand close'
		print 'o: off'
		print '-------------'
		print 'x: stiffness 1.0'
		print 'y: stiffness 0.75'
		print 'z: stiffness 0.5'
		print '--------------'
		print
		k=m3t.get_keystroke()
		print
		if k=='q':
			break
		if k=='x':
			stiffness=1.0
		if k=='y':
			stiffness=0.75
		if k=='z':
			stiffness=0.5
		if k=='o':
			bot.set_mode_off('torso')
			bot.set_mode_off('right_arm')
			hands['right_hand'].set_mode_off()

			
		if k=='a':
			bot.set_mode_theta_gc_mj('right_arm')
			bot.set_mode_theta_gc_mj('torso')
			des['right_arm']=postures['right_arm']['zero']
			des['torso']=postures['torso']['zero']
			
		if k=='b':
			bot.set_mode_theta_gc_mj('right_arm')
			bot.set_mode_theta_gc_mj('torso')
			des['right_arm']=postures['right_arm']['reach']
			des['torso']=postures['torso']['zero']
			
		if k=='c':
			bot.set_mode_theta_gc_mj('right_arm')
			bot.set_mode_theta_gc_mj('torso')
			des['right_arm']=postures['right_arm']['stow']
			des['torso']=postures['torso']['zero']
			
		if k=='d':
			bot.set_mode_theta_gc_mj('right_arm')
			bot.set_mode_theta_gc_mj('torso')
			des['right_arm']=postures['right_arm']['bow']
			des['torso']=postures['torso']['bow']
		
		if k=='e':
			hands['right_hand'].set_mode_theta_gc()
			hands['right_hand'].set_theta_deg(postures['right_hand']['open'])
			
		if k=='f':
			hands['right_hand'].set_mode_theta_gc([0])
			hands['right_hand'].set_mode_torque_gc([1,2,3,4])
			hands['right_hand'].set_theta_deg(postures['right_hand']['close'][0:1],[0])
			hands['right_hand'].set_torque_mNm(postures['right_hand']['close'][1:],[1,2,3,4])

			
		for c in chains:
			bot.set_thetadot_deg(c, [velocity[c]]*bot.get_num_dof(c))
			bot.set_stiffness(c,[stiffness]*bot.get_num_dof(c))
			bot.set_theta_deg(c,des[c])
		proxy.step()
		time.sleep(0.1)
except (KeyboardInterrupt,EOFError):
	pass

bot.set_mode_off('torso')
bot.set_mode_off('right_arm')
hands['right_hand'].set_mode_off()
proxy.step()
time.sleep(0.5)
proxy.stop()

	


