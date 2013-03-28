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
import os
import sys
import yaml
import m3.trajectory as m3jt
import numpy as nu
import m3.gui as m3g
import m3.toolbox as m3t
import m3.viz as m3v

# #####################################################
def load_vias():
	via_files=m3t.get_via_files()
	via_names=m3t.get_via_names()
	if len(via_files)==0:
		print 'No via files available'
		return
	print 'Enter file IDX'
	print '-----------------------'
	for i in range(len(via_names)):
		print i,' : ',via_names[i]
	idx=m3t.get_int(0)
	if idx>=0 and idx<len(via_names):
		f=file(via_files[idx],'r')
		d=yaml.safe_load(f.read())
		return d
	return None
					
# ######################################################

print 'Enable RVIZ? [n]'
pub=None
rviz = m3t.get_yes_no('n')
	
# ######################################################
proxy = m3p.M3RtProxy()
proxy.start()
bot_name=m3t.get_robot_name()
if bot_name == "":
	print 'Error: no robot components found:', bot_name
	quit
bot=m3f.create_component(bot_name)

if rviz == True:
	viz = m3v.M3Viz(proxy, bot)			

proxy.subscribe_status(bot)
proxy.publish_command(bot)
proxy.make_operational_all()
bot.set_motor_power_on()
proxy.step()

if rviz == True:
	viz.step()

print 'Select chains:'		
chains = m3t.user_select_components_interactive(bot.get_available_chains(),single=False)
# ######################################################
#Hardcode defaults for now...should move to config file
stiffness={
	'right_arm':[
		0.5, #J0
		0.5, #J1
		0.5, #J2
		0.5, #J3
		0.5, #J4
		0.5, #J5
		0.5],#J6
	'left_arm':[
		0.5, #J0
		0.5, #J1
		0.5, #J2
		0.5, #J3
		0.5, #J4
		0.5, #J5
		0.5],#J6
	'torso':[
		0.65, #J0
		0.65, #J1
		0.65]}#J2
#Deg/S
vel_avg={
	'right_arm':[
		40.0, #J0
		40.0, #J1
		40.0, #J2
		40.0, #J3
		40.0, #J4
		40.0, #J5
		40.0],#J6
	'left_arm':[
		40.0, #J0
		40.0, #J1
		40.0, #J2
		40.0, #J3
		40.0, #J4
		40.0, #J5
		40.0],#J6
	'torso':[
		5.0, #J0
		5.0, #J1
		5.0]}#J2
scale_stiffness={'right_arm': 1.0,'left_arm':1.0,'torso':1.0}
scale_vel_avg={'right_arm': 1.0,'left_arm':1.0,'torso':1.0}
use_theta_gc=True
# ######################################################


vias={}
for c in chains:
	vias[c]=[]

while True:    
	proxy.step()
	print '--------------'
	#print 'p: execute splined vias (python)'
	print 'e: execute trajectory (rt)'
	#print 'm: execute minimum jerk'
	#print 'd: execute direct THETA_GC mode'
	print 'l: load via file'
	print 'd: display vias'
	print 'v: scale avg velocity'
	print 's: set stiffness'
	print 'm: set control mode'
	print 'q: quit'
	print '--------------'
	print
	k=m3t.get_keystroke()
	print
	if k=='q':
		break
	if k=='r':
		vias=record_vias()
	if k=='l':
		v=load_vias()
		for c in chains:
			vias[c]=v[c]['postures']
	if k=='m':
		
		print 'Use mode THETA_GC [y]?'
		use_theta_gc= m3t.get_yes_no('y')
	if k=='d':
		print '-------- Mode ---------'
		if use_theta_gc:
			print 'Currently in mode: THETA_GC'
		else:
			print 'Currently in mode: THETA'
		print '------ Scaling --------'
		print 'Stiffness: ',stiffness
		print 'Vel avg: ',scale_vel_avg
		print '--------- Vias --------'
		print vias
	if k=='v':
		print 'Select chain'
		c = m3t.user_select_components_interactive(chains,single=True)[0]
		print 'Current scale for',c,': ',scale_vel_avg[c]
		print 'New scale: (0-2.0)'
		scale_vel_avg[c]=max(0,min(2.0,m3t.get_float()))
	if k=='s':
		
		print 'Current stiffness'
		print '--------------------'
		for c in chains:
			print c,' : ',stiffness[c]
		print
		print 'Select chain'
		c = m3t.user_select_components_interactive(chains,single=True)[0]
		print 'Enter stiffness: '
		s=max(0,min(1.0,m3t.get_float()))
		stiffness[c]=[s]*len(stiffness[c])
		print 'New stiffness: ',c,' : ',stiffness[c]
	if k=='e':
		use_chain={}
		done=True
		for c in chains:
			print 'Use ',c,'[y]?'
			use_chain[c]= m3t.get_yes_no('y')
			if use_chain[c] and len(vias[c]):
				ndof=bot.get_num_dof(c)
				done=False
				for v in vias[c]:
					print 'Adding via',v,'for',c
					va=nu.array(vel_avg[c])*scale_vel_avg[c]
					bot.add_splined_traj_via_deg(c,v,va)
				#bot.add_splined_traj_via_deg(c,[0.0]*ndof,va) #return home
				#print 'Adding via',[0.0]*ndof,'for',c
				if use_theta_gc:
					bot.set_mode_splined_traj_gc(c)
					ss=[max(0.0,min(1.0,scale_stiffness[c]*x)) for x in stiffness[c]]
					bot.set_stiffness(c,ss)
				else:
					bot.set_mode_splined_traj(c)
				bot.set_slew_rate_proportion(c, [1.0]*ndof)
		ts=time.time()
		
		print 'Hit enter when ready...'
		raw_input()
		while not done:
			done=True
			for c in chains:
				if use_chain[c] and not bot.is_splined_traj_complete(c):
					done=False
			print 'Running...',time.time()-ts
			proxy.step()
			if rviz == True:
				viz.step()
			time.sleep(0.1)
		time.sleep(1.0)
		for c in chains:
			bot.set_mode_off(c)
		proxy.step()
	if k=='p':
		if len(vias):     
			pass
			#ndof=bot.get_num_dof(arm_name)
			#jt = m3jt.JointTrajectory(ndof)
			#for v in vias:
				#print 'Adding via',v            
				#jt.add_via_deg(v, [vel_avg]*ndof)
			#print 'Adding via',[0.0]*ndof
			#jt.add_via_deg([0.0]*ndof, [vel_avg]*ndof)            
			#bot.set_motor_power_on()
			#bot.set_mode_theta_gc(arm_name)
			#bot.set_stiffness(arm_name,[stiffness]*ndof)			
			#ts=time.time()
			#jt.start([0]*ndof, [0]*ndof)
			#print 'hit any key to start!'
			#z=m3t.get_keystroke()
			#while not jt.is_splined_traj_complete():								
				#q = jt.step()
				#print q
				#bot.set_theta_deg(arm_name, q)
				#proxy.step()				
				#ros_publish()
			#print 'Running...',time.time()-ts
			#time.sleep(0.1)
			#proxy.pretty_print_component('m3sea_wrist_ma2_j6')#chain.name)
proxy.stop()
if rviz == True:
	viz.stop()
# ######################################################	




