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
import m3.gui as m3g
import m3.rt_proxy as m3p
import m3.toolbox as m3t
import m3.component_factory as m3f
import os
import sys
import yaml
import Numeric as nu
import m3.unit_conversion as m3u
from threading import Thread
import numpy as nu
import m3.viz as m3v

# ######################################################

def get_pose(proxy,bot,arm_name, viz):
    select=[]
    bot.set_slew_rate_proportion(arm_name, [1.0]*7)
    if viz==None:
	bot.set_mode_torque_gc(arm_name)
	bot.set_torque(arm_name,[0]*bot.get_num_dof(arm_name))
    else: # For virtual testing on right arm..
	q = []
	if arm_name == 'right_arm':
	    center = [.338,-.223,.190]			
	else:
	    center = [.338,.223,.190]	
	success = bot.get_tool_position_rpy_2_theta_deg(arm_name, center,[-90,0,-90],q)
	bot.set_mode_theta_gc(arm_name)
	bot.set_theta_deg(arm_name, q)
	proxy.step
	time.sleep(0.3)
    proxy.step()
    
    while True:
	print '--------------------------'
	print 'Pose arm'
	print 'Hit enter to set basis pose'
	print '--------------------------'
	os.system("stty raw")
	r = sys.stdin.read(1)
	os.system("stty sane")
	if r=='\r':
	    proxy.step()
	    print 'Basis pose of',bot.get_theta_deg(arm_name)
	    return bot.get_theta_deg(arm_name)

# ######################################################

class ik_thread(Thread):
    def __init__ (self,proxy,bot,step_delta,arm_name,pub, viz):
	Thread.__init__(self)
	self.bot=bot
	self.proxy=proxy
	self.verror=0
	self.aerror=0
	self.delta_done=False
	self.delta=nu.zeros(3) 
	self.target_pos=nu.zeros(3)
	self.pub = pub
	self.target_pos_start = bot.get_tool_position(arm_name)
	self.target_rpy = bot.get_tool_roll_pitch_yaw_deg(arm_name)
	self.step_delta=step_delta
	self.viz = viz
	self.update=True
    def set_delta(self,x):
	self.delta=nu.array(x)
	self.update=True
    def run(self):
	while not self.delta_done:	    
	    self.proxy.step()	 
	    if self.update:
		self.update=False
		self.target_pos=self.target_pos_start+self.delta
		qdes=[]
		success = self.bot.get_tool_position_rpy_2_theta_deg(arm_name, self.target_pos[:], self.target_rpy[:], qdes)
		#success = False
		if success:
		    self.bot.set_theta_deg(arm_name,qdes)
		self.aerror=nu.sqrt(sum((self.target_pos-bot.get_tool_position(arm_name))**2))
		self.bot.set_slew_rate_proportion(arm_name, [0.4]*7)
		self.proxy.step()
		if not self.viz == None:
		    self.viz.step()
	    time.sleep(0.1)
	    
# ######################################################

def run_ik(proxy,bot, step_delta, arm_name,pub, viz):
    pose=get_pose(proxy,bot, arm_name,viz)
    bot.set_mode_theta_gc(arm_name)
    bot.set_stiffness(arm_name, [stiffness]*bot.get_num_dof(arm_name))    
    bot.set_theta_deg(arm_name, pose)
    #bot.set_theta_deg([0,0,0],[4,5,6]) #No roll/pitch/yaw of wrist
    proxy.step()    
    t=ik_thread(proxy,bot,step_delta, arm_name,pub, viz)
    t.start()
    d=[0,0,0]
    while 1:
        print '-----------------------'
	print 'Delta: ',t.delta
	print '-----------------------'
	print 'q: quit'
	print '1: x+'
	print '2: x-'
	print '3: y+'
	print '4: y-'
	print '5: z+'
	print '6: z-'
	print 'space: step'
	k=m3t.get_keystroke()
	if k=='q':
	    t.delta_done=True
	    return
	if k=='1':
	    d[0]=d[0]+step_delta
	if k=='2':
	    d[0]=d[0]-step_delta
	if k=='3':
	    d[1]=d[1]+step_delta
	if k=='4':
	    d[1]=d[1]-step_delta
	if k=='5':
	    d[2]=d[2]+step_delta
	if k=='6':
	    d[2]=d[2]-step_delta
	t.set_delta(d)	
	print
	print 'Error: ',t.aerror
	print 'Target: ',t.target_pos
	
# ######################################################

proxy = m3p.M3RtProxy()
proxy.start()
proxy.make_operational_all()

print '--------------------------'
print 'Note: RVIZ support is only intended for debugging in the demo.'
print '      Motor power should be turned off if using RVIZ.'
print 'Use RVIZ? (y/n)'
print '--------------------------'		
print
k=m3t.get_keystroke()
rviz = False
pub = None
if k == 'y':
    rviz = True

bot_name=m3t.get_robot_name()
if bot_name == "":	
	print 'Error: no botanoid components found:', bot_names	
	proxy.stop()
	sys.exit() 
	
bot=m3f.create_component(bot_name)

viz = None
if rviz == True:
    viz = m3v.M3Viz(proxy, bot)

proxy.publish_param(bot) #allow to set payload
proxy.subscribe_status(bot)
proxy.publish_command(bot)		
proxy.make_operational_all()
if not rviz == True:
    bot.set_motor_power_on()

proxy.step()
if rviz == True:
    viz.step()

stiffness=0.5
step_delta=.002 #meters

print 'Select arm:'		
arm_names = ['right_arm', 'left_arm']		
arm_name = m3t.user_select_components_interactive(arm_names,single=True)[0]

while True:
    proxy.step()
    print '--------------'
    print 's: set stiffness (Current',stiffness,')'
    print 'd: set step delta (Current',step_delta,'(m))'
    print 'e: execute ijkt controller'
    print 'q: quit'
    print '--------------'
    print
    k=m3t.get_keystroke()
    if k=='q':
	break
    if k=='s':
	print 'Enter stiffness (0-1.0) [',stiffness,']'
	stiffness=max(0,min(1.0,m3t.get_float(stiffness)))
    if k=='d':
	print 'Enter step delta (m) [',step_delta,']'
	step_delta=max(0,min(.25,m3t.get_float(step_delta)))
    if k=='e':
	run_ik(proxy,bot, step_delta,arm_name,pub,viz)
proxy.stop()
if rviz==True:
    viz.stop()
	

# ######################################################	


	     
	
	