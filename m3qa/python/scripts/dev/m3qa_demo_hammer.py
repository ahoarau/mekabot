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

import time
import m3.toolbox as m3t
import m3.rt_proxy as m3p
import m3.humanoid 
import m3.hand
import m3.gui as m3g
import m3.trajectory as m3jt
import numpy as nu
import yaml
import math

class M3Proc:
	def __init__(self):
		self.proxy = m3p.M3RtProxy()
		self.gui = m3g.M3Gui(stride_ms=50)
	def stop(self):
		self.proxy.stop()

	def start(self):
		# ######## Setup Proxy and Components #########################
		self.proxy.start()
		bot_name=m3t.get_robot_name()
		if bot_name == "":
			print 'Error: no robot components found:', bot_names
			return
		self.bot=m3.humanoid.M3Humanoid(bot_name)	
		arm_names = self.bot.get_available_chains()	
		arm_names = [x for x in arm_names if x.find('arm')!=-1]
		if len(arm_names)==0:
			print 'No arms found'
			return
		if len(arm_names)==1:
			self.arm_name=arm_names[0]
		else:
			self.arm_name = m3t.user_select_components_interactive(arm_names,single=True)[0]

		# ####### Setup Hand #############
		hand_names=self.proxy.get_available_components('m3hand')
		hand_name=''
		if len(hand_names)>1:
			hand_name=m3t.user_select_components_interactive(chain_names,single=True)
		if len(hand_names)==1:
			hand_name=hand_names[0]
		if len(hand_name):
			self.hand=m3.hand.M3Hand(hand_name)
			self.proxy.publish_command(self.hand)
			self.proxy.subscribe_status(self.hand)
		else:
			self.hand=None
		# ####### Setup Proxy #############
		self.proxy.subscribe_status(self.bot)
		self.proxy.publish_command(self.bot)
		self.proxy.make_operational_all()
		self.bot.set_motor_power_on()
		self.ndof=self.bot.get_num_dof(self.arm_name)
		self.via_traj={}
		self.via_traj_first=True

		# ######## Demo and GUI #########################
		self.off=False
		self.grasp=False
		self.hammer_up=False
		self.switch_start=True
		self.task_mode_names=['Off','Zero','HammerGrasp','Hammer']
		self.arm_mode_methods=[self.step_off,self.step_zero,self.step_hammer_grasp, self.step_hammer]
		self.hand_mode_methods=[self.step_hand_off,self.step_hand_off,self.step_hand_grasp,self.step_hand_grasp]
		self.task_mode=[0]
		self.poses={'zero':  {'right_arm':[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]},
			    'hammer_grasp':{'right_arm':[33, 0, 0, 120.18858337402344, 0, -60,0]},
			    'hammer_up':{'right_arm':[33, 0, 0, 120.18858337402344, 0, -60, 0]},
			    'hammer_down':{'right_arm':[33, 0, 0, 71.249755859375, 0, -60, 0]}}
		
		self.stiffness=[100]
		self.velocity=[25]
		self.cycle_time=[3000]
		self.gui.add('M3GuiModes',  'TaskMode',      (self,'task_mode'),range(1),[self.task_mode_names,1],m3g.M3GuiWrite,column=1)
		if self.hand is not None:
			self.gui.add('M3GuiToggle', 'Grasp', (self,'grasp'),[],[['GraspOpen','GraspClosed']],m3g.M3GuiWrite,column=1)
		self.gui.add('M3GuiSliders','Stiffness ', (self,'stiffness'),[0],[0,100],m3g.M3GuiWrite,column=1) 
		self.gui.add('M3GuiSliders','Velocity ', (self,'velocity'),[0],[0,40],m3g.M3GuiWrite,column=1) 
		self.gui.add('M3GuiSliders','CycleTimeMs', (self,'cycle_time'),[0],[0,4000],m3g.M3GuiWrite,column=1) 
		self.gui.start(self.step)

	def get_task_mode_name(self):
		return self.task_mode_names[self.task_mode[0]]
	def get_stiffness(self):
		return [self.stiffness[0]/100.0]*self.ndof

	def step(self):
		self.proxy.step()
		#print 'Task: ',self.get_task_mode_name()
		apply(self.arm_mode_methods[self.task_mode[0]])
		if self.hand is not None:
			apply(self.hand_mode_methods[self.task_mode[0]])

	def step_hammer(self):
		self.bot.set_mode_theta_gc(self.arm_name)
		self.bot.set_stiffness(self.arm_name,self.get_stiffness())
		self.bot.set_slew_rate_proportion(self.arm_name,[1.0]*7)
		x=math.fmod(time.time(),self.cycle_time[0]/1000.0)/(self.cycle_time[0]/1000.0)
		#print 'X',x
		a=4000*math.sin(math.pi*2*x)
		a=min(a,1000)
		if self.hammer_up:
			if self.switch_start:
				self.ts_switch=time.time()
				self.switch_start=False
			if time.time()-self.ts_switch>self.cycle_time[0]/1000.0:
				self.switch_start=True
				self.hammer_up=False
			self.bot.set_theta_deg(self.arm_name,self.poses['hammer_up'][self.arm_name])
			
			
		if not self.hammer_up:
			if self.switch_start:
				self.ts_switch=time.time()
				self.switch_start=False
			if time.time()-self.ts_switch>self.cycle_time[0]/1000.0:
				self.switch_start=True
				self.hammer_up=True
			self.bot.set_theta_deg(self.arm_name,self.poses['hammer_down'][self.arm_name])
		self.bot.set_mode_torque_gc(self.arm_name,[3])
		self.bot.set_torque(self.arm_name,[a],[3])
		print 'A',a
			
			
	def step_hand_off(self):
		self.hand.set_mode_off()

	def step_hand_grasp(self):
		self.hand.set_stiffness([0.8]*5)
		self.hand.set_slew_rate_proportion([1.0]*5)
		if self.grasp:
			self.hand.set_mode_theta([0])
			self.hand.set_mode_torque_gc([1,2,3,4])
			self.hand.set_torque_mNm([80.0]*4,[1,2,3,4])
			self.hand.set_theta_deg([90.0],[0])
		else:
			self.hand.set_mode_theta([0])
			self.hand.set_mode_theta_gc([1,2,3,4])
			self.hand.set_theta_deg([90.0,0.0,0.0,0.0,0.0])

	def step_hammer_grasp(self):
		self.bot.set_mode_theta_gc_mj(self.arm_name)
		self.bot.set_theta_deg(self.arm_name,self.poses['hammer_grasp'][self.arm_name])
		self.bot.set_stiffness(self.arm_name,self.get_stiffness())
		self.bot.set_thetadot_deg(self.arm_name,[15.0]*self.ndof)

	def step_off(self):
		self.hammer_up=False
		self.switch_start=True
		self.bot.set_mode_off(self.arm_name)

	def step_zero(self):
		self.bot.set_mode_theta_gc_mj(self.arm_name)
		self.bot.set_theta_deg(self.arm_name,self.poses['zero'][self.arm_name])
		self.bot.set_stiffness(self.arm_name,self.get_stiffness())
		self.bot.set_thetadot_deg(self.arm_name,[20.0]*self.ndof)

if __name__ == '__main__':
	t=M3Proc()
	try:
		t.start()
	except (KeyboardInterrupt,EOFError):
		pass
	t.stop()





