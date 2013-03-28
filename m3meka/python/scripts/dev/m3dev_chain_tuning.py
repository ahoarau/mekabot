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

import m3.gui as m3g
import m3.rt_proxy as m3p
import m3.joint as m3
import m3.toolbox as m3t
import m3.unit_conversion as m3u
import m3.pwr
import m3.component_factory as m3f
import numpy as nu
import m3.viz as m3v
import time


class M3Proc:
	def __init__(self):
		self.proxy = m3p.M3RtProxy()
		self.gui = m3g.M3Gui(stride_ms=125)#125
		self.slew_rate = 0.5

	def stop(self):
		if self.rviz:
			self.viz.stop()
		self.proxy.stop()

	def start(self):
		self.proxy.start()
		print 'Enable RVIZ [n]?'
		self.rviz = False
		if m3t.get_yes_no('n'):
			self.rviz = True
			
		chain_names=self.proxy.get_chain_components()
		self.chain=[]
		if len(chain_names)>0:
			print 'Select kinematic chain'
			self.chain_names=m3t.user_select_components_interactive(chain_names,single=True)
		self.chain.append(m3f.create_component(self.chain_names[0]))
		self.proxy.subscribe_status(self.chain[-1])
		self.limb=m3t.get_chain_limb_name(self.chain_names[0])
		
		joint_names=m3t.get_chain_joint_names(self.chain_names[0])
		print 'Select joint'
		joint_names=m3t.user_select_components_interactive(joint_names,single=True)
		self.joint=[]
		self.actuator=[]
		self.actuator_ec=[]
		acutator_names=[]
		for n in joint_names:
			self.joint.append(m3f.create_component(n))			
			actuator_name = m3t.get_joint_actuator_component_name(n)
			actuator_ec_name = m3t.get_actuator_ec_component_name(actuator_name)
			self.actuator.append(m3f.create_component(actuator_name))
			self.actuator_ec.append(m3f.create_component(actuator_ec_name))
			self.proxy.subscribe_status(self.joint[-1])
			self.proxy.publish_param(self.joint[-1])
			self.proxy.subscribe_status(self.actuator[-1])
			self.proxy.publish_param(self.actuator[-1])
			if self.actuator_ec[0] is not None:
				self.proxy.subscribe_status(self.actuator_ec[-1])
				self.proxy.publish_param(self.actuator_ec[-1])
		
		
		#kine_names=self.proxy.get_available_components('m3dynamatics')
		self.kine = []
		#if len(kine_names)>0:
			#print 'Select dynamatics controller'
			#kine_names=m3t.user_select_components_interactive(kine_names)
			
		#for n in kine_names:
			#self.kine.append(m3f.create_component(n))			
			#self.proxy.subscribe_status(self.kine[-1])
	
		bot_name=m3t.get_robot_name()
		if bot_name == "":
			print 'Error: no robot components found:', bot_names
			return
		self.bot=m3f.create_component(bot_name)
				
		if self.rviz:			
			self.viz = m3v.M3Viz(self.proxy, self.bot)			
		
		#self.proxy.publish_param(self.bot) #allow to set payload
		#self.proxy.subscribe_status(self.bot)
		#self.proxy.publish_command(self.bot)		
		#self.proxy.make_operational_all()
		self.bot.initialize(self.proxy)
		#self.chain_names = self.bot.get_available_chains()
		#Create gui
		self.mode=[0]
		self.posture=[0]
		self.theta_desire_a=[0]*self.bot.get_num_dof(self.limb)
		self.theta_desire_b=[0]*self.bot.get_num_dof(self.limb)
		self.stiffness=[50.0]*self.bot.get_num_dof(self.limb)
		self.thetadot=[10.0]*self.bot.get_num_dof(self.limb)
		
		#print 'Selected: ',self.chain_names[0],self.limb,self.bot.get_num_dof(self.limb)
		
		#self.slew=[0]
		self.save=False
		self.save_last=False
		self.status_dict=self.proxy.get_status_dict()
		self.param_dict=self.proxy.get_param_dict()
		self.gui.add('M3GuiModes',  'Mode',      (self,'mode'),range(1),[['Off','Pwm','Torque','Theta','Torque_GC','Theta_GC','Theta_MJ', 'Theta_GC_MJ'],1],m3g.M3GuiWrite)
		self.gui.add('M3GuiTree',   'Status',    (self,'status_dict'),[],[],m3g.M3GuiRead,column=1)
		self.gui.add('M3GuiTree',   'Param',   (self,'param_dict'),[],[],m3g.M3GuiWrite,column=1)
		
		self.gui.add('M3GuiSliders','ThetaA (Deg)', (self,'theta_desire_a'),range(len(self.theta_desire_a)),[-45,140],m3g.M3GuiWrite,column=2)
		self.gui.add('M3GuiModes',  'Posture',      (self,'posture'),range(1),[['A','B','Cycle'],1],m3g.M3GuiWrite,column=2)
		self.gui.add('M3GuiSliders','ThetaB (Deg)', (self,'theta_desire_b'),range(len(self.theta_desire_b)),[-45,140],m3g.M3GuiWrite,column=2)
		
		self.gui.add('M3GuiSliders','Stiffness ', (self,'stiffness'),range(len(self.stiffness)),[0,100],m3g.M3GuiWrite,column=3)
		self.gui.add('M3GuiToggle', 'Save',      (self,'save'),[],[['On','Off']],m3g.M3GuiWrite,column=3)
		self.gui.add('M3GuiSliders','ThetaDot ', (self,'thetadot'),range(len(self.thetadot)),[0,100],m3g.M3GuiWrite,column=3)
		
		#self.gui.add('M3GuiSliders','Stiffness ', (self,'stiffness'),range(len(self.sea_joint)),[0,100],m3g.M3GuiWrite,column=3) 
		

		self.gui.start(self.step)

	def step(self):		
		self.status_dict=self.proxy.get_status_dict()
		self.proxy.set_param_from_dict(self.param_dict)
		if (self.save and not self.save_last):
			self.actuator[0].write_config()
			self.joint[0].write_config()
			if self.actuator_ec[0] is not None:
				self.actuator_ec[0].write_config()
		self.save_last=self.save
	

		s = [x/100.0 for x in self.stiffness]
		if self.posture[0]==0:
			self.bot.set_theta_deg(self.limb, self.theta_desire_a)			
		if self.posture[0]==1:
			self.bot.set_theta_deg(self.limb, self.theta_desire_b)
		if self.posture[0]==2:
			if time.time()%10.0>5.0:
				self.bot.set_theta_deg(self.limb, self.theta_desire_a)
			else:
				self.bot.set_theta_deg(self.limb, self.theta_desire_b)
		self.bot.set_mode(self.limb, [self.mode[0]]*self.bot.get_num_dof(self.limb))
		self.bot.set_stiffness(self.limb,s)
		self.bot.set_slew_rate_proportion(self.limb, [self.slew_rate]*self.bot.get_num_dof(self.limb))
		self.bot.set_thetadot_deg(self.limb, self.thetadot)	
		
		self.proxy.step()
		if self.rviz:		
			self.viz.step()
if __name__ == '__main__':
	t=M3Proc()
	try:
		t.start()
	except (KeyboardInterrupt,EOFError):
		pass
	t.stop()




