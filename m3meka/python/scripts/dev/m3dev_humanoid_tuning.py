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
		print '--------------------------'
		print 'Enable RVIZ? (y/n)'
		print '--------------------------'		
		print
		self.rviz = False
		if m3t.get_yes_no():
			self.rviz = True
			
		sea_joint_names=self.proxy.get_joint_components()
		sea_joint_names=m3t.user_select_components_interactive(sea_joint_names)
		
		self.sea_joint=[]
		    
		for n in sea_joint_names:
			self.sea_joint.append(m3f.create_component(n))			
			self.proxy.subscribe_status(self.sea_joint[-1])
		
		chain_names=self.proxy.get_chain_components()
		self.chain=[]
		if len(chain_names)>0:
			print 'Select kinematic chain'
			chain_names=m3t.user_select_components_interactive(chain_names)
			
		
		for n in chain_names:
			self.chain.append(m3f.create_component(n))
			self.proxy.subscribe_status(self.chain[-1])
						
		kine_names=self.proxy.get_available_components('m3dynamatics')
		self.kine = []
		if len(kine_names)>0:
			print 'Select dynamatics controller'
			kine_names=m3t.user_select_components_interactive(kine_names)
			
		for n in kine_names:
			self.kine.append(m3f.create_component(n))			
			self.proxy.subscribe_status(self.kine[-1])
	
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
		self.chain_names = self.bot.get_available_chains()
		#Create gui
		self.mode=[0]
		
		self.theta_desire_right_arm=[0]*self.bot.get_num_dof('right_arm')
		self.theta_desire_left_arm=[0]*self.bot.get_num_dof('left_arm')
		self.theta_desire_torso=[0]*self.bot.get_num_dof('torso')
		self.theta_desire_head=[0]*self.bot.get_num_dof('head')
		self.stiffness_right_arm=[0]*self.bot.get_num_dof('right_arm')
		self.stiffness_left_arm=[0]*self.bot.get_num_dof('left_arm')
		self.stiffness_torso=[0]*self.bot.get_num_dof('torso')
		self.stiffness_head=[0]*self.bot.get_num_dof('head')
		self.stiffness=[0]
		#self.slew=[0]
		self.save=False
		self.save_last=False
		self.status_dict=self.proxy.get_status_dict()
		self.param_dict=self.proxy.get_param_dict()
		self.gui.add('M3GuiTree',   'Status',    (self,'status_dict'),[],[],m3g.M3GuiRead,column=3)
		self.gui.add('M3GuiTree',   'Param',   (self,'param_dict'),[],[],m3g.M3GuiWrite,column=3)
		self.gui.add('M3GuiModes',  'Mode',      (self,'mode'),range(1),[['Off','Pwm','Torque','Theta','Torque_GC','Theta_GC','Theta_MJ', 'Theta_GC_MJ'],1],m3g.M3GuiWrite)
		#self.gui.add('M3GuiSliders','Torque (mNm)', (self,'tq_desire'),range(len(self.bot.right_arm_ndof)),[-8000,8000],m3g.M3GuiWrite)
		#self.gui.add('M3GuiSliders','Pwm', (self,'pwm_desire'),range(len(self.sea_joint)),[-3200,3200],m3g.M3GuiWrite) 
		#self.gui.add('M3GuiSliders','Stiffness ', (self,'stiffness'),range(1),[0,100],m3g.M3GuiWrite,column=1) 
		#self.gui.add('M3GuiSliders','Slew ', (self,'slew'),range(0),[0,100],m3g.M3GuiWrite,column=3) 
		self.gui.add('M3GuiSliders','Theta RA (Deg)', (self,'theta_desire_right_arm'),range(len(self.theta_desire_right_arm)),[-45,140],m3g.M3GuiWrite,column=1)
		self.gui.add('M3GuiSliders','Theta LA (Deg)', (self,'theta_desire_left_arm'),range(len(self.theta_desire_left_arm)),[-45,140],m3g.M3GuiWrite,column=1) 
		self.gui.add('M3GuiSliders','Theta T (Deg)', (self,'theta_desire_torso'),range(len(self.theta_desire_torso)),[-45,140],m3g.M3GuiWrite,column=1) 
		self.gui.add('M3GuiSliders','Theta H (Deg)', (self,'theta_desire_head'),range(len(self.theta_desire_head)),[-45,140],m3g.M3GuiWrite,column=1) 
		self.gui.add('M3GuiSliders','Stiffness RA ', (self,'stiffness_right_arm'),range(len(self.stiffness_right_arm)),[0,100],m3g.M3GuiWrite,column=2)
		self.gui.add('M3GuiSliders','Stiffness LA )', (self,'stiffness_left_arm'),range(len(self.stiffness_left_arm)),[0,100],m3g.M3GuiWrite,column=2) 
		self.gui.add('M3GuiSliders','Stiffness T ', (self,'stiffness_torso'),range(len(self.stiffness_torso)),[0,100],m3g.M3GuiWrite,column=2) 
		
		#self.gui.add('M3GuiSliders','Stiffness ', (self,'stiffness'),range(len(self.sea_joint)),[0,100],m3g.M3GuiWrite,column=3) 
		#self.gui.add('M3GuiToggle', 'Save',      (self,'save'),[],[['On','Off']],m3g.M3GuiWrite)

		self.gui.start(self.step)

	def step(self):		
		self.status_dict=self.proxy.get_status_dict()
		self.proxy.set_param_from_dict(self.param_dict)
				
		for chain in self.chain_names:
			theta = getattr(self, 'theta_desire_' + chain)
			stiff= getattr(self, 'stiffness_' + chain)
			s = [x/100.0 for x in stiff]
			self.bot.set_theta_deg(chain, theta)			
			self.bot.set_mode(chain, [self.mode[0]]*self.bot.get_num_dof(chain))
			self.bot.set_stiffness(chain,s)
			self.bot.set_slew_rate_proportion(chain, [self.slew_rate]*self.bot.get_num_dof(chain))
			#self.bot.set_slew_rate(chain, [20.]*self.bot.get_num_dof(chain))		
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




