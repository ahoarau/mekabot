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
import m3.toolbox as m3t
import m3.unit_conversion as m3u
import m3.component_factory as m3f
import numpy as nu
import m3.toolbox_head_s2 as m3th
import m3.head_s2csp_ctrl as m3csp
import math
import random
import time

class M3Proc:
	def __init__(self):
		self.proxy = m3p.M3RtProxy()
		self.gui = m3g.M3Gui(stride_ms=125)#125
		
	def stop(self):
		self.proxy.stop()

	def start(self):
		self.proxy.start()
		bot_name=m3t.get_robot_name()
		if bot_name == "":
			print 'Error: no robot components found:', bot_names
			return
		self.bot=m3f.create_component(bot_name)	
		self.bot.initialize(self.proxy)
		csp_name=self.proxy.get_available_components('m3head_s2csp_ctrl')
		if len(csp_name)!=1:
			print 'CSP component not available'
			exit()
		self.csp_rt=m3csp.M3HeadS2CSPCtrl(csp_name[0])
		self.proxy.subscribe_status(self.csp_rt)
		self.proxy.publish_command(self.csp_rt)
		self.proxy.publish_param(self.csp_rt)
		self.mode=[0]
		self.x=[5]
		self.y=[0]
		self.z=[0]
		self.theta_j2=[0]
		self.ts_rand=0
		self.joints=range(7)
		self.save=False
		self.save_last=False
		self.status_dict=self.proxy.get_status_dict()
		self.param_dict=self.proxy.get_param_dict()
		self.gui.add('M3GuiTree',   'Status',    (self,'status_dict'),[],[],m3g.M3GuiRead,column=1)
		self.gui.add('M3GuiTree',   'Param',   (self,'param_dict'),[],[],m3g.M3GuiWrite,column=1)
		self.gui.add('M3GuiModes',  'Mode',      (self,'mode'),range(1),[['Off','CSP','CSPRandom'],1],m3g.M3GuiWrite)
		self.gui.add('M3GuiSliders','x', (self,'x'),[0],[-10,10],m3g.M3GuiWrite,column=1) 
		self.gui.add('M3GuiSliders','y', (self,'y'),[0],[-10,10],m3g.M3GuiWrite,column=1) 
		self.gui.add('M3GuiSliders','z', (self,'z'),[0],[-10,10],m3g.M3GuiWrite,column=1) 
		self.gui.add('M3GuiSliders','Head Roll (Deg)', (self,'theta_j2'),[0],[-15,15],m3g.M3GuiWrite,column=1) 
		self.gui.add('M3GuiToggle', 'Save',      (self,'save'),[],[['On','Off']],m3g.M3GuiWrite)
		self.gui.start(self.step)
		
	def step(self):		
		if (self.save and not self.save_last):
			self.csp_rt.write_config()
		self.save_last=self.save
		self.status_dict=self.proxy.get_status_dict()
		self.proxy.set_param_from_dict(self.param_dict)

		#Define target on sphere 
		#lat=0, long=0 corresponds to [1,0,0]
		#lat=90, long=0 corresponds to [0,0,1]
		#lat=0, long=90 corresponds to [0,1,0]
		r=1.0
#		if self.mode[0]==1:
#			x = self.x[0]
#			y = self.y[0]
#			z = self.z[0]
		
		if self.mode[0]==2:
			if time.time()-self.ts_rand>6.0:
				self.x[0]=2*(random.random()-0.5)*10.0
				self.y[0]=2*(random.random()-0.5)*10.0
				self.z[0]=2*(random.random()-0.5)*10.0
				self.ts_rand=time.time()
		
		target = [self.x[0],self.y[0],self.z[0]]
		
		print 'Target',target
		
		if self.mode[0]==0:
			self.bot.set_mode_off('head')
			self.csp_rt.disable()
		else:
			self.csp_rt.enable()
			self.csp_rt.set_target_csp_frame(target)
			self.csp_rt.set_theta_j2_deg(self.theta_j2[0])
		self.proxy.step()

if __name__ == '__main__':
	t=M3Proc()
	try:
		t.start()
	except (KeyboardInterrupt,EOFError):
		pass
	t.stop()




