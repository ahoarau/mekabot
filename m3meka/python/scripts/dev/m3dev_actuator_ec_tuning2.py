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
import m3.gui as m3g
import m3.rt_proxy as m3p
import m3.toolbox as m3t
import m3.actuator_ec_pb2 as mec
import m3.component_factory as m3f
import math

idx = 0

class M3Proc:
	def __init__(self):
		self.proxy = m3p.M3RtProxy()
		self.gui = m3g.M3Gui(stride_ms=125)
		self.cnt=0
		self.bias=[]
	def stop(self):
		self.proxy.stop()
	def start(self):

		self.proxy.start()
		cnames=self.proxy.get_available_components('m3actuator_ec')
		self.names=m3t.user_select_components_interactive(cnames)
		if len(self.names)==0:
			return
		self.actuator_ec=[]
		for name in self.names:
			self.actuator_ec.append(m3f.create_component(name))
			self.proxy.subscribe_status(self.actuator_ec[-1])
			self.proxy.publish_command(self.actuator_ec[-1]) 
			self.proxy.publish_param(self.actuator_ec[-1]) 
			self.proxy.make_operational(name)
		
		#pwr_ec=self.proxy.get_available_components('m3pwr_ec')
		#pwr_rt=self.proxy.get_available_components('m3pwr')
		#print 'A',pwr_rt[0],pwr_ec[0]
		#if len(pwr_rt):
			#pr=m3f.create_component(pwr_rt[0])
			#self.proxy.publish_command(pr)
			#self.proxy.make_operational(pwr_rt[0])
			#self.proxy.make_operational(pwr_ec[0])
			#pr.set_motor_power_on()
			
		pwr_rt=m3t.get_actuator_ec_pwr_component_name(self.names[0])
		pwr_ec=pwr_rt.replace('m3pwr','m3pwr_ec')
		pr=m3f.create_component(pwr_rt)
		self.proxy.publish_command(pr)
		self.proxy.make_operational(pwr_rt)
		self.proxy.make_operational(pwr_ec)
		self.proxy.subscribe_status(pr)
		pr.set_motor_power_on()
		
		tmax=[x.param.t_max for x in self.actuator_ec]
		tmin=[x.param.t_min for x in self.actuator_ec]
		
		
		self.proxy.step()
		for c in self.actuator_ec:
			self.bias.append(c.status.adc_torque)
		tl=min(tmin)-self.bias[0]
		tu=max(tmax)-self.bias[0]
		
		#Create gui
		self.mode=[0]
		self.pwm_desire=[0]
		self.current_desire=[0]

		self.save=False
		self.save_last=False
		self.do_scope=False
		self.scope = None
		
		self.scope_keys=m3t.get_msg_fields(self.actuator_ec[0].status)
		self.scope_keys.sort()
		self.scope_keys=['None']+self.scope_keys
		self.scope_field1=[0]
		self.scope_field2=[0]

		self.status_dict=self.proxy.get_status_dict()
		self.param_dict=self.proxy.get_param_dict()
		self.gui.add('M3GuiTree',   'Status',    (self,'status_dict'),[],[],m3g.M3GuiRead,column=2)
		self.gui.add('M3GuiTree',   'Param',   (self,'param_dict'),[],[],m3g.M3GuiWrite,column=3)
		self.gui.add('M3GuiModes',  'Mode',      (self,'mode'),range(len(self.actuator_ec)),[['off','pwm','torque','current','brake'],1],m3g.M3GuiWrite)
		self.gui.add('M3GuiSliders','pwmDesire', (self,'pwm_desire'),range(len(self.actuator_ec)),[-1000,1000],m3g.M3GuiWrite) 
		self.gui.add('M3GuiSliders','currentDesire', (self,'current_desire'),range(len(self.actuator_ec)),[-10000,10000],m3g.M3GuiWrite) 
		self.gui.add('M3GuiToggle', 'Save',      (self,'save'),[],[['On','Off']],m3g.M3GuiWrite)
		self.gui.add('M3GuiToggle', 'Scope',      (self,'do_scope'),[],[['On','Off']],m3g.M3GuiWrite)
		self.gui.add('M3GuiModes',  'Scope1',	(self,'scope_field1'),range(1),[self.scope_keys,1],m3g.M3GuiWrite)
		self.gui.add('M3GuiModes',  'Scope2',	(self,'scope_field2'),range(1),[self.scope_keys,1],m3g.M3GuiWrite)
	
		self.gui.start(self.step)

	
	
	def step(self):
		self.proxy.step()

		for c in self.actuator_ec:
		
			if self.do_scope and self.scope is None:
				self.scope=m3t.M3Scope2(xwidth=100,yrange=None)

			self.status_dict=self.proxy.get_status_dict()
			self.proxy.set_param_from_dict(self.param_dict)
			
			c.command.mode=int(self.mode[idx])
			if self.mode[idx]==mec.ACTUATOR_EC_MODE_PWM:
				c.command.pwm_desired=int(self.pwm_desire[idx])
				print 'Desired',c.name,c.command.pwm_desired
			if self.mode[idx]==mec.ACTUATOR_EC_MODE_CURRENT:
				c.command.current_desired=int(self.current_desire[idx]) 
				print 'Desired',c.name,c.command.current_desired
				
			
			if self.do_scope and self.scope is not None:
				f1=self.scope_keys[self.scope_field1[0]]
				f2=self.scope_keys[self.scope_field2[0]]
				x1=x2=None
				if f1!='None' and f1!='base':
					x1=m3t.get_msg_field_value(self.actuator_ec[0].status,f1)
					print f1,':',x1
				if f2!='None' and f2!='base':
					x2=m3t.get_msg_field_value(self.actuator_ec[0].status,f2)   
					print f2,':',x2
				if x1==None:
					x1=x2
				if x2==None:
					x2=x1
				if x1!=None and x2!=None: #Handle only one value or two
					self.scope.plot(x1,x2)
					print'-----------------'
		

			if (self.save and not self.save_last):
				c.write_config()

			self.save_last=self.save

		
if __name__ == '__main__':
	t=M3Proc()
	try:
		t.start()
	except (KeyboardInterrupt,EOFError):
		pass
	t.stop()



