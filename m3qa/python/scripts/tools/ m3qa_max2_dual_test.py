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
		pwr_ec=self.proxy.get_available_components('m3pwr_ec')
		pwr_rt=self.proxy.get_available_components('m3pwr')
		
		print 'Select two ACTUATOR_EC components'
		self.names=m3t.user_select_components_interactive(cnames)
		if len(self.names)!=2:
			print 'Incorrect selection'
			return
		self.actuator_ec=[]
		for name in self.names:
			self.actuator_ec.append(m3f.create_component(name))
			self.proxy.subscribe_status(self.actuator_ec[-1])
			self.proxy.publish_command(self.actuator_ec[-1]) 
			self.proxy.publish_param(self.actuator_ec[-1]) 
			self.proxy.make_operational(name)

		if len(pwr_rt):
			pr=m3f.create_component(pwr_rt[0])
			self.proxy.publish_command(pr)
			self.proxy.make_operational(pwr_rt[0])
			self.proxy.make_operational(pwr_ec[0])
			pr.set_motor_power_on()
		
		tmax=[x.param.t_max for x in self.actuator_ec]
		tmin=[x.param.t_min for x in self.actuator_ec]
		
		
		self.proxy.step()
		for c in self.actuator_ec:
			self.bias.append(c.status.adc_torque)
		tl=min(tmin)-self.bias[0]
		tu=max(tmax)-self.bias[0]
		
		self.cycle=False
		self.cycle_last=False
		self.step_period=[2000.0]*len(self.actuator_ec)
	
		#Create gui
		self.mode=[0]*len(self.actuator_ec)
		self.t_desire=[0]*len(self.actuator_ec)
		self.pwm_desire_a=[0]*len(self.actuator_ec)
		self.pwm_desire_b=[0]*len(self.actuator_ec)
		self.save=False
		self.save_last=False
		self.status_dict=self.proxy.get_status_dict()
		self.param_dict=self.proxy.get_param_dict()
		self.gui.add('M3GuiTree',   'Status',    (self,'status_dict'),[],[],m3g.M3GuiRead,column=2)
		self.gui.add('M3GuiTree',   'Param',   (self,'param_dict'),[],[],m3g.M3GuiWrite,column=3)
		self.gui.add('M3GuiModes',  'Mode',      (self,'mode'),range(len(self.actuator_ec)),[['Off','Pwm','PID'],1],m3g.M3GuiWrite)
		self.gui.add('M3GuiSliders','tqDesire',  (self,'t_desire'),range(len(self.actuator_ec)),[tl,tu],m3g.M3GuiWrite)
		self.gui.add('M3GuiSliders','pwmDesireA', (self,'pwm_desire_a'),range(len(self.actuator_ec)),[-3200,3200],m3g.M3GuiWrite) 
		self.gui.add('M3GuiSliders','pwmDesireB', (self,'pwm_desire_b'),range(len(self.actuator_ec)),[-3200,3200],m3g.M3GuiWrite) 
		self.gui.add('M3GuiSliders','StepPeriod (ms) ', (self,'step_period'),range(len(self.actuator_ec)),[0,4000],m3g.M3GuiWrite) 
		self.gui.add('M3GuiToggle', 'Cycle',      (self,'cycle'),[],[['On','Off']],m3g.M3GuiWrite)	
		self.gui.add('M3GuiToggle', 'Save',      (self,'save'),[],[['On','Off']],m3g.M3GuiWrite)
		self.gui.start(self.step)

	def get_theta_raw_deg(self,c): #12bit MA3
		try:
			e= int((c.status.qei_on*4097)/c.status.qei_period)-1
		except ZeroDivisionError:
			e= 0
		scale=0.087890625
		e=e*scale
		return e
	
	def step(self):
		self.proxy.step()
		if False and self.cnt%5==0:
			for n in self.names:
				self.proxy.pretty_print_component(n)	
		if False and self.cnt%5==0:
			print '---------------'
			for c in self.actuator_ec:
				print 'Timestamp',c.name,m3t.timestamp_string(c.status.timestamp)
		self.cnt=self.cnt+1
		self.status_dict=self.proxy.get_status_dict()
		self.proxy.set_param_from_dict(self.param_dict)
		idx=0
		for c in self.actuator_ec:
			
			if not self.cycle_last and self.cycle:
				self.step_start=time.time()
			self.cycle_last=self.cycle
	    
			pwm=self.pwm_desire_a[idx]
			if self.cycle:
				dt=time.time()-self.step_start
				if math.fmod(dt,self.step_period[idx]/1000.0)>self.step_period[idx]/2000.0:
					pwm=self.pwm_desire_b[idx]
		    
			c.command.mode=int(self.mode[idx])
			if self.mode[idx]==mec.ACTUATOR_EC_MODE_PWM:
				c.command.t_desire=int(pwm)
			if self.mode[idx]==mec.ACTUATOR_EC_MODE_TORQUE:
				c.command.t_desire=int(self.t_desire[idx]+self.bias[idx]) #Bias slider around 'zero'
				print 'Desired',c.name,c.command.t_desire
			idx=idx+1
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



