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
		pr.set_motor_power_on()
		
		tmax=[x.param.t_max for x in self.actuator_ec]
		tmin=[x.param.t_min for x in self.actuator_ec]
		
		
		self.proxy.step()
		for c in self.actuator_ec:
			self.bias.append(c.status.adc_torque)
		tl=min(tmin)-self.bias[0]
		tu=max(tmax)-self.bias[0]
		
		self.cycle_pwm=False
		self.cycle_last_pwm=False
		self.cycle_tq=False
		self.cycle_last_tq=False
		self.step_period=[2000.0]*len(self.actuator_ec)
		self.brake=[0]
		#Create gui
		self.mode=[0]*len(self.actuator_ec)
		self.t_desire_a=[0]*len(self.actuator_ec)
		self.t_desire_b=[0]*len(self.actuator_ec)
		self.pwm_desire_a=[0]*len(self.actuator_ec)
		self.pwm_desire_b=[0]*len(self.actuator_ec)
		self.current_desire_a=[0]*len(self.actuator_ec)
		self.current_desire_b=[0]*len(self.actuator_ec)
		self.save=False
		self.save_last=False
		self.do_scope_torque=False
		self.scope_torque=None
		self.status_dict=self.proxy.get_status_dict()
		self.param_dict=self.proxy.get_param_dict()
		self.gui.add('M3GuiTree',   'Status',    (self,'status_dict'),[],[],m3g.M3GuiRead,column=2)
		self.gui.add('M3GuiTree',   'Param',   (self,'param_dict'),[],[],m3g.M3GuiWrite,column=3)
		self.gui.add('M3GuiModes',  'Mode',      (self,'mode'),range(len(self.actuator_ec)),[['Off','Pwm','PID','CURRENT'],1],m3g.M3GuiWrite)
		self.gui.add('M3GuiModes',  'Brake',      (self,'brake'),range(1),[['Enabled','Disabled'],1],m3g.M3GuiWrite)
		self.gui.add('M3GuiSliders','tqDesire',  (self,'t_desire_a'),range(len(self.actuator_ec)),[tl,tu],m3g.M3GuiWrite)
		self.gui.add('M3GuiSliders','tqDesire',  (self,'t_desire_b'),range(len(self.actuator_ec)),[tl,tu],m3g.M3GuiWrite)
		self.gui.add('M3GuiSliders','pwmDesireA', (self,'pwm_desire_a'),range(len(self.actuator_ec)),[-3200,3200],m3g.M3GuiWrite) 
		self.gui.add('M3GuiSliders','pwmDesireB', (self,'pwm_desire_b'),range(len(self.actuator_ec)),[-3200,3200],m3g.M3GuiWrite)
		self.gui.add('M3GuiSliders','currentDesireA', (self,'current_desire_a'),range(len(self.actuator_ec)),[-100,100],m3g.M3GuiWrite) 
		self.gui.add('M3GuiSliders','currentDesireB', (self,'current_desire_b'),range(len(self.actuator_ec)),[-3200,3200],m3g.M3GuiWrite) 
		self.gui.add('M3GuiSliders','StepPeriod (ms) ', (self,'step_period'),range(len(self.actuator_ec)),[0,4000],m3g.M3GuiWrite) 
		self.gui.add('M3GuiToggle', 'CyclePwm',      (self,'cycle_pwm'),[],[['On','Off']],m3g.M3GuiWrite)	
		self.gui.add('M3GuiToggle', 'CycleTq',      (self,'cycle_tq'),[],[['On','Off']],m3g.M3GuiWrite)	
		self.gui.add('M3GuiToggle', 'Save',      (self,'save'),[],[['On','Off']],m3g.M3GuiWrite)
		self.gui.add('M3GuiToggle', 'Scope',      (self,'do_scope_torque'),[],[['On','Off']],m3g.M3GuiWrite)
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
		if self.do_scope_torque and self.scope_torque is None and len(self.actuator_ec)==1:
			self.scope_torque=m3t.M3Scope2(xwidth=100,yrange=None)
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
			
			if not self.cycle_last_pwm and self.cycle_pwm:
				self.step_start=time.time()
			if not self.cycle_last_tq and self.cycle_tq:
				self.step_start=time.time()
			self.cycle_last_pwm=self.cycle_pwm
			self.cycle_last_tq=self.cycle_tq
			pwm=self.pwm_desire_a[idx]
			tq=self.t_desire_a[idx]
			current=self.current_desire_a[idx]
			if self.cycle_pwm:
				dt=time.time()-self.step_start
				if math.fmod(dt,self.step_period[idx]/1000.0)>self.step_period[idx]/2000.0:
					pwm=self.pwm_desire_b[idx]
			if self.cycle_tq:
				dt=time.time()-self.step_start
				if math.fmod(dt,self.step_period[idx]/1000.0)>self.step_period[idx]/2000.0:
					tq=self.t_desire_b[idx]
			c.command.mode=int(self.mode[idx])
			if self.mode[idx]==mec.ACTUATOR_EC_MODE_PWM:
				c.command.t_desire=int(pwm)
			if self.mode[idx]==mec.ACTUATOR_EC_MODE_TORQUE:
				c.command.t_desire=int(tq+self.bias[idx]) #Bias slider around 'zero'
				print 'Desired',c.name,c.command.t_desire
			if self.mode[idx]==mec.ACTUATOR_EC_MODE_CURRENT:
				c.command.t_desire=int(current) 
				print 'Desired',c.name,c.command.t_desire
				
			
			if self.do_scope_torque and self.scope_torque is not None:
			  if self.mode[idx]==mec.ACTUATOR_EC_MODE_TORQUE:
				self.scope_torque.plot(c.status.adc_torque,c.command.t_desire)
			  else:
				self.scope_torque.plot(c.status.adc_torque,c.status.adc_torque)
			idx=idx+1
			if (self.save and not self.save_last):
				c.write_config()
			c.command.brake_off=int(self.brake[0])
			print 't_desire:', c.command.t_desire
		self.save_last=self.save
		
if __name__ == '__main__':
	t=M3Proc()
	try:
		t.start()
	except (KeyboardInterrupt,EOFError):
		pass
	t.stop()



