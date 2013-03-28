#! /usr/bin/python
# -*- coding: utf-8 -*-

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
import m3.hand as m3h
import m3.toolbox as m3t
import m3.unit_conversion as m3u
import m3.component_factory as m3f
import Numeric as nu
import m3.rt_proxy as m3p
import yaml

class M3Proc:
    def __init__(self):
        self.proxy = m3p.M3RtProxy()
        self.gui = m3g.M3Gui()

    def stop(self):
        self.proxy.stop()
    def start(self):
        self.proxy.start()
        self.proxy.make_operational_all()

        chain_names=self.proxy.get_available_components('m3hand')
        if len(chain_names)>1:
            hand_name=m3t.user_select_components_interactive(chain_names,single=True)
        else:
            hand_name=chain_names
        pwr_name=self.proxy.get_available_components('m3pwr')
	if len(pwr_name)>1:
            pwr_name=m3t.user_select_components_interactive(pwr_name,single=True)

	print 'Position arm [y]?'
	if m3t.get_yes_no('y'):
		arm_names=self.proxy.get_available_components('m3arm')
		if len(arm_names)>0:
		  if len(arm_names)>1:
			  print 'Select arm: '	
			  arm_name=m3t.user_select_components_interactive(arm_names,single=True)[0]
		  else:
			  arm_name=arm_names[0]
		  self.arm=m3f.create_component(arm_name)
		  self.proxy.publish_command(self.arm)
		  self.arm.set_mode_theta_gc()
		  self.arm.set_theta_deg([30,0,0,110,0,0,0])
		  self.arm.set_stiffness(0.5)
		  self.arm.set_slew_rate_proportion([0.75]*7)
		
        self.chain=m3f.create_component(hand_name[0])
        self.proxy.publish_command(self.chain)
        self.proxy.subscribe_status(self.chain)

        self.pwr=m3f.create_component(pwr_name[0])
        self.proxy.publish_command(self.pwr)
        self.pwr.set_motor_power_on()

 	#Force safe-op of robot if present
        hum=self.proxy.get_available_components('m3humanoid')
        if len(hum)>0:
            self.proxy.make_safe_operational(hum[0])
	
        #Setup postures
	self.posture_filename=m3t.get_m3_animation_path()+self.chain.name+'_postures.yml'
	f=file(self.posture_filename,'r')
	self.data= yaml.safe_load(f.read())
	self.param=self.data['param']
	f.close()
	self.theta_desire=[0,0,0,0,0]
	self.mode=[1,1,1,1,1]
	
        #Create gui
        self.run=False
        self.run_last=False
        self.running=False
	self.grasp=False
	self.grasp_last=False
	self.grasp_off=False
	self.grasp_off_ts=time.time()
        self.status_dict=self.proxy.get_status_dict()
        self.gui.add('M3GuiTree',   'Status',    (self,'status_dict'),[],[],m3g.M3GuiRead,column=3)
        self.gui.add('M3GuiTree',   'Param',   (self,'param'),[],[],m3g.M3GuiWrite,column=3)
        self.gui.add('M3GuiToggle', 'Animation',      (self,'run'),[],[['Run','Stop']],m3g.M3GuiWrite,column=1)
	self.gui.add('M3GuiModes',  'Joint',      (self,'mode'),range(5),[['Off','Enabled'],1],m3g.M3GuiWrite,column=2)	
	self.gui.add('M3GuiSliders','Theta (Deg)', (self,'theta_desire'),range(5),[0,300],m3g.M3GuiWrite,column=2) 
	self.gui.add('M3GuiToggle', 'Power Grasp', (self,'grasp'),[],[['Run','Stop']],m3g.M3GuiWrite,column=2)
        self.gui.start(self.step)

    def step(self):
        self.proxy.step()
        self.status_dict=self.proxy.get_status_dict()
        self.chain.set_stiffness(self.param['stiffness'])
	self.chain.set_slew_rate_proportion(self.param['q_slew_rate'])
	#Do power Grasp
	if self.grasp and not self.grasp_last:
	    if self.mode[0]:
		self.chain.set_mode_theta_gc([0])
	    if self.mode[1]:
		self.chain.set_mode_torque_gc([1])
	    if self.mode[2]:
		self.chain.set_mode_torque_gc([2])
	    if self.mode[3]:
		self.chain.set_mode_torque_gc([3])
	    if self.mode[4]:
		self.chain.set_mode_torque_gc([4])
	    self.chain.set_theta_deg(self.chain.get_theta_deg())
	    self.chain.set_torque_mNm(self.param['grasp_torque'])
	self.grasp_last=self.grasp
	
	#Do joint theta control
	if not self.grasp and not self.running: #theta open
	    for jidx in range(5):
		    if self.mode[jidx]:
			self.chain.set_mode_theta_gc(jidx)
		    else:
			self.chain.set_mode_off(jidx)    
	
	#Start Animation
        if not self.run_last and self.run and not self.running:
            self.running=True
	    for jidx in range(5):
		    if self.mode[jidx]:
			    self.chain.set_mode_theta_gc(jidx)
		    else:
			    self.chain.set_mode_off(jidx)
	    self.pose_idx=0
	    self.ts_anim=time.time()
	
	if self.running:
	    self.chain.set_theta_deg(self.data['postures'][self.pose_idx])
	    if time.time()-self.ts_anim>self.param['pose_time']:
		    self.ts_anim=time.time()
		    self.pose_idx=self.pose_idx+1
		    if self.pose_idx>=len(self.data['postures']):
			    self.running=False
			    print 'Animation done'
			    
        if not self.running:
	    self.chain.set_theta_deg(self.theta_desire)
	self.run_last=self.run
		    
if __name__ == '__main__':
    t=M3Proc()
    try:
        t.start()
    except (KeyboardInterrupt,EOFError):
        pass
    t.stop()



