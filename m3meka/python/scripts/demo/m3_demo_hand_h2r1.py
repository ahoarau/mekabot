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
        pwr_name=self.proxy.get_available_components('m3pwr')[0]

	print 'Position arm [y]?'
	if m3t.get_yes_no('y'):
		arm_names=self.proxy.get_available_components('m3arm')
		if len(arm_names)>1:
			print 'Select arm: '	
			arm_name=m3t.user_select_components_interactive(arm_names,single=True)[0]
		else:
			arm_name=arm_names[0]
		self.arm=m3f.create_component(arm_name)
		self.proxy.publish_command(self.arm)
		self.arm.set_mode_theta_gc()
		self.arm.set_theta_deg([30,0,0,110,0,0,0])
		self.arm.set_stiffness(0.6)
		self.arm.set_slew_rate_proportion([0.75]*7)
		
        self.chain=m3f.create_component(hand_name[0])
        self.proxy.publish_command(self.chain)
        self.proxy.subscribe_status(self.chain)

        self.pwr=m3f.create_component(pwr_name)
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
	f.close()
	self.max_num_pose=min(12,len(self.data['postures'].keys()))
        self.posture_sel=range(self.max_num_pose)
	self.theta_desire=[0,0,0,0,0]
	self.mode=[1,1,1,1,1]
	
        #Create gui
        self.run=False
        self.run_last=False
        self.running=False
	self.record=False
	self.record_last=False
	self.grasp=False
	self.grasp_last=False
	self.grasp_off=False
	self.write=False
	self.write_last=False
	self.grasp_off_ts=time.time()
        self.status_dict=self.proxy.get_status_dict()
        self.gui.add('M3GuiTree',   'Status',    (self,'status_dict'),[],[],m3g.M3GuiRead,column=3)
        self.gui.add('M3GuiTree',   'Data',   (self,'data'),[],[],m3g.M3GuiWrite,column=3)
        self.gui.add('M3GuiModes',  'Posture', (self,'posture_sel'),range(self.max_num_pose),[self.data['postures'].keys(),1],m3g.M3GuiWrite)
        self.gui.add('M3GuiToggle', 'Animator',      (self,'run'),[],[['Run','Stop']],m3g.M3GuiWrite,column=1)
	self.gui.add('M3GuiModes',  'Joint',      (self,'mode'),range(5),[['Off','Enabled'],1],m3g.M3GuiWrite,column=2)	
	self.gui.add('M3GuiSliders','Theta (Deg)', (self,'theta_desire'),range(5),[0,300],m3g.M3GuiWrite,column=2) 
	self.gui.add('M3GuiToggle', 'Record Pose', (self,'record'),[],[['Run','Stop']],m3g.M3GuiWrite,column=2)
	self.gui.add('M3GuiToggle', 'Power Grasp', (self,'grasp'),[],[['Run','Stop']],m3g.M3GuiWrite,column=2)
	self.gui.add('M3GuiToggle', 'Save', (self,'write'),[],[['Run','Stop']],m3g.M3GuiWrite,column=2)
        self.gui.start(self.step)

    def step(self):
        self.proxy.step()
        self.status_dict=self.proxy.get_status_dict()
        self.chain.set_stiffness(self.data['param']['stiffness'])
	self.chain.set_slew_rate_proportion(self.data['param']['q_slew_rate'])
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
	    self.chain.set_torque_mNm(self.data['param']['grasp_torque'])
	
    	
	#if False: #theta open
	#if not self.grasp and self.grasp_last: #force open
		#self.grasp_off=True
		#self.chain.set_torque_mNm(self.data['param']['grasp_torque_off'])
		#self.grasp_off_ts=time.time()
	
	#if self.grasp_off and time.time()-self.grasp_off_ts>2.0: #Open
		#self.grasp_off=False
	self.grasp_last=self.grasp
	
	#Do joint theta control
	#if not self.grasp and not self.grasp_off and not self.running: #force open
	if not self.grasp and not self.running: #theta open
	    for jidx in range(5):
		    if self.mode[jidx]:
			if jidx==0:
			    self.chain.set_mode_theta(jidx)
			else:
			    self.chain.set_mode_theta_gc(jidx)
		    else:
			self.chain.set_mode_off(jidx)
		    
	#Record posture
	if self.record and not self.record_last and not self.grasp and not self.grasp_off:
	    print 'Enter pose name: (q to quit)'
	    while True:
		name=m3t.get_string()
		if self.data['postures'].has_key(name):
		    print 'Pose ',name,'already exists. Overwrite? '
		    if m3t.get_yes_no():
			break
		else:
		    break
	    if name!='q' and name !='Q':
		p=self.chain.get_theta_deg()
		self.data['postures'][name]=[float(x) for x in p]
		self.data['thetadot_avg'][name]=[100,100,100,100,100]
		print 'Posture',name,': ',p
	    else:
		print 'Record aborted'
	self.record_last=self.record
	    
	if self.write and not self.write_last:
	    print 'Writing ',self.posture_filename
	    f=file(self.posture_filename,'w')
	    f.write(yaml.safe_dump(self.data,width=200))
	    f.close()
	self.write_last=self.write
	
	#Run Animator
        if not self.run_last and self.run and not self.running:
            self.running=True
            self.pose_idx=-1
            self.end_time=0
	    self.start_time=0
	    if True:
		for jidx in range(5):
			if self.mode[jidx]:
			    if jidx==0:
				self.chain.set_mode_splined_traj(jidx)
			    else:
				self.chain.set_mode_splined_traj_gc(jidx)
			else:
				self.chain.set_mode_off(jidx)
		np=max(0,min(self.data['param']['num_execution_postures'],self.max_num_pose))
		for pidx in range(int(np)):
			pose_name=self.data['postures'].keys()[self.posture_sel[pidx]]
			theta_des=self.data['postures'][pose_name]
			thetadot_avg=self.data['thetadot_avg'][pose_name]
			self.chain.add_splined_traj_via_deg(theta_des,thetadot_avg)
			self.chain.add_splined_traj_via_deg(self.data['param']['posture_return'],
							self.data['param']['posture_return_speed'])
	    if False:
		for jidx in range(5):
			if self.mode[jidx]:
				self.chain.set_mode_theta_gc(jidx)
			else:
				self.chain.set_mode_off(jidx)
		self.pose_idx=0
		self.ts_anim=time.time()
		
	if False: #self.running:
		pose_name=self.data['postures'].keys()[self.posture_sel[self.pose_idx]]
		theta_des=self.data['postures'][pose_name]
		self.chain.set_theta_deg(theta_des)
		if time.time()-self.ts_anim>3.0:
			self.ts_anim=time.time()
			np=max(0,min(self.data['param']['num_execution_postures'],self.max_num_pose))
			self.pose_idx=self.pose_idx+1
			if self.pose_idx>=np:
				self.pose_idx=-1
				self.running=False
			print 'Pose',self.pose_idx
	if self.running:
	    if self.chain.is_splined_traj_complete():
		self.running=False
		print 'Splined traj. complete.'
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



