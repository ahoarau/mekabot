#! /usr/bin/python

#*************************************************************************
# 
# REDWOOD CONFIDENTIAL
# Author: Aaron Edsinger
# __________________
# 
#  [2012] - [+] Redwood Robotics Incorporated 
#  All Rights Reserved.
# 
# All information contained herein is, and remains
# the property of Redwood Robotics Incorporated and its suppliers,
# if any.  The intellectual and technical concepts contained
# herein are proprietary to Redwood Robotics Incorporated
# and its suppliers and may be covered by U.S. and Foreign Patents,
# patents in process, and are protected by trade secret or copyright law.
# Dissemination of this information or reproduction of this material
# is strictly forbidden unless prior written permission is obtained
# from Redwood Robotics Incorporated.
#


import time
import math
import glob 

import m3.gui as m3g
import m3.rt_proxy as m3p
import m3.toolbox as m3t
import m3.component_factory as m3f

import m3.ctrl_simple_pb2 as mec
import m3.ctrl_simple as m3cs
import m3.actuator_ec as m3aec
import m3.actuator as m3a
import m3.pwr as m3power
from m3dev_tuning import M3Tuning

class M3Proc(M3Tuning):
#	comps = {'act': {'name': 'm3actuator_ma15_j0', 'type':m3a.M3Actuator }, 
#			'act_ec': {'name': 'm3actuator_ec_ma15_j0', 'type':m3aec.M3ActuatorEc},
#			'ctrl': {'name': 'm3ctrl_simple_ma15_j0', 'type':m3cs.M3CtrlSimple}}#,
#			'pwr': {'name': 'm3pwr_ec_pwr026', 'type':m3power.M3Pwr}}



	def __init__(self):
		M3Tuning.__init__(self)
		self.proxy = m3p.M3RtProxy()
		self.gui = m3g.M3Gui(stride_ms=125)
		self.cnt=0
		self.bias=[]
		
	def stop(self):
		self.ctrl.set_mode_off()
		self.proxy.step()
		self.proxy.stop()
		
	def start(self):
		self.proxy.start()

		self.get_component('m3actuator')
		print "starting components"
		self.start_components(['ctrl','act','act_ec'],None)
		print "done starting components"
		

#		for k,v in self.comps.items():
#			# accomplishes this: self.act=m3s.M3Actuator(self.comp_name)
#			setattr(self, k, v['type'](v['name']) )
#			self.comps[k]['comp'] = getattr(self,k)
#			self.proxy.subscribe_status(getattr(self,k))
#			self.proxy.publish_command(getattr(self,k)) 
#			self.proxy.publish_param(getattr(self,k)) 
#			self.proxy.make_operational(v['name'])
#			
		pwr_rt=m3t.get_actuator_ec_pwr_component_name(self.comps['act_ec']['name'])
		pwr_ec=pwr_rt.replace('m3pwr','m3pwr_ec')
		pr=m3f.create_component(pwr_rt)
		self.proxy.publish_command(pr)
		self.proxy.make_operational(pwr_rt)
		self.proxy.make_operational(pwr_ec)
		pr.set_motor_power_on()
			

		self.proxy.step()

		#Create gui
		self.mode = [0]
		self.traj = [0]
		
		self.current	= [0]
		self.theta	= [0]
		self.torque	= [0]
		self.torque_lc	= [0]
		
		self.save		= False
		self.save_last	= False
		self.do_scope	= False
		self.scope		= None
		
		# extract status fields
		self.status_dict = self.proxy.get_status_dict()
		
		self.ctrl_scope_keys	= m3t.get_msg_fields(self.ctrl.status,prefix='',exclude=['ethercat','base'])
		self.ctrl_scope_keys.sort()
		self.ctrl_scope_keys	= ['None']+self.ctrl_scope_keys

		print self.ctrl_scope_keys

		self.ctrl_scope_field1	= [0]
		self.ctrl_scope_field2	= [0]
		self.ec_scope_field1	= [0]

		current_max = 3.5
		theta_max	= 100.0
		torque_max = 40.0

		self.param_dict=self.proxy.get_param_dict()
		
		print str(self.param_dict)

		self.modes = mec._CTRL_SIMPLE_MODE.values_by_number
		self.mode_names = [self.modes[i].name for i in sorted(self.modes.keys())]
		
		self.trajs = mec._CTRL_SIMPLE_TRAJ_MODE.values_by_number
		self.traj_names = [self.trajs[i].name for i in sorted(self.trajs.keys())]

		self.gui.add('M3GuiTree',   'Status',	(self,'status_dict'),[],[],m3g.M3GuiRead,column=2)
		self.gui.add('M3GuiTree',   'Param',	(self,'param_dict'),[],[],m3g.M3GuiWrite,column=3)
		
		self.gui.add('M3GuiModes',  'Mode',		(self,'mode'),range(1),[self.mode_names,1],m3g.M3GuiWrite)
																			
		self.gui.add('M3GuiModes',  'Traj',		(self,'traj'),range(1),[self.traj_names,1],m3g.M3GuiWrite)
#		self.gui.add('M3GuiModes',  'Mode',		(self,'mode'),range(1),[['Off','Current','Theta','Torque','Torque_LC'],1],m3g.M3GuiWrite)
																			
#		self.gui.add('M3GuiModes',  'Traj',		(self,'traj'),range(1),[['Off','Current Square','Current Sine','Theta Square','Theta Sine','Torque Square','Torque Sine','TorqueLC Square','TorqueLC Sine'],1],m3g.M3GuiWrite)
		
		self.gui.add('M3GuiSliders','Current',		(self,'current'),range(1),	[-current_max,current_max],m3g.M3GuiWrite)
		self.gui.add('M3GuiSliders','Theta(deg)',	(self,'theta'),range(1),[-theta_max,theta_max],m3g.M3GuiWrite)
		self.gui.add('M3GuiSliders','Torque',		(self,'torque'),range(1),[-torque_max,torque_max],m3g.M3GuiWrite)
		self.gui.add('M3GuiSliders','TorqueLC',		(self,'torque_lc'),range(1),[-torque_max,torque_max],m3g.M3GuiWrite)
		
		self.gui.add('M3GuiToggle', 'Save',			(self,'save'),[],[['On','Off']],m3g.M3GuiWrite)

		self.gui.add('M3GuiModes',  'Ctrl Scope1',	(self,'ctrl_scope_field1'),range(1),[self.ctrl_scope_keys,1],m3g.M3GuiWrite)
		self.gui.add('M3GuiModes',  'Ctrl Scope2',	(self,'ctrl_scope_field2'),range(1),[self.ctrl_scope_keys,1],m3g.M3GuiWrite)
		self.gui.add('M3GuiToggle', 'Scope',		(self,'do_scope'),[],[['On','Off']],m3g.M3GuiWrite)
		
		self.gui.start(self.step)


		
	def step(self):


		if self.do_scope and self.scope is None:
			self.scope = m3t.M3Scope2(xwidth=100,yrange=None)
			
		self.proxy.step()
		self.cnt = self.cnt+1
		self.status_dict = self.proxy.get_status_dict()

		self.proxy.set_param_from_dict(self.param_dict)

		if self.do_scope and self.scope is not None:
			ctrl_f1	= self.ctrl_scope_keys[self.ctrl_scope_field1[0]]
			ctrl_f2	= self.ctrl_scope_keys[self.ctrl_scope_field2[0]]
			
			x1=x2=None

			if ctrl_f1 != 'None':
				x1 = m3t.get_msg_field_value(self.ctrl.status,ctrl_f1)
				print ctrl_f1,':',x1

			if ctrl_f2 != 'None':
				x2 = m3t.get_msg_field_value(self.ctrl.status,ctrl_f2)
				print ctrl_f2,':',x2

			if x1==None:
				x1=x2

			if x2==None:
				x2=x1

			if x1!=None and x2!=None: #Handle only one value or two
				self.scope.plot(x1,x2)
				print'-----------------'

		
		self.ctrl.set_traj_mode(self.traj[0])
			

		if self.mode[0]==0:
			self.ctrl.set_mode_off()
			
#		elif self.mode[0]==1:
#			self.ctrl.set_mode_pwm()
#			self.ctrl.set_pwm(self.current[0])

		elif self.mode[0]==1:		
			self.ctrl.set_mode_current()
			self.ctrl.set_current(self.current[0])
			
		elif self.mode[0]==mec.CTRL_MODE_THETA:
			self.ctrl.set_mode_theta()
			self.ctrl.set_theta_deg(self.theta[0])
#			
		elif self.mode[0]==mec.CTRL_MODE_TORQUE:
			print "kp: " + str(self.ctrl.param.pid_torque.k_p)
			self.ctrl.set_mode_torque()
			self.ctrl.set_torque(self.torque[0])
#			
#		elif self.mode[0]==mec.CTRL_MODE_TORQUE_LC:
#			self.ctrl.set_mode_torque_lc()
#			self.ctrl.set_torque_lc(self.torque_lc[0])
		else:
			self.ctrl.set_mode_off()
		
		
		if (self.save and not self.save_last):
			self.ctrl.write_config()

		self.save_last = self.save


if __name__ == '__main__':
	t=M3Proc()
	try:
		t.start()
	except (KeyboardInterrupt,EOFError):
		pass
	t.stop()



