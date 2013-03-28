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

#import matplotlib
#matplotlib.use('TkAgg')
import time
import m3.gui as m3g
import m3.rt_proxy as m3p
import m3.toolbox as m3t
import m3.actuator_pb2 as mec

import m3.component_factory as m3f
import math
import glob
from m3dev_tuning import M3Tuning


class M3Proc(M3Tuning):

	def __init__(self):
		M3Tuning.__init__(self)
		self.proxy = m3p.M3RtProxy()
		self.gui = m3g.M3Gui(stride_ms=125)
		self.cnt=0
		self.bias=[]
		
	def stop(self):
		self.act.set_mode(0)
		self.proxy.step()
		self.proxy.stop()
		
	def start(self):
	
		self.proxy.start()
		
		self.get_component('m3actuator')
		print "starting components"
		self.start_components(['act','act_ec','pwr'],None)
		print "done starting components"
		
#		for k,v in self.comps.items():
#			# accomplishes this: self.act=m3s.M3Actuator(self.comp_name)
#			setattr(self, k, v['type'](v['name']) )
#			self.comps[k]['comp'] = getattr(self,k)
#			self.proxy.subscribe_status(getattr(self,k))
#			self.proxy.publish_command(getattr(self,k)) 
#			self.proxy.publish_param(getattr(self,k)) 
#			self.proxy.make_operational(v['name'])
			
#		pwr_rt=m3t.get_actuator_ec_pwr_component_name(self.comps['act_ec']['name'])
#		pwr_ec=pwr_rt.replace('m3pwr','m3pwr_ec')
#		pr=m3f.create_component(pwr_rt)
#		self.proxy.publish_command(pr)
#		self.proxy.make_operational(pwr_rt)
#		self.proxy.make_operational(pwr_ec)
#		pr.set_motor_power_on()
			

		self.proxy.step()

		#Create gui
		self.mode				= [0]
		
		self.current_desired	= [0]
		self.pwm_desired		= [0]
		

		#self.enable_ctrl_dev=[0]
		self.save		= False
		self.save_last	= False
		self.do_scope	= False
		self.scope		= None
		self.status_dict=self.proxy.get_status_dict()
		
		#extract status fields
		self.scope_keys=m3t.get_msg_fields(self.act.status,prefix='',exclude=['ethercat','base'])
		self.scope_keys.sort()
		self.scope_keys		= ['None']+self.scope_keys
		self.scope_field1	= [0]
		self.scope_field2	= [0]
		

		self.f1_last	= None
		self.f2_last	= None
		
		self.zero_motor_theta			= False
		self.zero_motor_theta_last		= False
		self.zero_joint_torque			= False
		self.zero_joint_torque_last		= False
		self.zero_joint_torque_lc		= False
		self.zero_joint_torque_lc_last	= False
		self.zero_joint_theta			= False
		self.zero_joint_theta_last		= False
		
		current_max = 2.5
		pwm_max = 200

		self.param_dict = self.proxy.get_param_dict()
#		self.joint_torque	= self.param_dict[self.comps['act']['name']]['calib']['torque']['cb_bias']
#		self.joint_theta	= self.param_dict[self.comps['act']['name']]['calib']['theta']['cb_bias']
	
		
		self.gui.add('M3GuiTree',   'Status',		(self,'status_dict'),[],[],m3g.M3GuiRead,column=2)
		self.gui.add('M3GuiTree',   'Param',		(self,'param_dict'),[],[],m3g.M3GuiWrite,column=3)

		self.gui.add('M3GuiModes',  'Mode',			(self,'mode'),range(1),[['Off','PWM','Current'],1],m3g.M3GuiWrite)

		self.gui.add('M3GuiSliders','PWM (counts)',	(self,'pwm_desired'),range(1),[-pwm_max,pwm_max],m3g.M3GuiWrite)
		self.gui.add('M3GuiSliders','Current (A)',	(self,'current_desired'),range(1),[-current_max,current_max],m3g.M3GuiWrite)

		
		
		self.gui.add('M3GuiToggle', 'ZeroJointTheta',	(self,'zero_joint_theta'),	[],[['On','Off']],m3g.M3GuiWrite)
		self.gui.add('M3GuiToggle', 'ZeroJointTorque',	(self,'zero_joint_torque'),	[],[['On','Off']],m3g.M3GuiWrite)
		self.gui.add('M3GuiToggle', 'ZeroJointTorqueLc',(self,'zero_joint_torque_lc'),	[],[['On','Off']],m3g.M3GuiWrite)


		self.gui.add('M3GuiModes',  'Scope1',		(self,'scope_field1'),range(1),[self.scope_keys,1],m3g.M3GuiWrite)
		self.gui.add('M3GuiModes',  'Scope2',		(self,'scope_field2'),range(1),[self.scope_keys,1],m3g.M3GuiWrite)
		self.gui.add('M3GuiToggle', 'Scope',		(self,'do_scope'),[],[['On','Off']],m3g.M3GuiWrite)
		self.gui.add('M3GuiToggle', 'Save',			(self,'save'),[],[['On','Off']],m3g.M3GuiWrite)

		self.gui.start(self.step)

		
	def step(self):

#		print self.comps['act_ec']['comp'].status.timestamp

		if self.do_scope and self.scope is None:
			self.scope=m3t.M3Scope2(xwidth=100,yrange=None)
		self.proxy.step()
		self.cnt=self.cnt+1
		self.status_dict=self.proxy.get_status_dict()

	
		if self.zero_joint_theta and not self.zero_joint_theta_last:
			self.joint_theta -= self.act.get_joint_theta()
			print 'New joint_theta zero',self.joint_theta
		
		if self.zero_joint_torque and not self.zero_joint_torque_last:
			self.joint_torque -= self.act.get_joint_torque()
			print 'New joint_torque zero',self.joint_torque
#
#		if self.zero_joint_torque_lc and not self.zero_joint_torque_lc_last:
#			self.joint_torque_lc -= self.act.get_joint_torque_lc()
#			print 'New joint_torque_lc zero',self.joint_torque_lc
#			
#		self.param_dict[self.comp_name]['calibration']['zero_motor_theta']		= self.motor_theta
#		self.param_dict[self.comp_name]['calibration']['zero_joint_theta']	= self.joint_theta
#		self.param_dict[self.comp_name]['calibration']['zero_joint_torque']	= self.joint_torque
#		self.param_dict[self.comp_name]['calibration']['zero_joint_torque_lc']	= self.joint_torque_lc
#		
#		self.zero_joint_theta_last		= self.zero_joint_theta
#		self.zero_joint_torque_last		= self.zero_joint_torque
#		self.zero_joint_torque_lc_last	= self.zero_joint_torque_lc

		self.proxy.set_param_from_dict(self.param_dict)


		if self.do_scope and self.scope is not None:
			f1=self.scope_keys[self.scope_field1[0]]
			f2=self.scope_keys[self.scope_field2[0]]
			x1=x2=None
			if f1!='None' and f1!='base':
				x1=m3t.get_msg_field_value(self.act.status,f1)
				print f1,':',x1
			if f2!='None' and f2!='base':
				x2=m3t.get_msg_field_value(self.act.status,f2)   
				print f2,':',x2
			if x1==None:
				x1=x2
			if x2==None:
				x2=x1
			if x1!=None and x2!=None: #Handle only one value or two
				self.scope.plot(x1,x2)
				print'-----------------'


		if self.mode[0] == 0: #Off
			self.act.set_mode(mec.ACTUATOR_MODE_OFF)
			
		elif self.mode[0] == 1: #Pwm
			self.act.set_mode(mec.ACTUATOR_MODE_PWM)
			self.act.set_pwm(self.pwm_desired[0])
				
		elif self.mode[0] == 2: #Current
			self.act.set_mode(mec.ACTUATOR_MODE_CURRENT)
			self.act.set_i_desired(self.current_desired[0]*1000.0)
		else:
			self.act.set_mode(mec.ACTUATOR_MODE_OFF)
		
		
		if (self.save and not self.save_last):
			self.act.write_config()

		self.save_last=self.save


if __name__ == '__main__':
	t=M3Proc()
	try:
		t.start()
	except Exception as e: #(KeyboardInterrupt,EOFError):
		print "Exception " + str(e)
		pass
	t.stop()



