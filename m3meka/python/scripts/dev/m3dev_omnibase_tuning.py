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
import m3.omnibase as m3o
import m3.omnibase_pb2 as mob
import time

class M3Proc:
	def __init__(self):
		self.proxy = m3p.M3RtProxy()
		self.gui = m3g.M3Gui(stride_ms=125)#125
		'''self.max_lin_acc = 0.25
		self.max_lin_vel = 0.5
		self.max_rot_acc = 8
		self.max_rot_vel = 17'''
		
		self.max_lin_vel = 0.25  # m/s (0.6)
		self.max_lin_acc = 0.4 # m/s^2  (0.2)  --- 1.0 gives really good performance but saturates motors...
		self.max_rot_vel = 25   # deg/s
		self.max_rot_acc = 60  # 100 for better    # deg/s^2

	def stop(self):
		self.pwr.set_motor_power_off()
		self.proxy.step()
		self.proxy.stop()

	def start(self):
		self.proxy.start()
		
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
								
		#Setup Components
		base_name=self.proxy.get_available_components('m3omnibase')
		if len(base_name)!=1:
			print 'Invalid number of base components available'
			self.proxy.stop()
			exit()
		self.omni=m3o.M3OmniBase(base_name[0])
		
		self.proxy.publish_param(self.omni) 
		self.proxy.subscribe_status(self.omni)
		self.proxy.publish_command(self.omni)
		
		pwr_name=[m3t.get_omnibase_pwr_component_name(base_name[0])]
		self.pwr=m3f.create_component(pwr_name[0])
		
		self.proxy.subscribe_status(self.pwr)
		self.proxy.publish_command(self.pwr) 
		self.proxy.make_operational(pwr_name[0])
		self.proxy.step()
					
		self.pwr.set_motor_power_on()
		
		self.proxy.make_operational_all()
		
		self.proxy.step()
		
		self.omni.calibrate(self.proxy)

		#Create gui
		self.ctrl_mode=[0]
		self.traj_mode=[0]
		self.joy_button=[0]
		self.local_velocity_desired_x=[0]
		self.local_velocity_desired_y=[0]
		self.local_velocity_desired_heading=[0]
		self.joy_x=[0]
		self.joy_y=[0]
		self.joy_yaw=[0]
		self.goal_x=[0]
		self.goal_y=[0]
		self.goal_yaw=[0]
		self.caster_ctrl_idx=[0]
		self.caster_ctrl_mode=[0]
		self.steer_or_roll_mode=[0]
		self.op_force_desired_x=[0]
		self.op_force_desired_y=[0]
		self.op_torque_desired=[0]
		self.joint_torque=[0]		
		self.joint_vel=[0]
		self.save=False
		self.save_last=False
		self.init_g = self.omni.get_global_position()
		
		self.status_dict=self.proxy.get_status_dict()
		self.param_dict=self.proxy.get_param_dict()
		self.gui.add('M3GuiTree',   'Status',    (self,'status_dict'),[],[],m3g.M3GuiRead,column=2)
		self.gui.add('M3GuiTree',   'Param',   (self,'param_dict'),[],[],m3g.M3GuiWrite,column=3)
		self.gui.add('M3GuiModes',  'CtrlMode',      (self,'ctrl_mode'),range(1),[['Off','Calibrate','Caster','OpSpaceForce','OpSpaceTraj','CartLocal','CartGlob'],1],m3g.M3GuiWrite)
		self.gui.add('M3GuiModes',  'TrajMode',      (self,'traj_mode'),range(1),[['Off','Joystick','Goal','Vias'],1],m3g.M3GuiWrite)
		self.gui.add('M3GuiModes',  'Caster Steer/Roll',      (self,'steer_or_roll_mode'),range(1),[['Steer','Roll'],1],m3g.M3GuiWrite)
		self.gui.add('M3GuiModes',  'CasterCtrlIdx',      (self,'caster_ctrl_idx'),range(1),[['0','1','2','3'],1],m3g.M3GuiWrite)
		self.gui.add('M3GuiModes',  'CasterCtrlMode',      (self,'caster_ctrl_mode'),range(1),[['Off','Tq','Vel'],1],m3g.M3GuiWrite)		
		self.gui.add('M3GuiModes',  'JoyButton',      (self,'joy_button'),range(1),[['Tri','Circ','Sqr','X'],1],m3g.M3GuiWrite)
		self.gui.add('M3GuiSliders','JoystickX', (self,'joy_x'),range(1),[-1,1],m3g.M3GuiWrite,column=1)
		self.gui.add('M3GuiSliders','JoystickY', (self,'joy_y'),range(1),[-1,1],m3g.M3GuiWrite,column=1)
		self.gui.add('M3GuiSliders','JoystickYaw', (self,'joy_yaw'),range(1),[-1,1],m3g.M3GuiWrite,column=1)
		self.gui.add('M3GuiSliders','LocalVelX (m/s)', (self,'local_velocity_desired_x'),range(1),[-1,1],m3g.M3GuiWrite,column=1)
		self.gui.add('M3GuiSliders','LocalVelY (m/s)', (self,'local_velocity_desired_y'),range(1),[-1,1],m3g.M3GuiWrite,column=1)
		self.gui.add('M3GuiSliders','LocalVelHeading (deg/s)', (self,'local_velocity_desired_heading'),range(1),[-60,60],m3g.M3GuiWrite,column=1)
		self.gui.add('M3GuiSliders','OpForceX (N)', (self,'op_force_desired_x'),range(1),[-1000,1000],m3g.M3GuiWrite,column=1)
		self.gui.add('M3GuiSliders','OpForceY (N)', (self,'op_force_desired_y'),range(1),[-1000,1000],m3g.M3GuiWrite,column=1)
		self.gui.add('M3GuiSliders','OpTorque (Nm)', (self,'op_torque_desired'),range(1),[-50,50],m3g.M3GuiWrite,column=1)
		self.gui.add('M3GuiSliders','Jnt Torque (Nm)', (self,'joint_torque'),range(1),[-12.8,12.8],m3g.M3GuiWrite,column=1)		
#		self.gui.add('M3GuiSliders','Jnt Torque (Nm)', (self,'joint_torque'),range(1),[-6.4,6.4],m3g.M3GuiWrite,column=1)		
		self.gui.add('M3GuiSliders','Jnt Vel (deg/s)', (self,'joint_vel'),range(1),[-90,90],m3g.M3GuiWrite,column=1)
		self.gui.add('M3GuiSliders','Goal X (m)', (self,'goal_x'),range(1),[-2,2],m3g.M3GuiWrite,column=1)
		self.gui.add('M3GuiSliders','Goal Y (m)', (self,'goal_y'),range(1),[-2,2],m3g.M3GuiWrite,column=1)
		self.gui.add('M3GuiSliders','Goal Yaw(deg)', (self,'goal_yaw'),range(1),[-180,180],m3g.M3GuiWrite,column=1)
		time.sleep(0.5)
		self.omni.set_local_position(0,0,0,self.proxy)
		self.omni.set_global_position(0,0,0,self.proxy)
		self.gui.start(self.step)
		

	def step(self):		
		self.status_dict=self.proxy.get_status_dict()
		self.proxy.set_param_from_dict(self.param_dict)
		self.omni.set_ctrl_mode(self.ctrl_mode[0])
		self.omni.set_traj_mode(self.traj_mode[0])		
		self.omni.set_local_velocities(self.local_velocity_desired_x[0], self.local_velocity_desired_y[0],self.local_velocity_desired_heading[0])
		self.omni.set_local_positions(0,0,0)
		self.omni.set_local_accelerations(0,0,0)
		self.omni.set_op_space_forces(self.op_force_desired_x[0],self.op_force_desired_y[0],self.op_torque_desired[0])
		self.omni.set_max_linear_accel(self.max_lin_acc)
		self.omni.set_max_linear_velocity(self.max_lin_vel)
		self.omni.set_max_rotation_velocity(self.max_rot_vel)
		self.omni.set_max_rotation_accel(self.max_rot_acc)		
		if self.ctrl_mode[0] == mob.OMNIBASE_CTRL_CASTER:
			for i in range(4):
				self.omni.set_mode_caster_off(i)				
				self.omni.set_steer_torques(0,i)
				self.omni.set_steer_velocities(0,i)
				self.omni.set_roll_torques(0,i)
				self.omni.set_roll_velocities(0,i)
				if self.steer_or_roll_mode[0] == 0:
					if self.caster_ctrl_idx[0] == i:
						if self.caster_ctrl_mode[0] == mob.OMNIBASE_CASTER_TORQUE:
							self.omni.set_mode_caster_torque(i)
							self.omni.set_steer_torques(self.joint_torque[0],i)						
						elif self.caster_ctrl_mode[0] == mob.OMNIBASE_CASTER_VELOCITY:
							self.omni.set_mode_caster_velocity(i)
							self.omni.set_steer_velocities(self.joint_vel[0],i)
				else:
					if self.caster_ctrl_idx[0] == i:
						if self.caster_ctrl_mode[0] == mob.OMNIBASE_CASTER_TORQUE:
							self.omni.set_mode_caster_torque(i)
							self.omni.set_roll_torques(self.joint_torque[0],i)						
						elif self.caster_ctrl_mode[0] == mob.OMNIBASE_CASTER_VELOCITY:
							self.omni.set_mode_caster_velocity(i)
							self.omni.set_roll_velocities(self.joint_vel[0],i)
		self.omni.set_joystick_x(self.joy_x[0])
		self.omni.set_joystick_y(self.joy_y[0])
		self.omni.set_joystick_yaw(self.joy_yaw[0])
		#self.omni.set_joystick_button(-1)
		self.omni.set_joystick_button(self.joy_button[0])
		#self.omni.set_traj_goal(self.init_g[0]+self.goal_x[0], self.init_g[1]+self.goal_y[0], self.init_g[2]+self.goal_yaw[0])
		self.omni.set_traj_goal(self.goal_x[0], self.goal_y[0], self.goal_yaw[0])
		print 'Bus voltage',self.omni.get_bus_voltage()
		self.proxy.step()
                
if __name__ == '__main__':
	t=M3Proc()
	try:
		t.start()
	except (KeyboardInterrupt,EOFError):
		pass
	t.stop()





