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
import m3.toolbox as m3t
import m3.rt_proxy as m3p
import m3.humanoid 
import m3.hand
import m3.gui as m3g
import m3.trajectory as m3jt
import numpy as nu
import yaml

class M3Proc:
	def __init__(self):
		self.proxy = m3p.M3RtProxy()
		self.gui = m3g.M3Gui(stride_ms=50)
	def stop(self):
		self.bot.set_mode_off(self.arm_name)
		self.bot.set_mode_off('torso')
		self.proxy.stop()

	def start(self):
		# ######## Setup Proxy and Components #########################
		self.proxy.start()
		bot_name=m3t.get_robot_name()
		if bot_name == "":
			print 'Error: no robot components found:', bot_names
			return
		self.bot=m3.humanoid.M3Humanoid(bot_name)	
		arm_names = self.bot.get_available_chains()	
		arm_names = [x for x in arm_names if x.find('arm')!=-1]
			
			
		if len(arm_names)==0:
			print 'No arms found'
			return
		if len(arm_names)==1:
			self.arm_name=arm_names[0]
		else:
			self.arm_name = m3t.user_select_components_interactive(arm_names,single=True)[0]

		# ####### Setup Hand #############
		hand_names=self.proxy.get_available_components('m3hand')
		hand_name=''
		if len(hand_names)>1:
			hand_name=m3t.user_select_components_interactive(hand_names,single=True)[0]
		if len(hand_names)==1:
			hand_name=hand_names[0]
		if len(hand_name):
			self.hand=m3.hand.M3Hand(hand_name)
			self.proxy.publish_command(self.hand)
			self.proxy.subscribe_status(self.hand)
		else:
			self.hand=None
		# ####### Setup Proxy #############
		self.proxy.subscribe_status(self.bot)
		self.proxy.publish_command(self.bot)
		self.proxy.make_operational_all()
		self.bot.set_motor_power_on()
		self.ndof=self.bot.get_num_dof(self.arm_name)
		self.via_traj={}
		self.via_traj_torso={}
		self.via_traj_first=True

		# ######## Square/Circle stuff #########################
		if self.arm_name == 'right_arm':
			self.center = [0.450, -0.25, -0.1745]				
		else:
			self.center = [0.450, 0.25, -0.1745]		
		avail_chains = self.bot.get_available_chains()
		for c in avail_chains:
			if c == 'torso':
				self.center[2] += 0.5079

		self.jt = m3jt.JointTrajectory(7)
		self.axis_demo = [0, 0, 1]

		# ##### Generate square vias ############################
		ik_vias=[]
		length_m = 20/ 100.0
		resolution = 20
		y_left = self.center[1] + length_m/2.0
		y_right = self.center[1] - length_m/2.0
		z_top = self.center[2] + length_m/2.0
		z_bottom = self.center[2] - length_m/2.0
		dy = (y_left - y_right) / nu.float(resolution)
		dz = (z_top - z_bottom) / nu.float(resolution)
		x = self.center[0]
		if self.arm_name=='right_arm':
			# first add start point
			ik_vias.append([x, y_left, z_top, self.axis_demo[0], self.axis_demo[1], self.axis_demo[2]])
			# add top line
			for i in range(resolution):				
				ik_vias.append([x, y_left - (i+1)*dy, z_top, self.axis_demo[0],self.axis_demo[1], self.axis_demo[2]])
			# add right line
			for i in range(resolution):				
				ik_vias.append([x, y_right, z_top - (i+1)*dz, self.axis_demo[0], self.axis_demo[1], self.axis_demo[2]])
			# add bottom line
			for i in range(resolution):				
				ik_vias.append([x, y_right + (i+1)*dy, z_bottom, self.axis_demo[0], self.axis_demo[1], self.axis_demo[2]])
			# add left line
			for i in range(resolution):				
				ik_vias.append([x, y_left, z_bottom + (i+1)*dz, self.axis_demo[0], self.axis_demo[1], self.axis_demo[2]])
		else:
			# first add start point
			ik_vias.append([x, y_right, z_top, self.axis_demo[0], self.axis_demo[1], self.axis_demo[2]])
			# add top line
			for i in range(resolution):				
				ik_vias.append([x, y_right + (i+1)*dy, z_top, self.axis_demo[0],self.axis_demo[1], self.axis_demo[2]])
			# add right line
			for i in range(resolution):				
				ik_vias.append([x, y_left, z_top - (i+1)*dz, self.axis_demo[0], self.axis_demo[1], self.axis_demo[2]])
			# add bottom line
			for i in range(resolution):				
				ik_vias.append([x, y_left - (i+1)*dy, z_bottom, self.axis_demo[0], self.axis_demo[1], self.axis_demo[2]])
			# add left line
			for i in range(resolution):				
				ik_vias.append([x, y_right, z_bottom + (i+1)*dz, self.axis_demo[0], self.axis_demo[1], self.axis_demo[2]])
		self.via_traj['Square']=[]
		# use zero position as reference for IK solver
		self.bot.set_theta_sim_deg(self.arm_name,[0]*self.bot.get_num_dof(self.arm_name))
		for ikv in ik_vias:
			theta_soln = []					
			#print 'solving for square ik via:', ikv
			if self.bot.get_tool_axis_2_theta_deg_sim(self.arm_name, ikv[:3], ikv[3:], theta_soln):					
				self.via_traj['Square'].append(theta_soln)
				self.bot.set_theta_sim_deg(self.arm_name,theta_soln)
			else:
				print 'WARNING: no IK solution found for via ', ikv
		self.bot.set_theta_sim_deg(self.arm_name,[0]*self.bot.get_num_dof(self.arm_name))

		# ##### Generate circle vias ############################
		ik_vias=[]
		diameter_m = 20.0 / 100.0
		resolution = 20
		x = self.center[0]
		for i in range(resolution*4 + 1):
			dt = 2*nu.pi/(nu.float(resolution)*4)
			t = (nu.pi/2) + i*dt
			if t > nu.pi:
				t -= 2*nu.pi
			y = self.center[1] + (diameter_m/2.0) * nu.cos(t)
			z = self.center[2] + (diameter_m/2.0) * nu.sin(t)
			ik_vias.append([x, y, z, self.axis_demo[0], self.axis_demo[1], self.axis_demo[2]])
		self.via_traj['Circle']=[]
		# use zero position as reference for IK solver
		self.bot.set_theta_sim_deg(self.arm_name,[0]*self.bot.get_num_dof(self.arm_name))
		for ikv in ik_vias:
			theta_soln = []					
			if self.bot.get_tool_axis_2_theta_deg_sim(self.arm_name, ikv[:3], ikv[3:], theta_soln):					
				self.via_traj['Circle'].append(theta_soln)
				self.bot.set_theta_sim_deg(self.arm_name,theta_soln)
			else:
				print 'WARNING: no IK solution found for via ', ikv
		self.bot.set_theta_sim_deg(self.arm_name,[0]*self.bot.get_num_dof(self.arm_name))				

		# ##### Load Via Trajectories ############################
		self.via_files={'TrajA':'right_left_torso_demo_a.via'}
		pt=m3t.get_m3_animation_path()
		for k in self.via_files.keys():
			fn=pt+self.via_files[k]
			try:
				f=file(fn,'r')
				d=yaml.safe_load(f.read())
				self.via_traj[k]=d[self.arm_name]['postures']
				self.via_traj_torso[k]=d['torso']['postures']
			except IOError:
				print 'Via file',k,'not present. Skipping...'


		# ######## Demo and GUI #########################
		self.off=False
		self.grasp=False
		self.torso_mode_names=['Off','Zero','ZeroForce','FollowArmTraj']
		self.arm_mode_names=['Off','Gravity','Zero','Current','HoldUp','Square','Circle','TrajA']#,'ZeroForce']
		self.hand_mode_names=['Off','Open','Grasp',]
		self.arm_mode_methods=[self.step_off,self.step_gravity,self.step_zero,self.step_current,self.step_hold_up,self.step_via_traj,
				       self.step_via_traj,self.step_via_traj,self.step_zero_force]
		self.hand_mode_methods=[self.step_hand_off,self.step_hand_open,self.step_hand_grasp]
		self.torso_mode_methods=[self.step_torso_off,self.step_torso_zero,self.step_torso_zero_force,self.step_torso_follow_traj]
		self.arm_mode=[0]
		self.torso_mode=[0]
		self.hand_mode=[0]
		self.poses={'zero':  {'right_arm':[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],'left_arm':[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]},
			    'holdup':{'right_arm':[56.0, 26.0, -8.0, 84.0, 119.0, -36.0, 2.0],'left_arm':[56.0, -26.0, 8.0, 84.0, -119.0, -36.0, -2.0]}}		
		self.stiffness=[35]
		self.velocity=[25]
		self.gui.add('M3GuiModes',  'Arm Mode',      (self,'arm_mode'),range(1),[self.arm_mode_names,1],m3g.M3GuiWrite,column=1)
		self.gui.add('M3GuiModes',  'Torso Mode',      (self,'torso_mode'),range(1),[self.torso_mode_names,1],m3g.M3GuiWrite,column=1)
		self.gui.add('M3GuiToggle', 'ESTOP', (self,'off'),[],[['Arm Enabled','Arm Disabled']],m3g.M3GuiWrite,column=1)
		if self.hand is not None:
			self.gui.add('M3GuiToggle', 'Grasp', (self,'grasp'),[],[['GraspOpen','GraspClosed']],m3g.M3GuiWrite,column=1)
			self.gui.add('M3GuiModes',  'Hand Mode',      (self,'hand_mode'),range(1),[self.hand_mode_names,1],m3g.M3GuiWrite,column=1)
		self.gui.add('M3GuiSliders','Stiffness ', (self,'stiffness'),[0],[0,100],m3g.M3GuiWrite,column=1) 
		self.gui.add('M3GuiSliders','Velocity ', (self,'velocity'),[0],[0,40],m3g.M3GuiWrite,column=1) 
		self.gui.start(self.step)

	def get_arm_mode_name(self):
		return self.arm_mode_names[self.arm_mode[0]]
	
	def get_torso_mode_name(self):
		return self.torso_mode_names[self.torso_mode[0]]

	def get_hand_mode_name(self):
		return self.hand_mode_names[self.hand_mode[0]]

	def get_stiffness(self):
		return [self.stiffness[0]/100.0]*self.ndof

	def step(self):
		self.proxy.step()

		print 'Arm: ',self.get_arm_mode_name()
		apply(self.arm_mode_methods[self.arm_mode[0]])
		if self.get_arm_mode_name()!='Square' and self.get_arm_mode_name()!='Circle' and self.get_arm_mode_name()!='TrajA':
			self.via_traj_first=True
		if self.get_arm_mode_name()!='Current':
			self.current_first=True
		apply(self.torso_mode_methods[self.torso_mode[0]])
		if self.hand is not None:
			#print 'Hand',self.get_hand_mode_name()
			if self.get_hand_mode_name()!='Animation':
				self.hand_traj_first=True
			apply(self.hand_mode_methods[self.hand_mode[0]])
			

	def step_hand_off(self):
		self.hand.set_mode_off()

	def step_hand_open(self):
		self.hand.set_stiffness([0.8]*5)
		self.hand.set_mode_theta_gc([0,1,2,3,4])
		self.hand.set_slew_rate_proportion([1.0]*5)
		self.hand.set_theta_deg([0.0]*5)

	def step_hand_grasp(self):
		self.hand.set_stiffness([0.8]*5)
		self.hand.set_slew_rate_proportion([1.0]*5)
		if self.grasp:
			self.hand.set_mode_theta_gc([0])
			self.hand.set_mode_torque_gc([1,2,3,4])
			self.hand.set_torque_mNm([200.0]*4,[1,2,3,4])
			self.hand.set_theta_deg([90.0],[0])
		else:
			self.hand.set_mode_theta_gc([0,1,2,3,4])
			self.hand.set_theta_deg([90.0,0.0,0.0,0.0,0.0])


	def step_via_traj(self):
		self.bot.set_mode_splined_traj_gc(self.arm_name)
		self.bot.set_stiffness(self.arm_name, self.get_stiffness())
		self.bot.set_slew_rate_proportion(self.arm_name,[1.0]*self.ndof)
		
		if self.have_torso_follow_traj:
			self.bot.set_mode_splined_traj_gc('torso')
			self.bot.set_stiffness('torso', [0.75,0.75,0.75])
			self.bot.set_slew_rate_proportion('torso',[1.0]*self.bot.get_num_dof('torso'))
		
		if self.via_traj_first and len(self.via_traj[self.get_arm_mode_name()]):
			self.via_traj_first=False
			theta_0 = self.bot.get_theta_deg(self.arm_name)[:]	
			vias=[theta_0]+self.via_traj[self.get_arm_mode_name()]#start at current pos	
			for v in vias:
				self.bot.add_splined_traj_via_deg(self.arm_name, v,[self.velocity[0]]*self.ndof)
			if self.have_torso_follow_traj and len(self.via_traj_torso[self.get_arm_mode_name()]):
				theta_0 = self.bot.get_theta_deg('torso')[:]	
				vias=[theta_0]+self.via_traj[self.get_arm_mode_name()] #start at current pos	
				for v in vias:
					self.bot.add_splined_traj_via_deg('torso', v,[self.velocity[0]/2.0]*self.bot.get_num_dof('torso'))
		if  self.bot.is_splined_traj_complete(self.arm_name):
			if self.have_torso_follow_traj:
				if self.bot.is_splined_traj_complete('torso'):
					self.via_traj_first=True
			else:
				self.via_traj_first=True

	def step_current(self):
		if self.current_first:
			self.current_first=False
			self.theta_curr = self.bot.get_theta_deg(self.arm_name)[:]
		self.bot.set_mode_theta_gc_mj(self.arm_name)
		self.bot.set_theta_deg(self.arm_name,self.theta_curr)
		self.bot.set_stiffness(self.arm_name,self.get_stiffness())
		self.bot.set_thetadot_deg(self.arm_name,[20.0]*self.ndof)   

	def step_torso_off(self):
		self.bot.set_mode_off('torso')
		self.have_torso_follow_traj = False
		
	def step_torso_zero(self):
		self.bot.set_mode_theta_gc_mj('torso')
		self.bot.set_theta_deg('torso',[0,0,0])
		self.bot.set_stiffness('torso',[0.75,0.75,0.75])
		self.bot.set_thetadot_deg('torso',[5.0,5.0,5.0])
		self.have_torso_follow_traj = False
		
	def step_torso_zero_force(self):
		self.bot.set_mode_theta_gc_mj('torso')
		self.bot.set_theta_deg('torso',[0,0,0])
		self.bot.set_stiffness('torso',[0.0,0.0,0.0])
		self.bot.set_thetadot_deg('torso',[5.0,5.0,5.0])
		self.have_torso_follow_traj = False
		
	def step_torso_follow_traj(self):
		self.have_torso_follow_traj = True
		
	def step_zero(self):
		self.bot.set_mode_theta_gc_mj(self.arm_name)
		self.bot.set_theta_deg(self.arm_name,self.poses['zero'][self.arm_name])
		self.bot.set_stiffness(self.arm_name,self.get_stiffness())
		self.bot.set_thetadot_deg(self.arm_name,[20.0]*self.ndof)

	def step_hold_up(self):
		self.bot.set_mode_theta_gc_mj(self.arm_name)
		self.bot.set_theta_deg(self.arm_name,self.poses['holdup'][self.arm_name])
		self.bot.set_stiffness(self.arm_name,self.get_stiffness())
		self.bot.set_thetadot_deg(self.arm_name,[15.0]*self.ndof)

	def step_gravity(self):
		#for now just J0
		self.bot.set_mode_theta_gc(self.arm_name)
		self.bot.set_theta_deg(self.arm_name,self.poses['zero'][self.arm_name])
		self.bot.set_stiffness(self.arm_name,[0.0,.75,.75,.75,.75,.75,.75])
		self.bot.set_thetadot_deg(self.arm_name,[20.0]*self.ndof)
		return True

	def step_zero_force(self):
		#self.bot.set_mode_torque(self.arm_name)
		self.bot.set_mode_off(self.arm_name)
		#self.bot.set_torque(self.arm_name,[0.0]*self.ndof)
		return True
	
	def step_off(self):
		self.bot.set_mode_off(self.arm_name)



if __name__ == '__main__':
	t=M3Proc()
	try:
		t.start()
	except (KeyboardInterrupt,EOFError):
		pass
	t.stop()





