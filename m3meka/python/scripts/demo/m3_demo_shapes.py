#! /usr/bin/python
 
#Copyright  2010, Meka Robotics
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
import numpy as nu
from m3.unit_conversion import *
import m3.component_factory as m3f
import m3.toolbox as m3t
import m3.trajectory as m3jt
import m3.rt_proxy as m3p
import m3.toolbox as m3t
import PyKDL as kdl
import m3.viz as m3v

class M3DemoShapes:
	def __init__(self):		
		self.jt = m3jt.JointTrajectory(7)
		self.vias=[]
		self.vel_avg=20.
		self.stiffness = 0.5
		self.ik_vias=[]
		self.ndof=7
		self.theta_0 = []
		self.thetadot_0 = []
		self.viz_launched = False
		self.m3_launched = False
		self.center = []
		self.axis_demo = []
		self.axis_demo = [0, 0, 1]
		
		
	def start(self):
		print '--------------------------'
		print 'm: Target M3 arm'
		print 'v: Target RVIZ'
		print 'b: Target Both M3 and RVIZ'
		print 'q: quit'
		print '--------------'
		print
		self.k = 'a'
		while self.k!='m' and self.k!='v' and self.k!='b' and self.k!='q':
			self.k=m3t.get_keystroke()

		if self.k=='q':
			return
		
		self.proxy = m3p.M3RtProxy()
		if self.k=='m' or self.k=='b':	
			self.proxy.start()			
			
		bot_name=m3t.get_robot_name()
		if bot_name == "":
			print 'Error: no robot components found:', bot_names
			return
		self.bot=m3f.create_component(bot_name)			
				
		arm_names = ['right_arm', 'left_arm']					
		self.arm_name = m3t.user_select_components_interactive(arm_names,single=True)[0]
				
		if self.arm_name == 'right_arm':
			self.center = [0.450, -0.25, -0.1745]				
		else:
			self.center = [0.450, 0.25, -0.1745]
			
		avail_chains = self.bot.get_available_chains()
		for c in avail_chains:
			if c == 'torso':
				self.center[2] += 0.5079
			
		if self.k=='v' or self.k=='b':
			self.viz = m3v.M3Viz(self.proxy, self.bot)
			self.viz_launched = True
			self.viz.turn_sim_on()			
			
		
		if self.k=='v':			
			self.theta_0[:] = self.bot.get_theta_sim_deg(self.arm_name)[:]			
		
		if self.k=='m' or self.k=='b':
			self.proxy.subscribe_status(self.bot)
			self.proxy.publish_command(self.bot)
			self.proxy.make_operational_all()
			self.proxy.step()
			self.theta_0[:] = self.bot.get_theta_deg(self.arm_name)[:]			
			self.m3_launched = True
		
		self.theta_soln_deg = [0.]*self.bot.get_num_dof(self.arm_name)
		self.thetadot_0 = [0.]*self.bot.get_num_dof(self.arm_name)
		self.bot.set_slew_rate_proportion(self.arm_name, [1.0]*7)
		
		while True:				
			print '--------------------------'			
			print 'g: generate vias'
			print 'd: display vias'			
			print 'v: set avg velocity (Current ',self.vel_avg,')'
			print 's: set stiffness (Current',self.stiffness,')'
			if self.k=='b' or self.k=='m':
				print 'e: execute vias'
			if self.k=='b' or self.k=='v':
				print 't: test vias in visualizer'
			print 'q: quit'
			print '--------------'
			print
			m=m3t.get_keystroke()
	
			if m=='q':
				return
			
			if m=='v':
				print 'Enter avg velocity (0-60 Deg/S) [',self.vel_avg,']'
				self.vel_avg=max(0,min(60,m3t.get_float(self.vel_avg)))
				
			if m=='s':
				print 'Enter stiffness (0-1.0) [',self.stiffness,']'
				self.stiffness=max(0,min(1.0,m3t.get_float(self.stiffness)))
											
			if m == 'g':
				self.vias=[]
				print
				print '(s)quare or (c)ircle?'
				shape = None
				while shape != 's' and shape != 'c':
					shape=m3t.get_keystroke()
				length_m = 0.0
				if shape == 's':
					print
					print 'Length of square side in cm (10-25) [25]'
					length_cm = nu.float(max(10,min(25,m3t.get_int(25))))
					length_m = length_cm / 100.0
				diameter_m = 0.0
				if shape == 'c':
					print
					print 'Diameter of circle in cm (10-25) [25]'
					diameter_cm = nu.float(max(10,min(25,m3t.get_int(25))))
					diameter_m = diameter_cm / 100.0
								
				print
				print 'Enter shape resolution (1-20 vias/side) [20]'
				resolution = max(1,min(20,m3t.get_int(20)))
						
				if self.m3_launched:
					self.proxy.step()
											
				x = self.center[0]
			
				if shape == 's':
					y_left = self.center[1] + length_m/2.0
					y_right = self.center[1] - length_m/2.0
					z_top = self.center[2] + length_m/2.0
					z_bottom = self.center[2] - length_m/2.0
					dy = (y_left - y_right) / nu.float(resolution)
					dz = (z_top - z_bottom) / nu.float(resolution)
					
					if self.arm_name=='right_arm':
						# first add start point
						self.ik_vias.append([x, y_left, z_top, self.axis_demo[0], self.axis_demo[1], self.axis_demo[2]])
						# add top line
						for i in range(resolution):				
							self.ik_vias.append([x, y_left - (i+1)*dy, z_top, self.axis_demo[0],self.axis_demo[1], self.axis_demo[2]])
						# add right line
						for i in range(resolution):				
							self.ik_vias.append([x, y_right, z_top - (i+1)*dz, self.axis_demo[0], self.axis_demo[1], self.axis_demo[2]])
						# add bottom line
						for i in range(resolution):				
							self.ik_vias.append([x, y_right + (i+1)*dy, z_bottom, self.axis_demo[0], self.axis_demo[1], self.axis_demo[2]])
						# add left line
						for i in range(resolution):				
							self.ik_vias.append([x, y_left, z_bottom + (i+1)*dz, self.axis_demo[0], self.axis_demo[1], self.axis_demo[2]])
					else:
						# first add start point
						self.ik_vias.append([x, y_right, z_top, self.axis_demo[0], self.axis_demo[1], self.axis_demo[2]])
						# add top line
						for i in range(resolution):				
							self.ik_vias.append([x, y_right + (i+1)*dy, z_top, self.axis_demo[0],self.axis_demo[1], self.axis_demo[2]])
						# add right line
						for i in range(resolution):				
							self.ik_vias.append([x, y_left, z_top - (i+1)*dz, self.axis_demo[0], self.axis_demo[1], self.axis_demo[2]])
						# add bottom line
						for i in range(resolution):				
							self.ik_vias.append([x, y_left - (i+1)*dy, z_bottom, self.axis_demo[0], self.axis_demo[1], self.axis_demo[2]])
						# add left line
						for i in range(resolution):				
							self.ik_vias.append([x, y_right, z_bottom + (i+1)*dz, self.axis_demo[0], self.axis_demo[1], self.axis_demo[2]])
				if shape == 'c':
					for i in range(resolution*4 + 1):
						dt = 2*nu.pi/(nu.float(resolution)*4)
						t = (nu.pi/2) + i*dt
						if t > nu.pi:
							t -= 2*nu.pi
						y = self.center[1] + (diameter_m/2.0) * nu.cos(t)
						z = self.center[2] + (diameter_m/2.0) * nu.sin(t)
						self.ik_vias.append([x, y, z, self.axis_demo[0], self.axis_demo[1], self.axis_demo[2]])
				
						
				self.vias.append(self.theta_0[:])
				# use zero position as reference for IK solver
				ref=[0]*self.bot.get_num_dof(self.arm_name)
				# use holdup position as reference
				ref= [30,0,0,40,0,0,0]
				self.bot.set_theta_sim_deg(self.arm_name,ref)
				
				for ikv in self.ik_vias:
					theta_soln = []					
					print 'solving for ik via:', ikv
					
					if self.bot.get_tool_axis_2_theta_deg_sim(self.arm_name, ikv[:3], ikv[3:], theta_soln):					
						self.vias.append(theta_soln)
						self.bot.set_theta_sim_deg(self.arm_name,theta_soln)
					else:
						print 'WARNING: no IK solution found for via ', ikv
				self.bot.set_theta_sim_deg(self.arm_name,ref)
				if self.viz_launched:					
					self.viz.step()
				self.vias.append(self.theta_0[:])
				
			if m=='d':
				print
				print '--------- IK Vias (', len(self.ik_vias), ')--------'
				print '---------------[end_xyz[3], end_axis[3]]-----------'
				for ikv  in self.ik_vias:
					print ikv
					
				print
				print '--------- Joint Vias (', len(self.vias), ')--------'
				for v  in self.vias:
					print v
				
			if m == 'e' or m=='t':
				if len(self.vias) != 0:					
					for v in self.vias:
						#print 'Adding via',v            
						self.jt.add_via_deg(v, [self.vel_avg]*self.ndof)					
					self.jt.start(self.theta_0[:], self.thetadot_0[:])
					
					print
					print '--------- Splines (', len(self.jt.splines), ')--------'
					print '------------q_0, q_f, qdot_0, qdot_f, tf--------------'					
					for s in self.jt.splines:
						print s.q_0, s.q_f, s.qdot_0, s.qdot_f, s.tf
					
					print
					print 'Hit any key to start or (q) to quit execution'
					
					
					p=m3t.get_keystroke()
					
					if p != 'q':						
						if self.m3_launched and m=='e':
							self.bot.set_motor_power_on()
							self.bot.set_mode_theta_gc(self.arm_name)
							self.bot.set_stiffness(self.arm_name, [self.stiffness]*self.bot.get_num_dof(self.arm_name))
						while not self.jt.is_splined_traj_complete():
							q = self.jt.step()
							if self.viz_launched and m=='t':
								self.bot.set_theta_sim_deg(self.arm_name, q)
								self.viz.step()
								time.sleep(0.1)
							elif self.m3_launched and m=='e':
								self.bot.set_theta_deg(self.arm_name, q)								
								self.proxy.step()
								
						self.ik_vias=[]
						
		
			
	def stop(self):
		if self.viz_launched:
			self.viz.stop()
		if self.m3_launched:
			self.proxy.stop()
	
if __name__ == '__main__':
	demo = M3DemoShapes()
	try:
		demo.start()
	except (KeyboardInterrupt):
		pass
	demo.stop()
	
