#M3 -- Meka Robotics Robot Components
#Copyright (c) 2010 Meka Robotics
#Author: edsinger@mekabot.com (Aaron Edsinger)

#M3 is free software: you can redistribute it and/or modify
#it under the terms of the GNU Lesser General Public License as published by
#the Free Software Foundation, either version 3 of the License, or
#(at your option) any later version.

#M3 is distributed in the hope that it will be useful,
#but WITHOUT ANY WARRANTY; without even the implied warranty of
#MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#GNU Lesser General Public License for more details.

#You should have received a copy of the GNU Lesser General Public License
#along with M3.  If not, see <http://www.gnu.org/licenses/>.


import time
import numpy as nu
from m3.unit_conversion import *
from datetime import *
import m3.toolbox as m3t

class JointVia:
	def __init__(self, q_desired, qdot_avg):
		self.q_desired = q_desired
		self.qdot_avg = qdot_avg
		for i in range(len(qdot_avg)):
			if qdot_avg[i] <= 0:
				raise m3t.M3Exception('qdot_avg must be greater than zero')
		
# Using splines from Craig's cubic polynomials for a path with via points
#         from Craig's Cubic polynomials for a path with via points
#         from Intro to Robotics 3rd Ed.
class JointSplineSegment:
	def __init__(self, q_0, q_f, qdot_0, qdot_f, tf):
		"""print '-----'
		print q_0, q_f, qdot_0, qdot_f, tf
		print '-----'"""
		self.a0 = nu.array(q_0,float)
		self.a1 = nu.array(qdot_0,float)
		self.a2 = (3/(tf**2)) * (nu.array(q_f,float)-nu.array(q_0,float)) - (2/tf) * nu.array(qdot_0,float) - (1/tf) * nu.array(qdot_f,float)
		self.a3 = (-2/(tf**3)) * (nu.array(q_f,float)-nu.array(q_0,float)) + (1/(tf**2)) * (nu.array(qdot_f,float)+nu.array(qdot_0,float))
		self.duration = tf
		self.q_0 = q_0
		self.q_f = q_f
		self.qdot_0 = qdot_0
		self.qdot_f = qdot_f
		self.tf = tf

	# return false if still on segment
	# return true if finished with segment
	def step(self, t):		
		self.elapsed = t		
		if t < self.duration:
			q = self.a0 + self.a1*t + self.a2*(t**2) + self.a3*(t**3)
			return False, q
		else:
			q = self.a0 + self.a1*self.duration + self.a2*(self.duration**2) + self.a3*(self.duration**3)
			return True, q

class JointTrajectory:
	def __init__(self, num_dof):		
		self.ndof=num_dof
		self.reset()
	
	def reset(self):
		self.vias = []	
		self.current = None
		self.splines = []
		self.set_idle_q = False
		self.q_0 = [0]*self.ndof
		self.q_f = [0]*self.ndof
		self.qdot_0 = [0]*self.ndof
		self.qdot_f = [0]*self.ndof
		self.qdot_avg = [0]*self.ndof
		self.qn_0 = [0]*self.ndof
		self.qn_f = [0]*self.ndof
		self.qndot_0 = [0]*self.ndof
		self.qndot_avg = [0]*self.ndof			
		self.completed_spline_idx = -1
		self.nsplines = 0
		self.theta_0 = []
		
	def add_via_rad(self, q_desired, qdot_avg):
		self.vias.append(JointVia(nu.array(q_desired,nu.float), nu.array(qdot_avg,nu.float)))
		
		
	def add_via_deg(self, q_desired, qdot_avg):
		self.add_via_rad(deg2rad(nu.array(q_desired,nu.float)), deg2rad(nu.array(qdot_avg,nu.float)))		
		

	def get_duration(self, q_start, q_end, qdot_avg):		
		duration = 0
		for i in range(self.ndof):	    
			d = nu.abs((q_end[i] - q_start[i]) / qdot_avg[i])
			if d > duration:
				duration = d		
		return duration

	# Note: self.ts_start will not get initialized until first step() call
	def start(self, theta_0, thetadot_0):
						
		if len(self.splines) == 0:
			self.add_splines(deg2rad(nu.array(theta_0,float)), deg2rad(nu.array(thetadot_0,float)))
					
		self.theta_0 = deg2rad(nu.array(theta_0,float))
		
	def is_splined_traj_complete(self):
		return self.completed_spline_idx >= (self.nsplines-1)
	
	def step(self):	
		if self.splines:
			if self.splines[0] != self.current: # we're starting a new spline
				self.ts_start = datetime.now()
			ts_dt = datetime.now() - self.ts_start
			ts = ts_dt.seconds + (ts_dt.microseconds*1e-6)
			self.current = self.splines[0]
			spline_completed, q_des = self.current.step((ts))
			if spline_completed:
				self.completed_spline_idx += 1
				self.set_idle_q = True		
				self.splines.pop(0)		
				self.q_des_last = q_des;
				self.current = None;
			return rad2deg(q_des)
		else:
			if self.set_idle_q:   #choose setpoint as last command		
				q_des = self.q_des_last
				self.set_idle_q = False
				return rad2deg(q_des)
			else:
				return rad2deg(self.theta_0)


	# adds all JointSplineSegment's to splines list
	# we choose common joint velocities using option 2 from
	#         Craig's Cubic polynomials for a path with via points
	#         from Intro to Robotics 3rd Ed.
	# qn refers to theta of next via
	def add_splines(self, theta_0, thetadot_0):
		
		self.current = None
		self.splines = []
		self.set_idle_q = False
		self.q_f = [0]*self.ndof
		self.qdot_f = [0]*self.ndof
		self.qdot_avg = [0]*self.ndof
		self.qn_0 = [0]*self.ndof
		self.qn_f = [0]*self.ndof
		self.qndot_0 = [0]*self.ndof
		self.qndot_avg = [0]*self.ndof			
		self.completed_spline_idx = -1
		self.nsplines = 0
		
		num_via = len(self.vias)
		if num_via == 0:
			return

		for i in range(num_via):
			if i == 0:
				self.q_0 = theta_0
				self.qdot_0 = thetadot_0
			for j in range(self.ndof):
				if i != 0:
					self.q_0[j] = self.vias[i-1].q_desired[j]
					self.qdot_0[j] = self.qndot_0[j]
				#print self.q_f, self.vias[i].q_desired
				self.q_f[j] = self.vias[i].q_desired[j]
				self.qdot_f[j] = 0 #Default
				self.qdot_avg[j] = self.vias[i].qdot_avg[j]
				#Next segment
				if i+1 < num_via:			
					self.qn_0[j] = self.q_f[j]
					self.qn_f[j] = self.vias[i+1].q_desired[j]
					self.qndot_avg[j] = self.vias[i+1].qdot_avg[j]
			
			tf_0 = self.get_duration(self.q_0, self.q_f, self.qdot_avg) #current duration
			# Overwrite final velociity if a next via
			if tf_0 > 0:  # prevent repeat vias
				if i+1 < num_via:
					tf_1 = self.get_duration(self.qn_0, self.qn_f, self.qndot_avg)
					if tf_1 > 0: # prevent repeat vias
						for j in range(self.ndof):
							m0 = (self.q_f[j] - self.q_0[j]) / tf_0
							m1 = (self.qn_f[j] - self.qn_0[j]) / tf_1
							if (m0>0 and m1<0) or (m0<0 and m1>0):
								self.qdot_f[j] = 0
								self.qndot_0[j] = 0
							else:
								self.qdot_f[j] = (m0 + m1) / 2
								self.qndot_0[j] = (m0 + m1) / 2
							
				self.splines.append(JointSplineSegment(self.q_0, self.q_f, self.qdot_0, self.qdot_f, tf_0))
		self.vias = []
		self.nsplines = len(self.splines)