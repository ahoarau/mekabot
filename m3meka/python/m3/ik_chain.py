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
import numpy as nu
from m3.unit_conversion import *
import m3.gui as m3g
import m3._ik_a1r1_right
import m3._ik_a1r1_left
import m3._ik_a2r2_right
import m3._ik_a2r2_left
import m3._ik_a2r1_right
import m3._ik_a2r1_left
from datetime import *
import m3.toolbox as m3t
import PyKDL as kdl


# joint_max/min_deg = [ndof]
# end_link_transform should be generated using the DH parameters defined for the end effector link like:
# a=kdl.Frame()
# a=a.DH_Craig1989(a, alpha, d, theta)
# disc_free_angle_deg specify the discretization level the solver uses to search for a valid arm free angle 
class M3IKChain:
	def __init__(self, config = 'a2r2_right', time_out = 1.0, disc_free_angle_deg = 0.5, ndof = 7,
		     joints_max_deg = [], joints_min_deg = [], free_angle = 2, end_link_transform = kdl.Frame()):
		ik_map={'a1r1_right': m3._ik_a1r1_right,
			'a1r1_left': m3._ik_a1r1_left,
			'a2r2_right': m3._ik_a2r2_right,
			'a2r2_left': m3._ik_a2r2_left,
			'a2r1_right': m3._ik_a2r1_right,
			'a2r1_left': m3._ik_a2r1_left,
			'a2_right': m3._ik_a2r1_right,
			'a2_left': m3._ik_a2r1_left}
		self.config = config;
		self.free_angle = free_angle
		self.ndof = ndof		
		self.end_link_transform = end_link_transform
		if joints_max_deg == []:
			for i in range(ndof):
				joints_max_deg.append(180)
		if joints_min_deg == []:
			for i in range(ndof):
				joints_min_deg.append(-180)
				
		self.joints_max_deg = nu.array(joints_max_deg,float)
		#for i in range(len(joints_max_deg)):
		#    joints_max_deg[i] = m3t.wrap_deg(joints_max_deg[i])
		self.joints_min_deg = nu.array(joints_min_deg,float)
		#for i in range(len(joints_min_deg)):
		#    joints_min_deg[i] = m3t.wrap_deg(joints_min_deg[i])
		self.time_out = time_out
		self.ik = None
		
		if not ik_map.has_key(config):			
			raise m3t.M3Exception('IK not configured for type ' + str(config))
		
		self.ik = ik_map[config]		
	
		self.IKSolver = self.ik.new_IKSolver()
		self.theta_soln_rad = None
		self.free_angle_soln_rad = None
		self.discretization_free_angle = deg2rad(disc_free_angle_deg)
		
	def __del__(self):		
		self.ik.delete_IKSolver(self.IKSolver)			
		
	def __get_solution(self, i , theta_rad, free_angle_test):
		# Note: the solver requires us to provide free angles for solution if needed
		#          They do not appear to be needed but we will check just in case
		free_angles_needed = self.ik.IKSolver_GetFree(self.IKSolver, i)
		free_angles_for_soln = []
		
		for j in range(len(free_angles_needed)):
			if free_angles_needed[j] == free_angle_test:
				free_angles_for_soln.append(free_angle_test)
			else:
				free_angles_for_soln.append(theta_rad[free_angles_needed[j]])					
		
		solution = list(self.ik.IKSolver_GetSolution(self.IKSolver, i, nu.array(free_angles_for_soln,float)))		
		return solution
	
	# find_ik calls the swig wrapper until it finds a solution within joint limits, otherwise returns false
	# end_pos = [3], end_rot = [3,3], theta_deg = [ndof]
	# In general the free_angle_ref_deg parameter should be the current value for the free angle specified
	# (currently J2, but can be changed in settings for ikfast.py).
	# The free_angle_ref_deg parameter could be used to direct the arm to a certain orientation,
	# for example to push elbow out.
	# When solution is returned as list of [True, solution] where solution is list of valid joint angles
	# with Euclidian distance closest to the theta_deg parameter passed in.
	def find_ik(self, end_eff_pos, end_eff_rot, theta_deg, free_angle_ref_deg = 0, T_tool_2_wrist = kdl.Frame(), 
		    T_base_2_world = kdl.Frame):
		time_start = datetime.now()
		success = False
		end_eff_rot = nu.array(end_eff_rot,float)		
		end_rot_kdl = kdl.Rotation(end_eff_rot[0,0],end_eff_rot[0,1],end_eff_rot[0,2],
					   end_eff_rot[1,0],end_eff_rot[1,1],end_eff_rot[1,2],
					   end_eff_rot[2,0],end_eff_rot[2,1],end_eff_rot[2,2])
		end_pos_kdl = kdl.Vector(end_eff_pos[0],end_eff_pos[1],end_eff_pos[2])
		end_eff_frame = kdl.Frame(end_rot_kdl, end_pos_kdl)
		
		end_frame_to_ik = T_base_2_world.Inverse() * end_eff_frame * T_tool_2_wrist.Inverse() * self.end_link_transform.Inverse()
		end_pos_to_ik = [end_frame_to_ik.p[0], end_frame_to_ik.p[1], end_frame_to_ik.p[2]]
		end_rot_to_ik = [end_frame_to_ik.M[0,0], end_frame_to_ik.M[0,1], end_frame_to_ik.M[0,2],
				 end_frame_to_ik.M[1,0], end_frame_to_ik.M[1,1], end_frame_to_ik.M[1,2],
				 end_frame_to_ik.M[2,0], end_frame_to_ik.M[2,1], end_frame_to_ik.M[2,2]]		
		count = 0
		theta_rad = m3t.wrap_rad(deg2rad(nu.array(theta_deg,float)))
		free_angle_ref_rad = deg2rad(free_angle_ref_deg)
		max_free_angle = deg2rad(self.joints_max_deg[self.free_angle])
		min_free_angle = deg2rad(self.joints_min_deg[self.free_angle])
		num_pos_inc = (max_free_angle - free_angle_ref_rad) / self.discretization_free_angle
		num_neg_inc = (min_free_angle - free_angle_ref_rad) / self.discretization_free_angle
				
		solution = [0]*self.ndof
		free_angle_test = free_angle_ref_rad
		solution_test = []
		
		while not success:
			time_now = datetime.now()
			time_diff = time_now - time_start
			if time_diff.seconds+(time_diff.microseconds*1e-6) > self.time_out:
				#print 'Time out finding solution free angle solution.'
				return False, []  #ToDo is it better to raise m3t.M3Exception('Time out in IK search.')?
			
			success = self.ik.IKSolver_Solve(self.IKSolver, nu.array(end_pos_to_ik,float), nu.array(end_rot_to_ik,float), free_angle_test)
							
			if success:
				success = False
				num_soln = self.ik.IKSolver_GetNumSolutions(self.IKSolver)
				# make a list containing solutions in range
				for i in range(num_soln):
					solution_range_unchecked = self.__get_solution(i, theta_rad, m3t.wrap_rad(free_angle_test))
                                        #print '-----------'
                                        #print "unchecked", i, solution_range_unchecked
					if m3t.in_range_rad(solution_range_unchecked, deg2rad(self.joints_max_deg), deg2rad(self.joints_min_deg)):
						solution_test.append(solution_range_unchecked)
                                        #solution_test.append(solution_range_unchecked)
				min_dist = 1000000
				soln_ind = -1
				
				for i in range(len(solution_test)):					
					dist = m3t.find_distance(nu.array(solution_test[i],float), nu.array(theta_rad,float))
					if dist < min_dist:				
						min_dist = dist
						soln_ind = i				
				if soln_ind != -1:						
					solution = solution_test[soln_ind]
					success = True
					time_diff = datetime.now() - time_start
					#print '-----------'
					#print "min dist", min_dist
					#print '-----------'
                                        #print "valids", solution_test
					#print '-----------'
					#print 'found free angle soln in %s usec'%(time_diff.microseconds)
				else:
					solution_test = []
										
			if not success: # adjust free angle for next try
				try:
					count = m3t.get_count(count, num_pos_inc, num_neg_inc)	        
				except m3t.M3Exception, e:
					#print 'No free angle solution found.'
					return False, [] #ToDo is it better to raise m3t.M3Exception('No solution found')?	
				free_angle_test = free_angle_ref_rad + self.discretization_free_angle * count
		self.free_angle_soln_rad = free_angle_test
		self.theta_soln_rad = m3t.unwrap_rad(solution, deg2rad(self.joints_max_deg), deg2rad(self.joints_min_deg))
		return True, rad2deg(nu.array(self.theta_soln_rad,float))
