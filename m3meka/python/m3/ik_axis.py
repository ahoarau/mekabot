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
from datetime import *
from m3.ik_chain import M3IKChain
import m3.toolbox as m3t
import PyKDL as kdl

# This component is used to generate joint angles which orient end effector such that the X-axis of end effector's local frame 
#   coincides with the reference axis specified in the chain's base frame, along with desired end position.
# The angle of the wrist around this axis is a free parameter, and it is up to the user to decide how this parameter should be chosen.
# When 'specify_wrist_angle = False', the solver will return the first solution it finds, however this solution may be on the border
#    of joints limits for some wrist joints or at an undesirable angle.  Depending on the application, a "better" solution 
#    may be determined by using specify_wrist_angle = True and varying the wrist_angle_deg value starting with the 
#    known solution found by a previous 'specify_wrist_angle = False' solution.  The set of valid wrist_roll angles and corresponding
#    wrist joint angles can then be analyzed and a wrist_angle value can be chosen that provides the "best" (centered?) wrist angle or "best" (centered?)
#    joint angles.
# disc_free_angle_deg and disc_wrist_roll_deg specify the discretization level the solver uses to search for a valid arm free angle or
#    wrist roll angle, respectively
# wrist_angle_init_deg should be set to the current Euler X angle rotation for the end effector or the best guest of what the desired Euler X angle
#     should be.
class M3IKAxis(M3IKChain):
	def __init__(self, config = 'a1_right', time_out = 30.0, disc_free_angle_deg = 1.5, ndof = 7,
		     joints_max_deg = [], joints_min_deg = [], free_angle = 2, end_link_transform = kdl.Frame(),
		     wrist_angle_init_deg = 180, disc_wrist_angle_deg = 1.5):# 1.5, 1.5 good for disc angles
		
		M3IKChain.__init__(self, config, time_out, disc_free_angle_deg, ndof,
				   joints_max_deg, joints_min_deg, free_angle, end_link_transform)
		
		self.wrist_angle_soln_deg = wrist_angle_init_deg		
		self.disc_wrist_angle_deg = disc_wrist_angle_deg
					
	# Respecifying the problem, we wish to align the X-axis of end eff to the axis specified in base frame.
	# This problem is under constrained (we don't care right now about the end frames rotation around
	# said axis, just that it coincides with it) so we can define such a satisfactory rotation by 2 Euler angles.
	# We will solve for them as follows:
	# By definition, a rotation matrix in composed of 3 unit vectors giving principle directions of a coordinate
	# system {B} written in terms of another system {A}. (Craig 3rd Ed page 21)
	# The X-unit vector for our desired rotation is the desired axis specified in the base frame (end_eff_axis).
	# The Y and Z unit vectors are arbitrary however, but we can use the z'y'x' Euler angle rotation formula
	# on Craig 3rd Edition page 374 to solve for alpha (z' rot) and beta (y' rot) setting the first column equal to our X unit vector.
	# Now:  X[0] = cos(alpha)*cos(beta), X[1] = sin(alpha)*cos(beta), X[2] = -sin(beta)
	# Solving for beta first:  beta = arcsin(-X[2])
	# Rearranging X[0] and X[1] we can say:
	#   cos(alpha) = X[0]/cos(beta)
	#   sin(alpha) = X[1]/cos(beta)
	#   solve for alpha using alpha = arctan2(sin(alpha), cos(alpha))
	#   alpha = arctan2(X[1]/cos(beta),X[0]/cos(beta))
	# Note: cos(beta) will be zero when beta is +/-90 which means X[2] = 1
	#   this will create the ugly divide by zero, because now alpha is undefined,
	#   but alpha can take any angle because rotation in the Z direction is not necessary.	
	def find_ik_axis(self, end_eff_pos, end_eff_axis, theta_deg, free_angle_ref_deg = 0, specify_wrist_roll = False, wrist_angle_deg = 0, 
			 T_tool_2_wrist = kdl.Frame(), T_base_2_world = kdl.Frame):
		time_start = datetime.now()
		end_axis_mag = nu.sqrt(nu.float(end_eff_axis[0])**2 + nu.float(end_eff_axis[1])**2 + nu.float(end_eff_axis[2])**2)
		#end axis normalized
		end_eff_axis = [nu.float(end_eff_axis[0])/end_axis_mag, 
				nu.float(end_eff_axis[1])/end_axis_mag,
				nu.float(end_eff_axis[2])/end_axis_mag]
		
		beta = nu.arcsin(-end_eff_axis[2])
		cos_beta = nu.cos(beta)
		if cos_beta == 0:
			cos_beta = 0.001		
		alpha = nu.arctan2(end_eff_axis[1]/cos_beta, end_eff_axis[0]/cos_beta)		
		
		if specify_wrist_roll:
			end_rot_kdl = kdl.Rotation()
			end_rot_kdl = end_rot_kdl.EulerZYX(alpha, beta, wrap_rad(deg2rad(wrist_roll_deg)))
			end_rot_to_ik = [end_rot_kdl[0,0], end_rot_kdl[0,1], end_rot_kdl[0,2],
					 end_rot_kdl[1,0], end_rot_kdl[1,1], end_rot_kdl[1,2],
					 end_rot_kdl[2,0], end_rot_kdl[2,1], end_rot_kdl[2,2]]
			[success, solution_deg]  = self.find_ik(end_eff_pos, end_rot_to_ik, theta_deg, free_angle_ref_deg, 
								T_tool_2_wrist, T_base_2_world)
			if success:
				self.wrist_roll_soln_deg = wrist_roll_deg
			return [success, solution_deg]
			
		success = False
		count = 0		
		num_pos_inc = (180) / self.disc_wrist_angle_deg
		num_neg_inc = (-180) / self.disc_wrist_angle_deg
		# since desired wrist angle was not specified or failed, use previous solution as starting point
		wrist_angle_test_deg = self.wrist_angle_soln_deg
		#print [yaw, pitch, wrist_roll_rad]
		
		while not success:
			time_now = datetime.now()
			time_diff = time_now - time_start
			if time_diff.seconds+(time_diff.microseconds*1e-6) > self.time_out:
				#print 'Timeout searching for roll angle. ', time_diff.seconds+(time_diff.microseconds*1e-6)
				return False, []
			#print 'trying roll angle: ', rad2deg(wrist_roll_test)
			end_rot_kdl = kdl.Rotation()			
			end_rot_kdl = end_rot_kdl.EulerZYX(alpha, beta, m3t.wrap_rad(deg2rad(wrist_angle_test_deg)))
			end_rot_to_ik = [[end_rot_kdl[0,0], end_rot_kdl[0,1], end_rot_kdl[0,2]],
					 [end_rot_kdl[1,0], end_rot_kdl[1,1], end_rot_kdl[1,2]],
					 [end_rot_kdl[2,0], end_rot_kdl[2,1], end_rot_kdl[2,2]]]
			
			[success, solution_deg] = self.find_ik(end_eff_pos, end_rot_to_ik, theta_deg, free_angle_ref_deg, 
							       T_tool_2_wrist, T_base_2_world)
			
			if success:
				self.wrist_angle_soln_deg = wrist_angle_test_deg
				#print 'found axis soln in %s sec angle: %s'%(time_diff.seconds+(time_diff.microseconds*1e-6), wrist_angle_test_deg)				
				return [success, solution_deg]			
			try:
				count = m3t.get_count(count, num_pos_inc, num_neg_inc)
			except m3t.M3Exception, e:
				#print 'No solution found for roll angle'
				return False, []
			
			wrist_angle_test_deg = self.wrist_angle_soln_deg + self.disc_wrist_angle_deg * count
			wrist_angle_test_deg=m3t.wrap_deg(wrist_angle_test_deg)
			
	
