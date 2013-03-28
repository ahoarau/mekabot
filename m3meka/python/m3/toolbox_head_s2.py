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

import os
import sys
import time
import yaml
import m3.unit_conversion as m3u
import m3.toolbox as m3t
import numpy as nu
import math
import copy
import random
import PyKDL

		
class M3HeadToolboxS2:
	def __init__(self,name,bot):
		#pass in the name of the head component
		self.bot=bot
		c=m3t.get_component_config_filename(name)
		self.config_name=c[:-4]+'_toolbox.yml'
		try:
			f=file(self.config_name,'r')
			self.config= yaml.safe_load(f.read())
		except (IOError, EOFError):
			print 'Config file not present:',self.config_name
			return

		self.joints=range(7) #lower 7Dof
		self.joint_names={'NeckTilt':0,
				  'NeckPan':1,
				  'HeadRoll':2,
				  'HeadTilt':3,
				  'EyeTilt':4,
				  'EyePanRight':5,
				  'EyePanLeft':6}

		fcr=m3t.get_m3_ros_config_path()+'cameras/'+self.config['right_camera_calibration']+'.yml'
		fcl=m3t.get_m3_ros_config_path()+'cameras/'+self.config['left_camera_calibration']+'.yml'
		try:
			f=file(fcr,'r')
			cr= yaml.safe_load(f.read())
		except (IOError, EOFError):
			print 'Config file not present:',fcr
			return
		try:
			f=file(fcl,'r')
			cl= yaml.safe_load(f.read())
		except (IOError, EOFError):
			print 'Config file not present:',fcl
			return
		pr=cr['projection_matrix']['data']
		pl=cl['projection_matrix']['data']
		#extract calibration data for camera. should really do this with ROS image_geometry package
		self.camera_calib={'right':{'fx':pr[0],'fy':pr[5],'cx':pr[2],'cy':pr[6],'w':cr['image_width'],'h':cr['image_height']},\
				   'left':{'fx':pl[0],'fy':pl[5],'cx':pl[2],'cy':pl[6] ,'w':cl['image_width'],'h':cl['image_height']}}


	def image_2_head_base(self,eye,xi,r):
		xw = self.image_2_world(eye,xi,r)
		return self.world_2_head_base(xw)
	
	def world_2_head_base(self,xw):
		#T = self.bot.get_link_2_world_transform('head', 0)
		#print
		#print T NOTE: (right column is x,y,z,1 of trans)
		#print
		#rot = self.bot.numpy_to_kdl_rotation(self.bot.get_rot_from_transform(T))	
		#trans = self.bot.list_to_kdl_vector(self.bot.get_pos_from_transform(T))
		#print 'Trans',trans NOTE: trans is bottom row, not right column, so returns [0,0,0]
		T_kdl = self.bot.get_link_2_world_kdl('head',0)#PyKDL.Frame(rot,trans)
		T_i = T_kdl.Inverse()
		p_kdl = self.bot.list_to_kdl_vector(xw)
		return nu.array(self.bot.kdl_to_list_vector(T_i*p_kdl),nu.Float)
	
	def image_2_world(self,eye,xi,r):
		xe=self.image_2_eye(eye,xi,r)
		return self.eye_2_world(eye,xe)
	
	def world_2_image(self,eye,xw):
		xe=self.world_2_eye(eye,xw)
		return self.eye_2_image(eye,xe)
	
	def eye_2_world(self,eye,xe):
		return self.bot.eye_2_world(eye,xe)
	
	def world_2_eye(self,eye,xw):
		return self.bot.world_2_eye(eye,xw)
	
	def image_2_eye(self,eye,xi,r=1.0):
		#start of ray
		p0=nu.array([0.0]*3) 
		#direction of ray
		f=(self.camera_calib[eye]['fx']+self.camera_calib[eye]['fy'])/2.0 #use avg focal
		d=nu.array([f,self.camera_calib[eye]['cx']-xi[0],self.camera_calib[eye]['cy']-xi[1]])
		d=d/math.sqrt(nu.dot(d,d)) #make unit length
		
		#centor of sphere
		pc=nu.array([0,0,0])
		pa,pb=intersection_ray_sphere(p0,d,pc,r)
		if pa[0]>0: #x direction should be pointing to forward half of camera view
			return pa
		if pb[0]>0:
			return pb 
		return None #should not hit
	
	def eye_2_image(self,eye,p):
		"""pass in a p=[x,y,z] point in eye coords. return pixel coord."""
		#map eye frame to image plane coords: eX->cZ,eY->cX eZ->cY
		x=self.camera_calib[eye]['cx']-p[1]*self.camera_calib[eye]['fx']/p[0]
		y=self.camera_calib[eye]['cy']-p[2]*self.camera_calib[eye]['fy']/p[0]
		return [x,y]

#Handle a fixed camera mounted to tool frame
class M3HeadToolboxS2ISS(M3HeadToolboxS2):
	def __init__(self,name,bot):
		M3HeadToolboxS2.__init__(self,name,bot)
		fcm=m3t.get_m3_ros_config_path()+'cameras/'+self.config['middle_camera_calibration']+'.yml'
		try:
			f=file(fcm,'r')
			cm= yaml.safe_load(f.read())
		except (IOError, EOFError):
			print 'Config file not present:',fcm
			return
		pm=cm['projection_matrix']['data']
		#extract calibration data for camera. should really do this with ROS image_geometry package
		self.camera_calib['middle']={'fx':pm[0],'fy':pm[5],'cx':pm[2],'cy':pm[6],'w':cm['image_width'],'h':cm['image_height']}
	
		P = self.bot.list_to_kdl_vector(self.config['middle_camera__translation_in_toolframe'])
		R = self.bot.list_to_kdl_rotation(self.config['middle_camera_rotation_in_toolframe'])
		self.tool_T_xe = PyKDL.Frame(R,P) #Fixed transform from middle eye frame to toolframe of head
		self.xe_T_tool=self.tool_T_xe.Inverse()
		
	def world_2_eye(self,eye,xw):
		if eye=='right' or 'eye'=='left':
			return M3HeadToolboxS2.world_2_eye(self,eye,xw)
		if eye=='middle':
			xt=self.bot.world_2_tool('head', list(xw))
			xe = self.xe_T_tool * self.bot.self.list_to_kdl_vector(xt)
			return self.bot.kdl_to_list_vector(xe)
	
		
	def eye_2_world(self,eye,xe):
		if eye=='right' or 'eye'=='left':
			return M3HeadToolboxS2.eye_2_world(self,eye,xe)
		if eye=='middle':
			xt = self.tool_T_xe * self.bot.list_to_kdl_vector(xe)
			HT=self.bot.get_tool_2_world_transform('head')
			xw=self.bot.tool_2_world('head', self.bot.kdl_to_list_vector(xt))
			return self.bot.kdl_to_list_vector(xw)
			
	def world_2_image(self,eye,xw):
		if eye=='right' or 'eye'=='left':
			return M3HeadToolboxS2.world_2_image(self,eye,xw)
		if eye=='middle':
			xe=self.world_2_eye(eye,xw,bot)
			return self.eye_2_image(eye,xe)
		
	#Note: check class inheritance rules...can probably get rid of checks and do through mostly by inheritance
	def image_2_world(self,eye,xi,r=1.0):
		if eye=='right' or 'eye'=='left':
			return M3HeadToolboxS2.image_2_world(self,eye,xi,r)
		if eye=='middle':
			xe=self.image_2_eye(eye,xi,r)
			return self.eye_2_world(eye,xe)
 
# This reduces makes a variable soft limit on the eyelid of UTA head based on the eye tilt
# Eye tilt is J4 ~+/-35 deg
# Eye lid is  J7 ~0-180 deg (180 is closed)
# When the tilt is outside of a threshold we reduce the eyelid below 180 so doesn't interfere with tilt
class M3HeadToolboxS2UTA(M3HeadToolboxS2):
	def __init__(self,name,bot):
		M3HeadToolboxS2.__init__(self,name,bot)

		#pass in the name of the head component
		jl=m3t.get_chain_joint_limits(name)
		self.eyelid_q_max=jl[7][1]
		self.eqm_param=self.config['eqm_param']

	def step_eyelids_limit(self,q_j4, q_j7):
		start=self.eqm_param['eqm_j4_ramp_start']
		end=self.eqm_param['eqm_j4_ramp_end']
		r=self.eqm_param['eqm_j7_reduce']
		if abs(q_j4)<start:
			return q_j7
		if abs(q_j4)>=start and abs(q_j4)<end:
			scale=(abs(q_j4)-start)/(end-start) #goes from 0 to 1 in start to end
			return min(q_j7,self.eyelid_q_max-r*scale)
		if abs(q_j4)>=end:
			return min(q_j7,self.eyelid_q_max-r )


# This reduces makes a variable soft limit on the eyelid of UTA head based on the eye tilt
# Eye tilt is J4 ~+/-35 deg
# Eye lid is  J7 ~0-180 deg (180 is closed)
# When the tilt is outside of a threshold we reduce the eyelid below 180 so doesn't interfere with tilt
class M3HeadToolboxS2ENS(M3HeadToolboxS2):
	def __init__(self,name,bot):
		M3HeadToolboxS2.__init__(self,name,bot)

		#pass in the name of the head component
		jl = m3t.get_chain_joint_limits(name)
		self.eyelid_q_max=jl[7][1]
		self.eyelid_q_min=jl[7][0]
		self.eqm_param=self.config['eqm_param']

	def step_eyelids_limit(self,q_j4, q_j7):
		start=self.eqm_param['eqm_j4_ramp_start']
		end=self.eqm_param['eqm_j4_ramp_end']
		r=self.eqm_param['eqm_j7_reduce']
		if abs(q_j4)<start:
			return q_j7
		if abs(q_j4)>=start and abs(q_j4)<end:
			scale=(abs(q_j4)-start)/(end-start) #goes from 0 to 1 in start to end
			return min(q_j7,self.eyelid_q_max-r*scale)
		if abs(q_j4)>=end:
			return min(q_j7,self.eyelid_q_max-r )


def spherical_to_cartesian(latitude,longitude,r=1.0):
	#Define target on sphere 
	#lat=0, long=0 corresponds to [1,0,0]
	#lat=90, long=0 corresponds to [0,0,1]
	#lat=0, long=90 corresponds to [0,1,0]
	x=r*math.sin(m3u.deg2rad(90-latitude))*math.cos(m3u.deg2rad(longitude))
	y=r*math.sin(m3u.deg2rad(90-latitude))*math.sin(m3u.deg2rad(longitude))
	z=r*math.cos(m3u.deg2rad(90-latitude))
	return [x,y,z]

def intersection_ray_sphere(p0,d,pc,r):
	#http://www.csee.umbc.edu/~olano/435f02/ray-sphere.html
	# p0: start of ray
	# d: direction of ray
	# pc: center of sphere
	# r: radius of sphere
	p0=nu.array(p0,nu.Float)
	d=nu.array(d,nu.Float)
	pc=nu.array(pc,nu.Float)
	a=nu.dot(d,d)
	b = 2*nu.dot(d,(p0-pc))
	c = nu.dot((p0 - pc),(p0 - pc)) - r*r
	discr=(b**2)-(4*a*c)
	if discr<=0:
		print 'No intersection for intersection_ray_sphere'
		return None

	if discr>0: #two intersections
		ta=(-b+math.sqrt(discr))/(2*a)
		tb=(-b-math.sqrt(discr))/(2*a)
		pa=ta*d+p0
		pb=tb*d+p0
		return pa,pb
	
    
