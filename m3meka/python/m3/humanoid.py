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

import yaml
import os 
from m3.toolbox import *
import m3.humanoid_pb2 as mrt
import m3.joint_array_mode_pb2 as mab
import m3.smoothing_mode_pb2 as msm
from m3.component import M3Component
import m3.toolbox as m3t
import m3.toolbox_ctrl as m3tc
from m3.unit_conversion import *
import numpy as nu
from m3.ik_axis import M3IKAxis
from m3.ik_chain import M3IKChain
from m3.robot import M3Robot
from PyKDL import *

class ChainAttributes:   
    def __init__(self):
	self.chain_name = None
	self.T_parent_base = Frame()
	self.T_wrist_tool = Frame()
	self.ndof = 0
	self.max_slew_rates = []
	self.d = []
	self.a = []
	self.alpha = []
	self.joint_offset = []
	self.kdl_chain = Chain()
	self.fksolver_pos = None	
	self.theta_sim = []
	self.ik_axis = None
	self.ik = None
	self.joints_max_deg = []
	self.joints_min_deg = []
	self.joint_names = []
	self.vias = []
	self.via_idx = 0
	
class CameraAttributes:
    def __init__(self):
	self.camera_name=None
	self.focal_length
	self.resolution_x
	self.resolution_y
	
class M3Humanoid(M3Robot):
    """
    The M3Humanoid class has been designed as the principal interface for controlling an M3 robot.
    It must be defined in the m3_config.yml file and can be created using the m3.component_factory module.
    The example belows creates an interface for an m3humanoid defined as m3humanoid_mr0.
    
    >>> import m3.humanoid as m3h
    >>> bot = m3h.M3Humanoid('m3humanoid_mr0') # creates M3Humanoid class
    
    The M3Humanoid class can be used to send commands and retrieve status updates to and from the m3 realtime server.  The 
    example below configures the realtime proxy to update the M3Humanoid class with updates from the robot and to recieve commands.
    It also sets the joint controllers to joint angle control with gravity compenstation.
    
    >>> import m3.rt_proxy as m3p
    >>> proxy = m3p.M3RtProxy()
    >>> proxy.start() # m3rt server must be running first
    >>> proxy.make_operational_all() 
    >>> proxy.subscribe_status(bot)
    >>> proxy.publish_command(bot) 
    >>> proxy.publish_param(bot)
    >>> bot.set_mode_theta_gc('torso')
    >>> bot.set_mode_theta_gc('right_arm')
    >>> bot.set_motor_power_on()
    >>> proxy.step()
    
    Now the M3Humanoid class can be used to issue joint commands.
    
    >>> bot.set_slew_rate_proportion('right_arm',[0.5]*7)
    >>> bot.set_theta_deg('right_arm', [0]*7)  # command right arm to default position at half max slew rate.
    >>> proxy.step()
    
    The M3Humanoid class also provides simulated forward kinematics to allow testing of joint configurations.  For these "sim"
    methods the component does not communicate with the realtime server and can be used without a proxy.
    
    >>> import m3.component_factory as m3f
    >>> bot = m3f.create_component('m3humanoid_mr0') # creates M3Humanoid class    
    >>> q_t = [0., 10., 10.]
    >>> bot.set_theta_sim_deg('torso', q_t)	
    >>> q_a = [0., 10., 20., 30., 40., 30., 20.]
    >>> bot.set_theta_sim_deg('right_arm', q_a)
    >>> T = bot.get_tool_2_world_transform_sim('right_arm')    
    """
    def __init__(self,name,type='m3humanoid'):
        M3Robot.__init__(self,name,type=type)
	
        self.status=mrt.M3HumanoidStatus()
        self.command=mrt.M3HumanoidCommand()
        self.param=mrt.M3HumanoidParam()
			
	self.right_arm = ChainAttributes()
	self.left_arm = ChainAttributes()
	self.torso = ChainAttributes()
	self.head = ChainAttributes()
	self.available_chains = []
	self.read_config()
	
	self.right_arm.theta_sim = [0]*self.right_arm.ndof
	self.left_arm.theta_sim = [0]*self.left_arm.ndof
	self.head.theta_sim = [0]*self.head.ndof
	self.torso.theta_sim = [0]*self.torso.ndof
	
    def initialize(self, proxy):
	"""
	Configures Humanoid in a state to publish commands and parameter updates and receive status updates.
	Also enables power to the motors and sets all components in OP state.
		
	:param proxy: running proxy
	:type proxy: M3RtProxy
	"""
	proxy.subscribe_status(self)
	proxy.publish_command(self)
	proxy.publish_param(self)
	self.set_motor_power_on()
	proxy.make_operational_all()
	proxy.step()
	

    def get_available_chains(self):
	"""
	Returns list of chains defined in humanoid config file.
		
	:rtype: list of strings
	:returns: list of chains defined in humanoid config file.
	"""
	return self.available_chains
	
    def tool_2_world(self, chain, p):
	"""
	Maps a point in the tool frame to the world frame.  Units assumed to be in meters.
	
	:param chain: Desired chain to use for tool frame.
	:type chain: 'right_arm', 'left_arm', 'torso', 'head'
	:param p: Description of vector with respect to tool frame.
	:type p: array_like, shape (3)
	
	:rtype: numpy.array, shape (3)
	:returns: Description of vector with respect to world frame.
		
	:raises: M3Exception if p_t.shape is not (3) or chain is not supported
	
	:See Also: 
	   :meth:`M3Humanoid.get_tool_2_world_transform`
	   :meth:`M3Humanoid.link_2_world`
	   :meth:`M3Humanoid.world_2_tool`
	   
	:Examples:
	
	Map the point [1,2,3] from right arm tool frame to world frame.
	
	>>> from m3.humanoid import M3Humanoid
	>>> bot = M3Humanoid('bob')
	>>> import m3.rt_proxy as m3p
	>>> proxy = m3p.M3RtProxy()
	>>> proxy.start()
	>>> proxy.subscribe_status(bot)
	>>> proxy.step()
	>>> p_tool = [1.,2.,3.]
	>>> p_world = bot.tool_2_world('right_arm', p_tool)	    
	"""
	
	T = self.__get_tool_2_world_kdl(chain)
	#print p, len(p)
	
	p_kdl = self.list_to_kdl_vector(p)
	return nu.array(self.kdl_to_list_vector(T*p_kdl),float)
    
    def world_2_tool(self, chain, p):
	"""
	Maps a point in the world frame to the tool frame.  Units assumed to be in meters.
	
	:param chain: Desired chain to use for tool frame.
	:type chain: 'right_arm', 'left_arm', 'torso', 'head'
	:param p: Description of vector with respect to world frame.
	:type p: array_like, shape (3)
	
	:rtype: numpy.array, shape (3)
	:returns: Description of vector with respect to world frame.
		
	:raises: M3Exception if p_t.shape is not (3) or chain is not supported
	
	:See Also: 
	   :meth:`M3Humanoid.get_tool_2_world_transform`
	   :meth:`M3Humanoid.link_2_world`
	   :meth:`M3Humanoid.tool_2_world`
	
	:Examples:
	
	Map the point [1,2,3] from right arm tool frame to world frame.
	
	>>> from m3.humanoid import M3Humanoid
	>>> bot = M3Humanoid('bob')
	>>> import m3.rt_proxy as m3p
	>>> proxy = m3p.M3RtProxy()
	>>> proxy.start()
	>>> proxy.subscribe_status(bot)
	>>> proxy.step()
	>>> p_world = [1.,2.,3.]
	>>> p_tool = bot.world_2_tool('right_arm', p_world)	    
	"""
	
	T = self.__get_tool_2_world_kdl(chain)
	Ti=T.Invers()
	#print p, len(p)
	
	p_kdl = self.list_to_kdl_vector(p)
	return nu.array(self.kdl_to_list_vector(Ti*p_kdl),float)
    
    def link_2_world(self, chain, n, p):
	"""
	Maps a point in link frame to the world frame.  Units assumed to be in meters.
	
	:param chain: Desired chain to use for tool frame.
	:type chain: 'right_arm', 'left_arm', 'torso', 'head'
	:param n: Link frame at joint n to use.
	:type n: int, <= ndof+1
	:param p: Description of vector with respect to link frame.
	:type p: array_like, shape (3)
	      			
	:rtype: numpy.array, shape (3)
	:returns: Description of vector with respect to world frame.
	      
	:raises: M3Exception if p.shape is not (3), chain is not supported, or n > ndof+1
	
	:See Also: 
	   :meth:`M3Humanoid.get_link_2_world_transform`
	   :meth:`M3Humanoid.tool_2_world`	   
	   :meth:`M3Humanoid.world_2_link`
	   
	:Examples:
	
	Map the point [1,2,3] from right arm link_5 frame to world frame.
	
	>>> from m3.humanoid import M3Humanoid
	>>> bot = M3Humanoid('bob')
	>>> import m3.rt_proxy as m3p
	>>> proxy = m3p.M3RtProxy()
	>>> proxy.start()
	>>> proxy.subscribe_status(bot)
	>>> proxy.step()
	>>> p_link = [1.,2.,3.]
	>>> p_world = bot.link_2_world('right_arm', 5, p_link)
	"""	
	T = self.get_link_2_world_kdl(chain, n)
	p_vec = self.list_to_kdl_vector(p)
	return T * p_vec
    
    def world_2_link(self, chain, n, p):
	"""
	Maps a point in world frame to the link frame.  Units assumed to be in meters.
	
	:param chain: Desired chain to use for tool frame.
	:type chain: 'right_arm', 'left_arm', 'torso', 'head'
	:param n: Link frame at joint n to use.
	:type n: int, <= ndof+1
	:param p: Description of vector with respect to world frame.
	:type p: array_like, shape (3)
	      			
	:rtype: numpy.array, shape (3)
	:returns: Description of vector with respect to link frame.
	      
	:raises: M3Exception if p.shape is not (3), chain is not supported, or n > ndof+1
	
	:See Also: 
	   :meth:`M3Humanoid.get_link_2_world_transform`
	   :meth:`M3Humanoid.tool_2_world`	   
	   :meth:`M3Humanoid.link_2_world`	
	
	:Examples:
	
	Map the point [1,2,3] from right arm link_5 frame to world frame.
	
	>>> from m3.humanoid import M3Humanoid
	>>> bot = M3Humanoid('bob')
	>>> import m3.rt_proxy as m3p
	>>> proxy = m3p.M3RtProxy()
	>>> proxy.start()
	>>> proxy.subscribe_status(bot)
	>>> proxy.step()
	>>> p_world = [1.,2.,3.]
	>>> p_link = bot.world_2_link('right_arm', 5, p_world)
	"""	
	T = self.get_link_2_world_kdl(chain, n)
	Ti=T.Inverse()
	p_vec = self.list_to_kdl_vector(p)
	return Ti * p_vec
    
    def get_link_2_world_transform(self, chain, n):
	"""
	Describes the coordinate system attached at link joint relative to the world frame.  This same
	transform can also be used to map descriptions in the link frame to the world frame.  Translation component is in meters.
			
	:param chain: Desired chain to use for tool frame.
	:type chain: 'right_arm', 'left_arm', 'torso', 'head'
	:param n: Link frame at joint n to use.
	:type n: int, <= ndof+1
	    
	:rtype: numpy.array, shape (4,4)
	:returns: Description of link frame with respect to world frame.
	      
	:raises: M3Exception if chain is not supported or n > ndof+1
	
	:See Also: 
	   :meth:`M3Humanoid.get_tool_2_world_transform`
	   :meth:`M3Humanoid.link_2_world`
	   :meth:`M3Humanoid.get_link_2_world_transform_sim`
	
	:Examples:
	
	Get the tranform describing the right arm link_5 frame in reference to world frame.
	
	>>> from m3.humanoid import M3Humanoid
	>>> bot = M3Humanoid('bob')
	>>> import m3.rt_proxy as m3p
	>>> proxy = m3p.M3RtProxy()
	>>> proxy.start()
	>>> proxy.subscribe_status(bot)
	>>> proxy.step()
	>>> T = bot.get_link_2_world_transform('right_arm', 5)		
	"""	
	return self.kdl_2_numpy_transform(self.get_link_2_world_kdl(chain, n))
	
    
    def get_tool_2_world_transform(self, chain):
	"""
	Describes the coordinate system attached at the tool tip relative to the world frame.  This same
	transform can also be used to map descriptions in the tool frame to the world frame.  The position and
	orientation of the tool tip are described by this transform.  Translation component is in meters.
	
	:param chain: Desired chain to use for tool frame.
	:type chain: 'right_arm', 'left_arm', 'torso', 'head'
	:returns: Description of tool frame with respect to world frame.
	:rtype:	numpy.array, shape (4,4)
	      
	:raises: M3Exception if chain is not supported.
	
	:See Also: 
	   :meth:`M3Humanoid.get_link_2_world_transform`
	   :meth:`M3Humanoid.tool_2_world`
	   :meth:`M3Humanoid.get_tool_2_world_transform_sim`
	     
	:Examples:
	
	Get the tranform describing the right arm tool frame in reference to world frame.
	
	>>> from m3.humanoid import M3Humanoid
	>>> bot = M3Humanoid('bob')
	>>> import m3.rt_proxy as m3p
	>>> proxy = m3p.M3RtProxy()
	>>> proxy.start()
	>>> proxy.subscribe_status(bot)
	>>> proxy.step()
	>>> T = bot.get_tool_2_world_transform('right_arm')		
	       
	To extract position and rotation components:
	
	>>> R = bot.get_rot_from_transform(T)	
	>>> p = bot.get_pos_from_transform(T)	
	"""	
	return self.kdl_2_numpy_transform(self.__get_tool_2_world_kdl(chain))
    
    def __get_tool_2_world_kdl(self, chain):
	self.__assert_chain(chain)
	end_rot = self.get_status(chain).end_rot
	end_pos = self.get_status(chain).end_pos
	P = self.list_to_kdl_vector(end_pos)
	R = self.list_to_kdl_rotation(end_rot)
	T = Frame(R,P)
	return T * self.__get_tool_2_wrist_kdl(chain)
    
    
    def get_end_2_world_jacobian(self, chain):
	"""
	Returns Jacobian expressed in the world frame for the end frame defined by the chain DH kinematic parameters.
			
	:param chain: Desired chain to use for end frame.
	:type chain: 'right_arm', 'left_arm', 'torso', 'head'
	:returns: Jacobian with respect to world frame.
	:rtype:	numpy.array, shape (6,ndof)
	      
	:raises: M3Exception if chain is not supported.
			     
	:Examples:
	
	Get the Jacobian describing the right arm end frame in reference to world frame.
	
	>>> from m3.humanoid import M3Humanoid
	>>> bot = M3Humanoid('bob')
	>>> import m3.rt_proxy as m3p
	>>> proxy = m3p.M3RtProxy()
	>>> proxy.start()
	>>> proxy.subscribe_status(bot)
	>>> proxy.step()
	>>> J = bot.get_end_2_world_jacobian('right_arm')	
	"""	
	self.__assert_chain(chain)
	J = nu.array(self.get_status(chain).J,float)
	ndof = self.get_num_dof(chain)
	J.resize([6,ndof])
	return J
	
    def get_link_2_world_transform_sim(self, chain, n):
	"""
	This method is similar to get_link_2_world_transform(), except that instead of using joint values from the m3rt
	realtime server to calculate forward kinematics it uses artificial values set using set_theta_sim_deg().
	Describes the coordinate system attached at link joint relative to the world frame.  This same
	transform can also be used to map descriptions in the link frame to the world frame. Translational units assumed to be in meters.
	
	:param chain: Desired chain to use for tool frame.
	:type chain: 'right_arm', 'left_arm', 'torso', 'head'
	:param n: Link frame at joint n to use.
	:type n: int, <= ndof+1
	    
	:rtype: numpy.array, shape (4,4)
	:returns: Description of link frame with respect to world frame.
	      
	:raises: M3Exception if chain is not supported or n > ndof+1
	
	:See Also: 
	   :meth:`M3Humanoid.get_link_2_world_transform`
	   :meth:`M3Humanoid.get_tool_2_world_transform_sim`
	
	:Examples:
	
	Get the tranform describing the right arm link_5 frame in reference to world frame using artificial joint values.
	
	>>> from m3.humanoid import M3Humanoid
	>>> bot = M3Humanoid('bob')
	>>> q_t = [0., 10., 10.]
	>>> bot.set_theta_sim_deg('torso', q_t)	
	>>> q_a = [0., 10., 20., 30., 40., 30., 20.]
	>>> bot.set_theta_sim_deg('right_arm', q_a)
	>>> T = bot.get_link_2_world_transform_sim('right_arm', 5)		
	"""	
	return self.kdl_2_numpy_transform(self.__get_link_2_world_kdl_sim(chain, n))
    
    def get_rot_from_transform(self, t):
	"""
	Extracts 3x3 rotation matrix from 4x4 frame description.
	
	:param t: Description of frame.
	:type t: array_like, shape (4,4)
	
	:rtype: numpy.array, shape (3,3)
	:returns: Rotation matrix component of frame description.
	
	:raises: M3Exception if t.shape is not (4,4)
		
	:See Also: 
	   :meth:`M3Humanoid.get_pos_from_transform`
	   
	:Examples:
		
	>>> T = bot.get_tool_2_world_transform('right_arm')
	>>> R = bot.get_rot_from_transform(T)	
	"""
	self.__assert_matrix_size(t,4,4)
	return nu.array(t,float)[:3,:3]
    
    def get_pos_from_transform(self, t):
	"""
	Extracts 3 element position vector from 4x4 frame description.
	
	:param t: Description of frame.
	:type t: array_like, shape (4,4)
	
	:rtype: numpy.array, shape (3)
	:returns: Translation vector component of frame description.
	
	:raises: M3Exception if t.shape is not (4,4)
		
	:See Also: 
	   :meth:`M3Humanoid.get_rot_from_transform`
	   
	:Examples:
		
	>>> T = bot.get_tool_2_world_transform('right_arm')
	>>> p = bot.get_pos_from_transform(T)	
	"""
	self.__assert_matrix_size(t,4,4)
	t = nu.array(t,float)
	p = nu.zeros((3))
	p[0] = t[3,0]
	p[1] = t[3,1]
	p[2] = t[3,2]
	return p
    
    def numpy_2_kdl_transform(self, frame_np):
	p = self.get_pos_from_transform(frame_np)
	r = self.get_rot_from_transform(frame_np)
	
	p_kdl = self.list_to_kdl_vector(p)
	r_kdl = self.numpy_to_kdl_rotation(r)
	
	return Frame(r_kdl, p_kdl)
    
    def kdl_2_numpy_transform(self, frame_kdl):
	rot = self.kdl_to_numpy_rotation(frame_kdl.M)
	pos = self.kdl_to_list_vector(frame_kdl.p)
	
	return self.rot_and_pos_2_transform(rot, pos)
    
    def rot_and_pos_2_transform(self,R,P):
	"""
	Creates 4x4 frame description from rotation matrix and position vector.
	
	:param R: Rotation matrix of transform
	:type R: array_like, shape (3,3)
	:param P: Translation component of transform
	:type P: array_like, shape (3)
	
	:rtype: array_like, shape (4,4)
	:returns: Description of frame.
	
	:raises: M3Exception if R.shape is not (3,3) or P.shape is not (3)
		
	:See Also: 
	   :meth:`M3Humanoid.get_rot_from_transform`
	   :meth:`M3Humanoid.get_pos_from_transform`
	   
	:Examples:
	
	>>> R = np.array([[1,0,0],[0,1,0],[0,0,1]])
	>>> p = np.array([1,2,3])
	>>> T = bot.rot_and_pos_2_transform(R,p)
	>>> T	
	array([[ 1.,  0.,  0.,  1.],
	       [ 0.,  1.,  0.,  2.],
	       [ 0.,  0.,  1.,  3.],
	       [ 0.,  0.,  0.,  1.]])
	"""
	self.__assert_list_size(P,3)
	self.__assert_matrix_size(R,3,3)
	frame=nu.zeros([4,4])	
	rot = nu.array(R,float)	
	pos=nu.array(P,float)	
	frame[:3,:3] = rot
	frame[0,3] = pos[0]
	frame[1,3] = pos[1]
	frame[2,3] = pos[2]	
	frame[3,3] = 1
	return frame
    
    def get_tool_2_world_transform_sim(self, chain):
	"""
	This method is similar to get_tool_2_world_transform(), except that instead of using joint values from the m3rt
	realtime server to calculate forward kinematics it uses artificial values set using set_theta_sim_deg().
	Describes the coordinate system attached at the tool tip relative to the world frame.  This same
	transform can also be used to map descriptions in the tool frame to the world frame.  Translational units assumed to be in meters.
	
	:param chain: Desired chain to use for tool frame.
	:type chain: 'right_arm', 'left_arm', 'torso', 'head'
	:returns: Description of tool frame with respect to world frame.
	:rtype:	numpy.array, shape (4,4)
	      
	:raises: M3Exception if chain is not supported.
	
	:See Also: 
	   :meth:`M3Humanoid.get_link_2_world_transform_sim`	   
	   :meth:`M3Humanoid.get_tool_2_world_transform`
	
	:Examples:
	
	Get the tranform describing the right arm tool frame in reference to world frame using artificial joint values.
	
	>>> from m3.humanoid import M3Humanoid
	>>> bot = M3Humanoid('bob')
	>>> q_t = [0., 10.]
	>>> bot.set_theta_sim_deg('torso', q_t)	
	>>> q_a = [0., 10., 20., 30., 40., 30., 20.]
	>>> bot.set_theta_sim_deg('right_arm', q_a)
	>>> T = bot.get_tool_2_world_transform_sim('right_arm')		
	"""	
	return self.kdl_2_numpy_transform(self.__get_tool_2_world_kdl_sim(chain))
    
    def __get_tool_2_world_kdl_sim(self, chain):	
	self.__assert_chain(chain)
	T_base_end_torso = Frame()	
	if self.torso.fksolver_pos is not None:	    
	    j=JntArray(self.torso.kdl_chain.getNrOfJoints())
	    for i in range(self.torso.kdl_chain.getNrOfJoints()):
		j[i] = deg2rad(self.torso.theta_sim[i])
	    err = self.torso.fksolver_pos.JntToCart(j, T_base_end_torso)	
	    if err != 0:
		raise m3t.M3Exception('Error in forward kinematics solution')
	T_world_end_torso = self.torso.T_parent_base * T_base_end_torso
	
	if chain == 'torso':
	    return T_world_end_torso
	    		
	chain_attr = getattr(self, chain)
		
	T_base_end = Frame()
	if chain_attr.fksolver_pos is not None:	   	    
	    j=JntArray(chain_attr.kdl_chain.getNrOfJoints())	    
	    for i in range(chain_attr.kdl_chain.getNrOfJoints()):
		j[i] = deg2rad(chain_attr.theta_sim[i])
	    err = chain_attr.fksolver_pos.JntToCart(j, T_base_end)
	    if err != 0:
		raise m3t.M3Exception('Error in forward kinematics solution')	
	T_world_tool = T_world_end_torso * chain_attr.T_parent_base * T_base_end * chain_attr.T_wrist_tool
	return T_world_tool

    def get_link_2_world_kdl(self, chain, n):
	self.__assert_chain(chain)
	T_base_end_torso = Frame()	
	if self.torso.fksolver_pos is not None:
	    torso_theta = self.get_theta_rad('torso')
	    j=JntArray(self.torso.kdl_chain.getNrOfJoints())
	    for i in range(self.torso.kdl_chain.getNrOfJoints()):
		j[i] = deg2rad(torso_theta[i])
	    err = 0
	    if chain == 'torso':
		err = self.torso.fksolver_pos.JntToCart(j, T_base_end_torso, n)
	    else:
		err = self.torso.fksolver_pos.JntToCart(j, T_base_end_torso)
	    if err != 0:
		raise m3t.M3Exception('Error in forward kinematics solution')
	T_world_end_torso = self.torso.T_parent_base * T_base_end_torso
	
	if chain == 'torso':
	    return T_world_end_torso
	    		
	chain_attr = getattr(self, chain)
	theta = self.get_theta_rad(chain)
	T_base_end = Frame()
	if chain_attr.fksolver_pos is not None:	   	    
	    j=JntArray(chain_attr.kdl_chain.getNrOfJoints())
	    for i in range(chain_attr.kdl_chain.getNrOfJoints()):
		j[i] = deg2rad(theta[i])	    
	    err = chain_attr.fksolver_pos.JntToCart(j, T_base_end, n)	    
	    if err != 0:
		raise m3t.M3Exception('Error in forward kinematics solution')
		
	T_world_tool = T_world_end_torso * chain_attr.T_parent_base * T_base_end * chain_attr.T_wrist_tool
	return T_world_tool
    
    def __get_link_2_world_kdl_sim(self, chain, n):
	self.__assert_chain(chain)
	T_base_end_torso = Frame()	
	if self.torso.fksolver_pos is not None:	    
	    j=JntArray(self.torso.kdl_chain.getNrOfJoints())
	    for i in range(self.torso.kdl_chain.getNrOfJoints()):
		j[i] = deg2rad(self.torso.theta_sim[i])
	    err = 0
	    if chain == 'torso':
		err = self.torso.fksolver_pos.JntToCart(j, T_base_end_torso, n)
	    else:
		err = self.torso.fksolver_pos.JntToCart(j, T_base_end_torso)
	    if err != 0:
		raise m3t.M3Exception('Error in forward kinematics solution')
	T_world_end_torso = self.torso.T_parent_base * T_base_end_torso
	
	if chain == 'torso':
	    return T_world_end_torso
	    		
	chain_attr = getattr(self, chain)
		
	T_base_end = Frame()
	if chain_attr.fksolver_pos is not None:	   	    
	    j=JntArray(chain_attr.kdl_chain.getNrOfJoints())	    	    
	    for i in range(chain_attr.kdl_chain.getNrOfJoints()):
		j[i] = deg2rad(chain_attr.theta_sim[i])
	    err = chain_attr.fksolver_pos.JntToCart(j, T_base_end, n)
	    if err != 0:
		raise m3t.M3Exception('Error in forward kinematics solution')
		
	T_world_tool = T_world_end_torso * chain_attr.T_parent_base * T_base_end * chain_attr.T_wrist_tool
	return T_world_tool
    
    def set_theta_sim_deg(self, chain, theta, ind=None):
	"""
	Sets artificial joint values for selected chain.  These joint values are used for simulated forward
	kinematics methods.  A list of joint indexes can be supplied to set specific joint angles, or the index
	can be omitted if the length of theta is equal to the number of degrees of freedom for that chain.
	
	:param chain: Desired chain.
	:type chain: 'right_arm', 'left_arm', 'torso', 'head'
	:param theta: Joint values in degrees.
	:type theta: array_like
	:param ind: Index of joints.
	:type ind: array_like, shape(len(theta)), optional
	
	:raises: 
	   M3Exception if chain is not supported or theta.shape is not (ndof) and
	   theta.shape is not ind.shape and ind is not None.
	  	
	:See Also: 
	   :meth:`M3Humanoid.set_theta_sim_rad`	   
	   :meth:`M3Humanoid.set_theta_deg`
	   :meth:`M3Humanoid.get_theta_sim_deg`
	   
	:Examples:

	To set all virtual joint angles with one list:
	
	>>> from m3.humanoid import M3Humanoid
	>>> bot = M3Humanoid('bob')	
	>>> q_a = [0., 10., 20., 30., 40., 30., 20.]
	>>> bot.set_theta_sim_deg('right_arm', q_a)
	>>> bot.get_theta_sim_deg()
	array([0., 10., 20., 30., 40., 30., 20.])
	
	To change a subset of joint angles:
	
	>>> q_b = [50., 60.]
	>>> index = [3, 4]
	>>> bot.set_theta_sim_deg('right_arm', q_b, index)
	>>> bot.get_theta_sim_deg()
	array([0., 10., 20., 50., 60., 30., 20.])  # Now only joints 3 and 4 have changed.
	
	
	"""
	self.__assert_chain(chain)
	ndof = self.get_num_dof(chain)
	self.__assert_list_size(theta, ndof)	
	chain_attr = getattr(self, chain)
	chain_attr.theta_sim = theta[:]
	
	
    def __assert_chain(self, chain):
	if not self.available_chains.__contains__(chain):
	    raise m3t.M3Exception('chain: ' + chain + ' not configured. Choices are: ' + self.available_chains.__str__())
	
    def set_theta_sim_rad(self, chain, theta, ind=None):
	"""
	Sets artificial joint values for selected chain.  These joint values are only used for simulated forward
	kinematics methods.  A list of joint indexes can be supplied to set specific joint angles, or the index
	can be omitted if the length of theta is equal to the number of degrees of freedom for that chain.
	
	:param chain: Desired chain.
	:type chain: 'right_arm', 'left_arm', 'torso', 'head'
	:param theta: Joint values in radians.
	:type theta: array_like
	:param ind: Index of joints.
	:type ind: array_like, shape(len(theta)), optional
	
	:raises: 
	   M3Exception if chain is not supported or theta.shape is not (ndof) and
	   theta.shape is not ind.shape and ind is not None.
	  	
	:See Also: 
	   :meth:`M3Humanoid.set_theta_sim_deg`	   
	   :meth:`M3Humanoid.set_theta_deg`
	   :meth:`M3Humanoid.get_theta_sim_rad`
	   
	:Examples:

	To set all virtual joint angles with one list:
	
	>>> from m3.humanoid import M3Humanoid
	>>> bot = M3Humanoid('bob')	
	>>> q_a = [0., .1, .2, .3, .4, .3, .2]
	>>> bot.set_theta_sim_deg('right_arm', q_a)
	>>> bot.get_theta_sim_deg()
	array([0., .1, .2, .3, .4, .3, .2])
	
	To change a subset of joint angles:
	
	>>> q_b = [.5, .6]
	>>> index = [3, 4]
	>>> bot.set_theta_sim_deg('right_arm', q_b, index)
	>>> bot.get_theta_sim_deg()
	array([0., .1, .2, .5, .6, .3, .2])  # Now only joints 3 and 4 have changed.
	"""
	self.set_theta_sim_deg(chain, rad2deg(theta), ind)
	
    def set_tool_2_wrist_transform(self, chain, T):
	"""
	Sets description of tool frame relative to wrist reference frame.  The frame description is intialized by the 
	values in the humanoid yaml files, but can be changed using this method if tools are changed dynamically at run time.		
	
	:param chain: Desired chain to use for tool frame.
	:type chain: 'right_arm', 'left_arm', 'torso', 'head'
	:param T: Frame describing tool frame relative to wrist reference frame.
	:type T: array_like, shape (4,4)
	
	.. Note:: 
	   Translational units must be meters.  The columns of R should be orthonormal vectors.
	
	:raises: M3Exception if chain is not supported or T.shape is not (4,4)
	
	:See Also: 
	   :meth:`M3Humanoid.set_tool_2_wrist_rpy_deg`
	   :meth:`M3Humanoid.set_tool_2_wrist_rpy_position`
	   :meth:`M3Humanoid.get_tool_2_wrist_transform`	   	   
	   
	:Examples:
		
	>>> from m3.humanoid import M3Humanoid
	>>> import numpy as np
	>>> bot = M3Humanoid('bob')	
	>>> R_t = [[-1,0,0],[0,-1,0],[0,0,-1]]
	>>> p_t = [.01,.02,.03]
	>>> T_t = bot.rot_and_pos_2_transform(R_t, p_t)
	>>> bot.set_tool_2_wrist_transform('right_arm', T_t)
	>>> bot.get_tool_2_wrist_transform()
	array([[ -1.,  0.,  0.,  .01],
	       [ 0.,  -1.,  0.,  .02],
	       [ 0.,  0.,  -1.,  .03],
	       [ 0.,  0.,  0.,   1.]])
	"""
	self.__assert_matrix_size(T, 4, 4)
	chain_attr = getattr(self, chain)
	rot = self.numpy_to_kdl_rotation(self.get_rot_from_transform(T))	
	trans = self.list_to_kdl_vector(self.get_pos_from_transform(T))	
	chain_attr.T_wrist_tool = Frame(rot,trans)	
	
    def set_tool_2_wrist_position(self, chain, tool_pos):
	"""
	Sets position of tool frame origin relative to wrist reference frame.  The tool frame origin is intialized by the 
	the humanoid yaml files, but can be changed using this method if tools are changed dynamically at run time.		
	
	:param chain: Desired chain to use for tool frame.
	:type chain: 'right_arm', 'left_arm', 'torso', 'head'
	:param tool_pos: tool frame origin in meters specified in wrist coordinate system.
	:type tool_pos: array_like, shape (3)
	
	.. Note:: 
	   Translational units must be meters.
	
	:raises: M3Exception if chain is not supported or tool_pos.shape is not (3)
	
	:See Also: 
	   :meth:`M3Humanoid.set_tool_2_wrist_transform`
	   :meth:`M3Humanoid.set_tool_2_wrist_rpy_deg`
	   :meth:`M3Humanoid.set_tool_2_wrist_rpy_rad`
	   
	:Examples:
		
	>>> from m3.humanoid import M3Humanoid
	>>> import numpy as np
	>>> bot = M3Humanoid('bob')	
	>>> p_t = [.01,.02,.03]
	>>> bot.set_tool_2_wrist_position('right_arm', p_t)	
	"""
	self.__assert_list_size(tool_pos,3)
	trans = self.list_to_kdl_vector(tool_pos)
	T = self.__get_tool_2_wrist_kdl(chain)
	T.p = trans
	
    def set_tool_2_wrist_rpy_deg(self, chain, tool_rpy):
	"""
	Sets orientation of tool frame relative to wrist reference frame.  The orientation is intialized by the 
	humanoid yaml files, but can be changed using this method if tools are changed dynamically at run time.		
	
	:param chain: Desired chain to use for tool frame.
	:type chain: 'right_arm', 'left_arm', 'torso', 'head'
	:param tool_rpy: roll, pitch, and yaw of tool frame in degrees for chain with respect to wrist frame.
	:type tool_rpy: array_like, shape (3)
	
	.. Note::
	
	   * -180 <= Roll <= 180
	   * -90 <= Pitch <= 90
	   * -180 <= Yaw  <= 180
	
	   Roll, pitch, and yaw angles in this context are considered X-Y-Z fixed angles around a fixed
	   reference frame (the world frame).  Starting with the tool frame coincident with the world frame,
	   the RPY values returned describe the tool frame orientation as follows:
	   
	   * First rotate the tool frame by roll angle around X-axis of world frame.
	   * Next rotate the tool frame by pitch angle around Y-axis of world frame.	   
	   * Finally rotate the tool frame by yaw angle around Z-axis of world frame.	
	
	:raises: M3Exception if chain is not supported or tool_pos.shape is not (3)
	
	:See Also: 
	   :meth:`M3Humanoid.set_tool_2_wrist_transform`
	   :meth:`M3Humanoid.set_tool_2_wrist_position`
	   :meth:`M3Humanoid.set_tool_2_wrist_rpy_rad`
	   
	:Examples:
		
	>>> from m3.humanoid import M3Humanoid	
	>>> bot = M3Humanoid('bob')	
	>>> rpy = [90, 25, 45]
	>>> bot.set_tool_2_wrist_rpy('right_arm', rpy)	
	"""
	self.set_tool_2_wrist_rpy_rad(chain, deg2rad(nu.array(tool_rpy,float)))
	
    def set_tool_2_wrist_rpy_rad(self, chain, tool_rpy):
	'''
	Same as :meth:`M3Humanoid.set_tool_2_wrist_rpy_deg` except expecting input in radians
	:See Also: 
	   :meth:`M3Humanoid.set_tool_2_wrist_rpy_deg`
	   :meth:`M3Humanoid.set_tool_2_wrist_transform`
	   :meth:`M3Humanoid.set_tool_2_wrist_position`
	   
	'''
	self.__assert_list_size(tool_rpy, 3)
	rot_kdl = Rotation()
	rot_kdl = Rotation.RPY(tool_rpy[0], tool_rpy[1], tool_rpy[2])	
	T = self.__get_tool_2_wrist_kdl(chain)
	T.M = rot_kdl
	
    def get_tool_2_wrist_transform(self, chain):
	"""
	Gets description of tool frame relative to wrist reference frame.  The frame description is intialized by the 
	values in the humanoid yaml files, but can be changed using set_tool_2_wrist_transform  method if tools are changed dynamically at run time.
	Translational units are in meters.
	
	:param chain: Desired chain to use for tool frame.
	:type chain: 'right_arm', 'left_arm', 'torso', 'head'
	:returns: Description of tool frame with respect to wrist frame.
	:rtype:	numpy.array, shape (4,4)
	
	:raises: M3Exception if chain is not supported or T.shape is not (4,4)
	
	:See Also: 
	   :meth:`M3Humanoid.set_tool_2_wrist_transform`
	   
	:Examples:
		
	>>> from m3.humanoid import M3Humanoid
	>>> import numpy as np
	>>> bot = M3Humanoid('bob')	
	>>> R_t = [[-1,0,0],[0,-1,0],[0,0,-1]]
	>>> p_t = [.1,.2,.3]
	>>> T_t = bot.rot_and_pos_2_transform(R_t, p_t)
	>>> bot.set_tool_2_wrist_transform('right_arm', T_t)
	>>> bot.get_tool_2_wrist_transform()
	array([[ -1.,  0.,  0.,  .1],
	       [ 0.,  -1.,  0.,  .2],
	       [ 0.,  0.,  -1.,  .3],
	       [ 0.,  0.,  0.,   1.]])
	"""
	return self.kdl_2_numpy_transform(self.__get_tool_2_wrist_kdl(chain))
				      
    def __get_tool_2_wrist_kdl(self, chain):	
	chain_attr = getattr(self, chain)
	return chain_attr.T_wrist_tool
    
    def __get_base_2_parent_transform(self, chain):
	chain_attr = getattr(self, chain)
	return chain_attr.T_parent_base
    
    def __make_kdl_chain(self, chain):
	chain_attr = getattr(self, chain)
	
	# This is only intended to provide forward positions, velocities.  Ignoring inertial/mass 
	for i in range(len(chain_attr.a)):
	    frame = Frame()
	    frame = frame.DH_Craig1989(chain_attr.a[i], deg2rad(chain_attr.alpha[i]), chain_attr.d[i], deg2rad(chain_attr.joint_offset[i]))
	    if i==0:
		chain_attr.kdl_chain.addSegment(Segment(Joint(Joint.None),frame))
	    else:
		chain_attr.kdl_chain.addSegment(Segment(Joint(Joint.RotZ), frame))
	
	#self.jjsolver = ChainJntToJacSolver(self.kdl_chain)
	#self.fksolver_vel = ChainFkSolverVel_recursive(self.kdl_chain)
	
	chain_attr.fksolver_pos = ChainFkSolverPos_recursive(chain_attr.kdl_chain)	
	

	
    def read_config(self):
	M3Component.read_config(self)
	right_arm_model = None
	left_arm_model = None
		
	try:
            f=file(self.config_name,'r')
            self.config= yaml.safe_load(f.read())
        except (IOError, EOFError):
            print 'Config file not present:',self.config_name
            return
	# Parse the humanoid config
        if self.config['chains'].has_key('right_arm'):
	    self.available_chains.append('right_arm')
            self.right_arm.chain_name = self.config['chains']['right_arm']['chain_component']	    
	    self.__set_base_offset_tool_transforms('right_arm', self.config['chains']['right_arm'])
	    right_arm_model = self.config['chains']['right_arm']['arm_model']
	    
	if self.config['chains'].has_key('left_arm'):
	    self.available_chains.append('left_arm')
            self.left_arm.chain_name = self.config['chains']['left_arm']['chain_component']	    
	    self.__set_base_offset_tool_transforms('left_arm', self.config['chains']['left_arm'])
	    left_arm_model = self.config['chains']['left_arm']['arm_model']
	    
	if self.config['chains'].has_key('torso'):
	    self.available_chains.append('torso')
            self.torso.chain_name = self.config['chains']['torso']['chain_component']
	    self.__set_base_offset_tool_transforms('torso', self.config['chains']['torso'])
	    
	if self.config['chains'].has_key('head'):
	    self.available_chains.append('head')
            self.head.chain_name = self.config['chains']['head']['chain_component']
	    self.__set_base_offset_tool_transforms('head', self.config['chains']['head'])
	    
	# Now configure
	if self.right_arm.chain_name is not None:
	    self.right_arm.ndof = self.__get_ndof_from_config('right_arm')
	    self.right_arm.max_slew_rates = self.__get_max_slew_rates_from_config('right_arm')	    
	    self.__set_params_from_config('right_arm')
	    self.__grow_command_message('right_arm')
	    self.__load_dh_from_kinefile('right_arm')
	    self.__make_kdl_chain('right_arm')
	    frame = Frame()	    
	    frame = frame.DH_Craig1989(self.right_arm.a[-1], deg2rad(self.right_arm.alpha[-1]), 
			       self.right_arm.d[-1], deg2rad(self.right_arm.joint_offset[-1]))	    
	    self.__set_joints_max_min_deg('right_arm')
	    self.right_arm.ik = M3IKChain(config=right_arm_model, end_link_transform = frame,
					  joints_max_deg = self.right_arm.joints_max_deg, joints_min_deg = self.right_arm.joints_min_deg)
	    self.right_arm.ik_axis = M3IKAxis(config=right_arm_model, end_link_transform = frame,
					  joints_max_deg = self.right_arm.joints_max_deg, joints_min_deg = self.right_arm.joints_min_deg)
	    
	if self.left_arm.chain_name is not None:	    
	    self.left_arm.ndof = self.__get_ndof_from_config('left_arm')	    	    
	    self.left_arm.max_slew_rates = self.__get_max_slew_rates_from_config('left_arm')
	    self.__set_params_from_config('left_arm')
	    self.__grow_command_message('left_arm')
	    self.__load_dh_from_kinefile('left_arm')
	    self.__make_kdl_chain('left_arm')
	    frame = Frame()	    
	    frame = frame.DH_Craig1989(self.left_arm.a[-1], deg2rad(self.left_arm.alpha[-1]), 
			       self.left_arm.d[-1], deg2rad(self.left_arm.joint_offset[-1]))	    
	    self.__set_joints_max_min_deg('left_arm')	    
	    self.left_arm.ik = M3IKChain(config=left_arm_model, end_link_transform = frame,
					  joints_max_deg = self.left_arm.joints_max_deg, joints_min_deg = self.left_arm.joints_min_deg)
	    self.left_arm.ik_axis = M3IKAxis(config=left_arm_model, end_link_transform = frame,
					  joints_max_deg = self.left_arm.joints_max_deg, joints_min_deg = self.left_arm.joints_min_deg)
	    
    	if self.torso.chain_name is not None:
	    self.torso.ndof = self.__get_ndof_from_config('torso')
	    self.torso.max_slew_rates = self.__get_max_slew_rates_from_config('torso')	
	    self.__set_params_from_config('torso')
	    self.__grow_command_message('torso')
	    self.__load_dh_from_kinefile('torso')
	    self.__make_kdl_chain('torso')
	    self.__set_joints_max_min_deg('torso')
	    
	if self.head.chain_name is not None:
	    self.head.ndof = self.__get_ndof_from_config('head')	    
	    self.head.max_slew_rates = self.__get_max_slew_rates_from_config('head')	
	    self.__set_params_from_config('head')
	    self.__grow_command_message('head')
	    self.__load_dh_from_kinefile('head')
	    self.__make_kdl_chain('head')
	    self.__set_joints_max_min_deg('head')
	    

    def get_joints_max_deg(self, chain):
	chain_attr = getattr(self, chain)
	return chain_attr.joints_max_deg[:]
	    
    def get_joints_min_deg(self, chain):
	chain_attr = getattr(self, chain)
	return chain_attr.joints_min_deg[:]
    
    def get_joint_names(self, chain):
	chain_attr = getattr(self, chain)
	return chain_attr.joint_names[:]
    
    def get_chain_component_name(self,chain):
	chain_attr = getattr(self, chain)
	return chain_attr.chain_name
    
    def __set_joints_max_min_deg(self, chain):
	chain_attr = getattr(self, chain)
	file_name = m3t.get_component_config_filename(chain_attr.chain_name)
	
	try:
            f=file(file_name,'r')
            config= yaml.safe_load(f.read())
        except (IOError, EOFError):
            print 'Config file not present:',file_name
            return

	chain_attr.joints_max_deg = [0]*self.get_num_dof(chain)
	chain_attr.joints_min_deg = [0]*self.get_num_dof(chain)
	chain_attr.joint_names = ['']*self.get_num_dof(chain)
	for k in config['joint_components']:
	    joint_name = config['joint_components'][k]
	    joint_file_name = m3t.get_component_config_filename(joint_name)
	    try:
		jf=file(joint_file_name,'r')
		joint_config= yaml.safe_load(jf.read())
	    except (IOError, EOFError):
		print 'Config file not present:', joint_file_name
		return
	    
	    chain_attr.joint_names[int(k[1:])] = joint_name
	    chain_attr.joints_max_deg[int(k[1:])] = joint_config['param']['max_q']
	    chain_attr.joints_min_deg[int(k[1:])] = joint_config['param']['min_q']	
	    
    def __get_max_slew_rates_from_config(self, chain):
	chain_attr = getattr(self, chain)
	file_name = m3t.get_component_config_filename(chain_attr.chain_name)
	
	try:
            f=file(file_name,'r')
            config= yaml.safe_load(f.read())
        except (IOError, EOFError):
            print 'Config file not present:',file_name
            return

	max_slew_rates = [0.0]*self.get_num_dof(chain)
	
	for k in config['joint_components']:
	    joint_name = config['joint_components'][k]
	    joint_file_name = m3t.get_component_config_filename(joint_name)
	    try:
		jf=file(joint_file_name,'r')
		joint_config= yaml.safe_load(jf.read())
	    except (IOError, EOFError):
		print 'Config file not present:', joint_file_name
		return
	    
	    max_slew_rates[int(k[1:])] = joint_config['param']['max_q_slew_rate']
	    
	return max_slew_rates
	
    def __set_base_offset_tool_transforms(self, chain, config):	
	offset_trans = []
	offset_rot = []
	tool_trans = []
	tool_rot = []		
	chain_attr = getattr(self, chain)
	
	
	for i in range(len(config['base_rotation_in_parent'])):
	    offset_rot.append(config['base_rotation_in_parent'][i])
	    
	for i in range(len(config['base_translation_in_parent'])):
	    offset_trans.append(config['base_translation_in_parent'][i])

	offset_rot_kdl = Rotation()
	offset_trans_kdl = Vector()
	offset_trans_kdl = self.list_to_kdl_vector(offset_trans)	
	offset_rot_kdl = self.list_to_kdl_rotation(offset_rot)
	chain_attr.T_parent_base = Frame(offset_rot_kdl, offset_trans_kdl)
		
	if config.has_key('tool_translation_in_end') and config.has_key('tool_rotation_in_end'):	    
	    for i in range(len(config['tool_translation_in_end'])):
		tool_trans.append(config['tool_translation_in_end'][i])
	
	    for i in range(len(config['tool_rotation_in_end'])):
		tool_rot.append(config['tool_rotation_in_end'][i])			
				
	    tool_rot_kdl = Rotation()
	    tool_trans_kdl = Vector()
	    
	    tool_trans_kdl = self.list_to_kdl_vector(tool_trans)
    
	    tool_rot_kdl = self.list_to_kdl_rotation(tool_rot)
			
	    chain_attr.T_wrist_tool = Frame(tool_rot_kdl, tool_trans_kdl)	    
	
    def __get_T_base_wrist(self, chain, tool_pos, tool_rot):	
	T_world_tool = Frame(self.numpy_to_kdl_rotation(tool_rot), self.list_to_kdl_vector(tool_pos))
	if self.torso.chain_name != None:
	    T_base_world = (self.__get_tool_2_world_kdl('torso') * self.__get_T_parent_base(chain)).Inverse()
	else:
	    T_base_world = self.__get_T_parent_base(chain).Inverse()
	T_base_wrist = T_base_world * T_world_tool * self.__get_T_wrist_tool(chain).Inverse()
	return T_base_wrist
    
    def __get_T_base_wrist_sim(self, chain, tool_pos, tool_rot):	
	T_world_tool = Frame(self.numpy_to_kdl_rotation(tool_rot), self.list_to_kdl_vector(tool_pos))
	if self.torso.chain_name != None:
	    T_base_world = (self.__get_tool_2_world_kdl_sim('torso') * self.__get_T_parent_base(chain)).Inverse()
	else:
	    T_base_world = self.__get_T_parent_base(chain).Inverse()	
	
	T_base_wrist = T_base_world * T_world_tool * self.__get_T_wrist_tool(chain).Inverse()
	return T_base_wrist
	
    def __get_T_wrist_tool(self, chain):
	chain_attr = getattr(self, chain)
	return chain_attr.T_wrist_tool
	
    
    def __get_T_parent_base(self, chain):
	chain_attr = getattr(self, chain)
	return chain_attr.T_parent_base
    
    def __get_P_base_wrist(self, chain, tool_pos):
	P_world_tool = self.list_to_kdl_vector(tool_pos)
	if self.torso.chain_name != None:
	    T_base_world = (self.__get_tool_2_world_kdl('torso') * self.__get_T_parent_base(chain)).Inverse()
	else:
	    T_base_world = self.__get_T_parent_base(chain).Inverse()
	P_base_wrist = self.__get_T_wrist_tool(chain).Inverse() * T_base_world * P_world_tool
	return P_base_wrist
        
    def get_tool_position_rpy_2_theta_deg(self, chain, tool_pos, tool_rpy_deg, theta_deg_out):
	"""
	Gets a set of joint angles to make the arm's tool frame origin coincident with the tool position specified
	and also orient the tool frame according to the roll, pitch, and yaw values specified in degrees.
	Returns false if no solution can be found.
					
	:param chain: Desired chain to use for tool frame.
	:type chain: 'right_arm', 'left_arm'
	:param tool_pos: Desired tool frame origin in meters specified in world coordinate system.
	:type tool_pos: array_like, shape (3)
	:param tool_rpy_deg: roll, pitch, and yaw of tool frame in degrees for chain with respect to world frame.
	:type tool_rpy_deg: array_like, shape (3)
	:param theta_deg_out: Solution with Euclidian distance closest to current theta values
	:type theta_deg_out: array, shape (ndof)	
	:returns: A boolean value indicating if a solution could be found
	:rtype:	bool
	
	.. Note:: 
	   The tool position must be meters with respect to world coordinate system.
	   
	   * -180 <= Roll <= 180
	   * -90 <= Pitch <= 90
	   * -180 <= Yaw  <= 180
	   
	   Joint solution (if found) is passed back through the parameter theta_deg_out.
	
	   Roll, pitch, and yaw angles in this context are considered X-Y-Z fixed angles around a fixed
	   reference frame (the world frame).  Starting with the tool frame coincident with the world frame,
	   the RPY values returned describe the tool frame orientation as follows:
	   
	   * First rotate the tool frame by roll angle around X-axis of world frame.
	   * Next rotate the tool frame by pitch angle around Y-axis of world frame.	   
	   * Finally rotate the tool frame by yaw angle around Z-axis of world frame.	   	
	
	:raises: M3Exception if chain is not supported or tool_pos.shape is not (3) or tool_rpy.shape is not (3) or
	   RPY angles are outside of bounds.
	
	:See Also: 
	   :meth:`M3Humanoid.get_tool_position_rpy_2_theta_rad`
	   :meth:`M3Humanoid.get_tool_axis_2_theta_deg`
	   
	:Examples:
	
	To move the right arm's tool 10 cm upwards in the Z-axis of the world frame and keep it's current orientation:
		
	>>> from m3.humanoid import M3Humanoid
	>>> import numpy as np	
	>>> bot = M3Humanoid('bob')
	>>> p = bot.get_tool_position('right_arm') 
	>>> rpy = bot.get_tool_rpy('right_arm')  	
	>>> p[2] += 0.1  # move position up 10 cm	
	>>> theta_soln = []
	>>> success = bot.get_tool_position_rpy_2_theta_deg('right_arm', p, rpy, theta_soln)	
	>>> if success: bot.set_theta_deg('right_arm', theta_soln)	
	"""
	self.__assert_rpy_deg(tool_rpy_deg)
	theta_soln = []
	success = self.get_tool_position_rpy_2_theta_rad(chain, tool_pos, deg2rad(nu.array(tool_rpy_deg,float)), theta_soln)
	if success:
	    theta_deg_out[:] = rad2deg(nu.array(theta_soln,float))
	return success
    
    def __assert_rpy_rad(self, rpy_rad):
	'''
	 * -180 <= Roll <= 180
	   * -90 <= Pitch <= 90
	   * -180 <= Yaw  <= 180
	'''
	if (rpy_rad[0] < -nu.pi) or (rpy_rad[0] > nu.pi):
	    raise m3t.M3Exception('Roll value must be between -Pi and Pi.')
	
	if (rpy_rad[1] < -nu.pi/2) or (rpy_rad[1] > nu.pi/2):
	    raise m3t.M3Exception('Pitch value must be between -Pi/2 and Pi/2.')
	
	if (rpy_rad[2] < -nu.pi) or (rpy_rad[2] > nu.pi):
	    raise m3t.M3Exception('Yaw value must be between -Pi and Pi.')
	
	
    def __assert_rpy_deg(self, rpy_deg):
	'''
	 * -180 <= Roll <= 180
	   * -90 <= Pitch <= 90
	   * -180 <= Yaw  <= 180
	'''
	if (rpy_deg[0] < -180.) or (rpy_deg[0] > 180.):
	    raise m3t.M3Exception('Roll value must be between -180 and 180.')
	
	if (rpy_deg[1] < -90.) or (rpy_deg[1] > 90.):
	    raise m3t.M3Exception('Pitch value must be between -90 and 90.')
	
	if (rpy_deg[2] < -180.) or (rpy_deg[2] > 180.):
	    raise m3t.M3Exception('Yaw value must be between -180 and 180.')
	    
	
    
    def get_tool_position_rpy_2_theta_deg_sim(self, chain, tool_pos, tool_rpy_deg, theta_deg_out):
	'''
	Assumes RPY angles are in radians and returns joint angles in radians.
	:See Also: 
	   :meth:`M3Humanoid.get_tool_position_rpy_2_theta_deg`
	'''	
	theta_soln = []
	success = self.get_tool_position_rpy_2_theta_rad_sim(chain, tool_pos, deg2rad(nu.array(tool_rpy_deg,float)), theta_soln)
	if success:
	    theta_deg_out[:] = rad2deg(nu.array(theta_soln,float))
	return success	
    
    def get_tool_position_rpy_2_theta_rad(self, chain, tool_pos, tool_rpy_rad, theta_rad_out):
	'''
	Assumes RPY angles are in radians and returns joint angles in radians.
	:See Also: 
	   :meth:`M3Humanoid.get_tool_position_rpy_2_theta_deg`
	'''	
	theta_rad = self.get_theta_rad(chain)
	
	success = self.__get_tool_position_rpy_2_theta_rad(chain, tool_pos, tool_rpy_rad, theta_rad, theta_rad_out)		
	
	return success
    
    def get_tool_position_rpy_2_theta_rad_sim(self, chain, tool_pos, tool_rpy_rad, theta_rad_out):	
	'''
	Assumes RPY angles are in radians and returns joint angles in radians.
	:See Also: 
	   :meth:`M3Humanoid.get_tool_position_rpy_2_theta_deg`
	'''	
	theta_rad = self.get_theta_sim_rad(chain)
	success = self.__get_tool_position_rpy_2_theta_rad_sim(chain, tool_pos, tool_rpy_rad, theta_rad, theta_rad_out)
	return success
    
    #same as above, except taking theta as input 
    def __get_tool_position_rpy_2_theta_rad(self, chain, tool_pos, tool_rpy_rad, theta_rad_in, theta_rad_out):
	self.__assert_chain(chain)
	self.__assert_list_size(tool_pos, 3)
	self.__assert_list_size(tool_rpy_rad, 3)		
	tool_rot_kdl = Rotation.RPY(tool_rpy_rad[0], tool_rpy_rad[1], tool_rpy_rad[2])	
	tool_rot = self.kdl_to_numpy_rotation(tool_rot_kdl)
	chain_attr = getattr(self, chain)
	    
	if self.torso.chain_name != None:
	    T_base_2_world = self.__get_tool_2_world_kdl('torso') * self.__get_T_parent_base(chain)
	else:
	    T_base_2_world = self.__get_T_parent_base(chain)
	
	T_tool_2_wrist = self.__get_T_wrist_tool(chain)
	
	theta_deg = rad2deg(theta_rad_in)
	
	solution = chain_attr.ik_axis.find_ik(tool_pos, tool_rot, theta_deg, theta_deg[2], T_tool_2_wrist, T_base_2_world)
	theta_rad_out[:] = deg2rad(nu.array(solution[1],float))	
	return solution[0]
    
    def __get_tool_position_rpy_2_theta_rad_sim(self, chain, tool_pos, tool_rpy_rad, theta_rad_in, theta_rad_out):
	self.__assert_chain(chain)
	self.__assert_list_size(tool_pos, 3)
	self.__assert_list_size(tool_rpy_rad, 3)		
	tool_rot_kdl = Rotation.RPY(tool_rpy_rad[0], tool_rpy_rad[1], tool_rpy_rad[2])	
	tool_rot = self.kdl_to_numpy_rotation(tool_rot_kdl)
	chain_attr = getattr(self, chain)
	    
	if self.torso.chain_name != None:
	    T_base_2_world = self.__get_tool_2_world_kdl_sim('torso') * self.__get_T_parent_base(chain)
	else:
	    T_base_2_world = self.__get_T_parent_base(chain)
	
	T_tool_2_wrist = self.__get_T_wrist_tool(chain)
	
	theta_deg = rad2deg(theta_rad_in)
	
	solution = chain_attr.ik_axis.find_ik(tool_pos, tool_rot, theta_deg, theta_deg[2], T_tool_2_wrist, T_base_2_world)
	theta_rad_out[:] = deg2rad(nu.array(solution[1],float))	
	return solution[0]
	
    def __get_tool_axis_2_theta_rad(self, chain, tool_pos, tool_axis, theta_rad_in, theta_rad_out):	
	self.__assert_chain(chain)
	self.__assert_list_size(tool_pos,3)
	self.__assert_list_size(tool_axis,3)	
	chain_attr = getattr(self, chain)
	
	if self.torso.chain_name != None:
	    T_base_2_world = self.__get_tool_2_world_kdl('torso') * self.__get_T_parent_base(chain)
	else:
	    T_base_2_world = self.__get_T_parent_base(chain)
	
	T_tool_2_wrist = self.__get_T_wrist_tool(chain)
	
	theta_deg = rad2deg(theta_rad_in)
	
	solution = chain_attr.ik_axis.find_ik_axis(tool_pos, tool_axis, theta_deg, theta_deg[2], False, 0, T_tool_2_wrist, T_base_2_world)
	theta_rad_out[:] = deg2rad(nu.array(solution[1],float))	
	return solution[0]
    
    def __get_tool_axis_2_theta_rad_sim(self, chain, tool_pos, tool_axis, theta_rad_in, theta_rad_out):	
	self.__assert_chain(chain)
	self.__assert_list_size(tool_pos,3)
	self.__assert_list_size(tool_axis,3)	
	chain_attr = getattr(self, chain)
	
	if self.torso.chain_name != None:
	    T_base_2_world = self.__get_tool_2_world_kdl_sim('torso') * self.__get_T_parent_base(chain)
	else:
	    T_base_2_world = self.__get_T_parent_base(chain)
	
	T_tool_2_wrist = self.__get_T_wrist_tool(chain)
	
	theta_deg = rad2deg(theta_rad_in)
	
	solution = chain_attr.ik_axis.find_ik_axis(tool_pos, tool_axis, theta_deg, theta_deg[2], False, 0, T_tool_2_wrist, T_base_2_world)
	theta_rad_out[:] = deg2rad(nu.array(solution[1],float))	
	return solution[0]
    
    def get_tool_axis_2_theta_deg_sim(self, chain, tool_pos, tool_axis, theta_deg_out):
	theta_rad_in = self.get_theta_sim_rad(chain)
	theta_soln_rad = []
	success = self.__get_tool_axis_2_theta_rad_sim(chain, tool_pos, tool_axis, theta_rad_in, theta_soln_rad)
	if success:	    
	    theta_deg_out[:] = rad2deg(nu.array(theta_soln_rad,float))
	return success
	    
    def get_tool_axis_2_theta_deg(self, chain, tool_pos, tool_axis, theta_deg_out):
	"""
	Gets a set of joint angles that will make the arm's tool frame origin coincident with position specified in world frame
	and also such that the X-axis of tool frame is coincident with the axis specified in world coordinate system.	
	Returns false if no solution can be found for the desired tool frame position and axis.
					
	:param chain: Desired chain to use for tool frame.
	:type chain: 'right_arm', 'left_arm'
	:param tool_pos: Desired tool frame origin in meters specified in world coordinate system.
	:type tool_pos: array_like, shape (3)
	:param tool_axis: Desired tool frame X-axis specified in world coordinate system.
	:type tool_axis: array_like, shape (3)
	:param theta_deg_out: Solution with Euclidian distance closest to current theta values
	:type theta_deg_out: array, shape (ndof)
	
	:returns: A boolean value indicating if a solution could be found
	:rtype:	bool
	
	.. Note::
	   The tool frame parameters must be defined with respect to the world frame.  The translational component must be in meters.
	   Joint solution (if found) is passed back through the parameter theta_deg_out.	
	
	:raises: M3Exception if chain is not supported or tool_pos.shape is not (3) or tool_axis.shape is not (3)
	
	:See Also: 
	   :meth:`M3Humanoid.get_tool_position_rpy_2_theta_deg`
	   
	:Examples:
	
	To move the right arm's tool X-axis to be coincident with the world frame Z-axis and keep the tool
	at it's current position:
		
	>>> from m3.humanoid import M3Humanoid
	>>> import numpy as np	
	>>> bot = M3Humanoid('bob')
	>>> p = bot.get_tool_position('right_arm') # current tool position w.r.t. world		
	>>> axis = [0, 0, 1]	
	>>> theta_soln = []
	>>> success = bot.get_tool_axis_2_theta_deg('right_arm', tool_pos, tool_axis, theta_soln)
	>>> if success: bot.set_theta_deg('right_arm', theta_soln)
	
	"""
	theta_rad_in = self.get_theta_rad(chain)
	theta_soln_rad = []
	success = self.__get_tool_axis_2_theta_rad(chain, tool_pos, tool_axis, theta_rad_in, theta_soln_rad)
	if success:
	    theta_deg_out[:] = rad2deg(nu.array(theta_soln_rad,float))
	return success
    def list_to_kdl_vector(self, t):
	self.__assert_list_size(t, 3)
	vec_kdl = Vector()
	for i in range(3):
	    vec_kdl[i] = t[i]
	return vec_kdl
	    
    def kdl_to_list_vector(self, t):
	vec = [0]*3
	for i in range(3):
	    vec[i] = t[i]
	return vec
	    
    def list_to_kdl_rotation(self, l):
	self.__assert_list_size(l, 9)
	return self.numpy_to_kdl_rotation(self.list_to_numpy_rotation(l))
    
    def __assert_matrix_size(self, m, rows, cols):
	
	if len(m) != rows:
	    raise m3t.M3Exception('Data must be a list or array of size '+ str(rows) +'x' + str(cols) + ' but rows = ' + str(len(m)))
	if len(m[0]) != cols:
	    raise m3t.M3Exception('Data must be a list or array of size ' + str(rows) + 'x' + str(cols) + ' but cols = ' + str(len(m[0])))
	
	
    def __assert_list_size(self, l, n):
	
	if len(l) != n:
	    raise m3t.M3Exception('Data must be a list or array of length ' + str(n) + ' but size is ' + str(len(l)))	
	
    
    # copying manually because KDL::Rotation doesn't support slicing
    def numpy_to_kdl_rotation(self, m):
	mtx_kdl = Rotation(m[0,0], m[0,1], m[0,2], 
			   m[1,0], m[1,1], m[1,2],
			   m[2,0], m[2,1], m[2,2])	
	return mtx_kdl
    
    # copying manually because KDL::Rotation doesn't support slicing
    def kdl_to_numpy_rotation(self, mtx_kdl):
	mtx = nu.zeros([3,3])
	mtx[0,0] = mtx_kdl[0,0]
	mtx[0,1] = mtx_kdl[0,1]
	mtx[0,2] = mtx_kdl[0,2]
	mtx[1,0] = mtx_kdl[1,0]
	mtx[1,1] = mtx_kdl[1,1]
	mtx[1,2] = mtx_kdl[1,2]
	mtx[2,0] = mtx_kdl[2,0]
	mtx[2,1] = mtx_kdl[2,1]
	mtx[2,2] = mtx_kdl[2,2]
	return mtx
    
    def list_to_numpy_rotation(self, l):
	mtx = mtx = nu.zeros([3,3])
	mtx[0,0] = l[0]
	mtx[0,1] = l[1]
	mtx[0,2] = l[2]
	mtx[1,0] = l[3]
	mtx[1,1] = l[4]
	mtx[1,2] = l[5]
	mtx[2,0] = l[6]
	mtx[2,1] = l[7]
	mtx[2,2] = l[8]
	return mtx
	
    def __get_ndof_from_config(self, chain):
	
	chain_attr = getattr(self, chain)
	file_name = m3t.get_component_config_filename(chain_attr.chain_name)
	
	try:
            f=file(file_name,'r')
            config= yaml.safe_load(f.read())
        except (IOError, EOFError):
            print 'Config file not present:',file_name
            return
	
	return config['ndof']
    
    def __set_params_from_config(self, chain):	
	chain_attr = getattr(self, chain)
	param = self.get_param(chain)
	file_name = m3t.get_component_config_filename(chain_attr.chain_name)		
	
	try:
            f=file(file_name,'r')
            config= yaml.safe_load(f.read())
        except (IOError, EOFError):
            print 'Config file not present:',file_name
            return
	
	name = config['dynamatics_component']

	if name == "":
	    print "Could not find dynamatics component for ", chain
	    return
	
	file_name = m3t.get_component_config_filename(name)
	
	try:
            f=file(file_name,'r')
            config= yaml.safe_load(f.read())
        except (IOError, EOFError):
            print 'Config file not present:',file_name
            return
		
	param.payload_mass = config['param']['payload_mass']
	param.payload_com[:] = config['param']['payload_com'][:]
	param.payload_inertia[:] = config['param']['payload_inertia'][:]
	param.use_velocities = config['param']['use_velocities']
	param.use_accelerations = config['param']['use_accelerations']
    
    def __load_dh_from_kinefile(self, chain):
	
	chain_attr = getattr(self, chain)
	file_name = m3t.get_component_config_filename(chain_attr.chain_name)
	
	try:
            f=file(file_name,'r')
            config= yaml.safe_load(f.read())
        except (IOError, EOFError):
            print 'Config file not present:',file_name
            return
	
	name = config['dynamatics_component']

	if name == "":
	    print "Could not find dynamatics component for ", chain
	    return
	
	file_name = m3t.get_component_config_filename(name)
	
	try:
            f=file(file_name,'r')
            config= yaml.safe_load(f.read())
        except (IOError, EOFError):
            print 'Config file not present:',file_name
            return
	
	for i in range(len(config['links'])):
	    #print "link type = ", type(links[k]), " (should be dict)"
	    #if type(config['links'][i])==dict: #repeated field	    	    
	    chain_attr.a.append(config['links'][i]['a'])	    
	    chain_attr.alpha.append(config['links'][i]['alpha'])	    
	    chain_attr.d.append(config['links'][i]['d'])	    
	    chain_attr.joint_offset.append(config['links'][i]['joint_offset'])
	    		
	
    def get_status(self, chain):
	self.__assert_chain(chain)
	return getattr(self.status, chain)
        
    
    def get_command(self, chain):
	self.__assert_chain(chain)
	return getattr(self.command, chain)
    
    def get_param(self, chain):
	self.__assert_chain(chain)
	return getattr(self.param, chain)
    
        		
    def __grow_command_message(self, chain):	
	chain_attr = getattr(self, chain)
	for i in range(self.get_num_dof(chain)):	    
	    self.get_command(chain).tq_desired.append(0)
	    self.get_command(chain).q_desired.append(0)
	    self.get_command(chain).qdot_desired.append(0)
	    self.get_command(chain).q_stiffness.append(0)
	    self.get_command(chain).q_slew_rate.append(0)
	    self.get_command(chain).pwm_desired.append(0)
	    self.get_command(chain).ctrl_mode.append(mab.JOINT_ARRAY_MODE_OFF)
	    self.get_command(chain).smoothing_mode.append(msm.SMOOTHING_MODE_OFF)	
				
    def set_motor_power_on(self):
	"""
	Enables power supply for motor controllers.
	
	:See Also: 
	   :meth:`M3Humanoid.set_motor_power_off`
	"""
	self.command.enable_motor=True
	
    def set_motor_power_off(self):
	"""
	Disables power supply for motor controllers.
	
	:See Also: 
	   :meth:`M3Humanoid.set_motor_power_on`
	"""
	self.command.enable_motor=False    
	    
    def get_num_dof(self, chain):
	"""
	Gets number of degrees of freedom for a chain.
	
	:param chain: Desired chain.
	:type chain: 'right_arm', 'left_arm', 'torso', 'head'
	:returns: Number of degrees of freedom for selected chain.
	:rtype:	int
	
	:raises: M3Exception if chain is not supported.
	"""
	chain_attr = getattr(self, chain)
	return chain_attr.ndof
    
    def get_torque(self, chain):
	"""
	Gets torque values of all joints in mN*m for a chain.
	
	:param chain: Desired chain.
	:type chain: 'right_arm', 'left_arm', 'torso', 'head'
	:returns: Torque values in mN*m for selected chain.
	:rtype:	array, shape (ndof)
	
	:raises: M3Exception if chain is not supported.
	"""
	return nu.array(self.get_status(chain).torque,float)        
    
    def get_torquedot(self, chain):
	"""
	Gets torque time derivative values of all joints in mN*m/sec for a chain.
	
	:param chain: Desired chain.
	:type chain: 'right_arm', 'left_arm', 'torso', 'head'
	:returns: Torque derivative values in mN*m/sec for selected chain.
	:rtype:	array, shape (ndof)
	
	:raises: M3Exception if chain is not supported.
	"""
	return nu.array(self.get_status(chain).torquedot, nu.float32)
    
    def get_theta_deg(self, chain):
	"""
	Gets joint values in degrees for selected chain.  
	
	:param chain: Desired chain.
	:type chain: 'right_arm', 'left_arm', 'torso', 'head'
	:returns: Joint values in degrees for chain.
	:rtype: array, shape (ndof)
	
	:raises: M3Exception if chain is not supported
	
	:See Also: 
	   :meth:`M3Humanoid.get_theta_rad`	   
	   :meth:`M3Humanoid.set_theta_deg`
	   :meth:`M3Humanoid.get_theta_sim_deg`
	   :meth:`M3Humanoid.get_thetadot_deg`
	   :meth:`M3Humanoid.get_thetadotdot_deg`
	"""
	return nu.array(self.get_status(chain).theta, nu.float32)
    
    def get_theta_sim_deg(self, chain):
	"""
	Gets artificial joint values in degrees for selected chain.  
	
	:param chain: Desired chain.
	:type chain: 'right_arm', 'left_arm', 'torso', 'head'
	:returns: Artificial joint values in degrees for chain.
	:rtype: array, shape (ndof)
	
	:raises: M3Exception if chain is not supported
	
	:See Also: 
	   :meth:`M3Humanoid.get_theta_sim_rad`	   
	   :meth:`M3Humanoid.set_theta_sim_deg`
	   :meth:`M3Humanoid.get_theta_deg`	
	"""	
	chain_attr = getattr(self, chain)
	return nu.array(chain_attr.theta_sim,float)
    
    
    
    def get_theta_sim_rad(self, chain):
	"""
	Gets artificial joint values in radians for selected chain.  
	
	:param chain: Desired chain.
	:type chain: 'right_arm', 'left_arm', 'torso', 'head'
	:returns: Artificial joint values in radians for chain.
	:rtype: array, shape (ndof)
	
	:raises: M3Exception if chain is not supported
	
	:See Also: 
	   :meth:`M3Humanoid.get_theta_sim_deg`	   
	   :meth:`M3Humanoid.set_theta_sim_rad`
	   :meth:`M3Humanoid.get_theta_rad`
	"""	
	return deg2rad(self.get_theta_sim_deg(chain))
    
    def get_theta_rad(self, chain):
	"""
	Gets joint values in radians for selected chain.  
	
	:param chain: Desired chain.
	:type chain: 'right_arm', 'left_arm', 'torso', 'head'
	:returns: Joint values in radians for chain.
	:rtype: array, shape (ndof)
	
	:raises: M3Exception if chain is not supported
	
	:See Also: 
	   :meth:`M3Humanoid.get_theta_deg`	   
	   :meth:`M3Humanoid.set_theta_rad`
	   :meth:`M3Humanoid.get_theta_sim_rad`
	   :meth:`M3Humanoid.get_thetadot_rad`
	   :meth:`M3Humanoid.get_thetadotdot_rad`
	"""
	return deg2rad(self.get_theta_deg(chain))
    
    def get_pwm(self, chain):
	"""
	Gets motor commanded pwm duty cycle percentage for selected chain.  
	
	:param chain: Desired chain.
	:type chain: 'right_arm', 'left_arm', 'torso', 'head'
	:returns: PWM duty cycle percentages.
	:rtype: array, shape (ndof)
	
	:raises: M3Exception if chain is not supported
	
	:See Also: 
	   :meth:`M3Humanoid.set_pwm`	   
	"""
	return nu.array(self.get_status(chain).pwm_cmd,float)

    
    def get_thetadot_rad(self, chain):
	"""
	Gets joint velocity values in radians/second for selected chain.  
	
	:param chain: Desired chain.
	:type chain: 'right_arm', 'left_arm', 'torso', 'head'
	:returns: Joint velocity values in radians/second for chain.
	:rtype: array, shape (ndof)
	
	:raises: M3Exception if chain is not supported
	
	:See Also: 
	   :meth:`M3Humanoid.get_thetadot_deg`	   
	   :meth:`M3Humanoid.get_theta_rad`
	   :meth:`M3Humanoid.get_thetadotdot_rad`
	"""
	return deg2rad(self.get_thetadot_deg(chain))
    
    def get_thetadot_deg(self, chain):
	"""
	Gets joint velocity values in degrees/second for selected chain.  
	
	:param chain: Desired chain.
	:type chain: 'right_arm', 'left_arm', 'torso', 'head'
	:returns: Joint velocity values in degrees/second for chain.
	:rtype: array, shape (ndof)
	
	:raises: M3Exception if chain is not supported
	
	:See Also: 
	   :meth:`M3Humanoid.get_thetadot_rad`	   
	   :meth:`M3Humanoid.get_theta_deg`
	   :meth:`M3Humanoid.get_thetadotdot_deg`
	"""
	return nu.array(self.get_status(chain).thetadot,float)
    
    def get_thetadotdot_rad(self, chain):
	"""
	Gets joint acceleration values in radians/(sec^2) for selected chain.  
	
	:param chain: Desired chain.
	:type chain: 'right_arm', 'left_arm', 'torso', 'head'
	:returns: Joint acceleration values in radians/(sec^2) for chain.
	:rtype: array, shape (ndof)
	
	:raises: M3Exception if chain is not supported
	
	:See Also: 
	   :meth:`M3Humanoid.get_thetadotdot_deg`	   
	   :meth:`M3Humanoid.get_theta_rad`
	   :meth:`M3Humanoid.get_thetadot_rad`
	"""
	return deg2rad(self.get_thetadotdot_deg(chain))
    
    def get_thetadotdot_deg(self, chain):
	"""
	Gets joint acceleration values in degrees/(sec^2) for selected chain.  
	
	:param chain: Desired chain.
	:type chain: 'right_arm', 'left_arm', 'torso', 'head'
	:returns: Joint acceleration values in degrees/(sec^2) for chain.
	:rtype: array, shape (ndof)
	
	:raises: M3Exception if chain is not supported
	
	:See Also: 
	   :meth:`M3Humanoid.get_thetadotdot_rad`   
	   :meth:`M3Humanoid.get_theta_deg`
	   :meth:`M3Humanoid.get_thetadot_deg`
	"""
	return nu.array(self.get_status(chain).thetadotdot,float)
    
    def get_timestamp_uS(self):
	return self.status.base.timestamp

    def set_torque(self, chain, v, ind=None):
	"""
	Sets joint controller commands for selected chain to desired torque value in mN*m.  A list of joint indexes can be 
	supplied to set specific joint torques, or the index
	can be omitted if the length of v is equal to the number of degrees of freedom for that chain.
	
	.. Note:: Joint must be in torque or torque_gc mode for controller to track desired torque commands.
	
	:param chain: Desired chain.
	:type chain: 'right_arm', 'left_arm', 'torso', 'head'
	:param v: Joint torque commands in mN*m.
	:type v: array_like
	:param ind: Index of joints.
	:type ind: array_like, shape(len(v)) optional
	
	:raises: 
	   M3Exception if chain is not supported or v.shape is not (ndof) and
	   v.shape is not ind.shape and ind is not None.
	  	
	:See Also: 	   
	   :meth:`M3Humanoid.get_torque`
	   :meth:`M3Humanoid.set_mode_torque`
	   :meth:`M3Humanoid.set_mode_torque_gc`
	   
	:Examples:

	To set all joint torque commands with one list:
	
	>>> from m3.humanoid import M3Humanoid
	>>> bot = M3Humanoid('bob')
	>>> tq = [0., .1, .2, .3, .4, .3, .2]
	>>> bot.set_mode_torque_gc('right_arm')
	>>> bot.set_torque('right_arm', tq)
	"""
	if ind is not None:
	    self.__assert_list_size(v, len(ind))
	else:
	    chain_attr = getattr(self, chain)
	    self.__assert_list_size(v, chain_attr.ndof)	
	self.set_float_array(self.get_command(chain).tq_desired,nu.array(v,float),ind)
	
    def set_pwm(self, chain, v, ind=None):
	"""
	Sets joint controller commands for selected chain to desired pwm duty cycle percentage.  A list of joint indexes can be 
	supplied to set specific pwm duty cycles, or the index
	can be omitted if the length of v is equal to the number of degrees of freedom for that chain.
	
	.. Note:: Joint must be in pwm mode for controller to issue correct commands.
	
	:param chain: Desired chain.
	:type chain: 'right_arm', 'left_arm', 'torso', 'head'
	:param v: PWM duty cycle percentage (0-100)
	:type v: array_like
	:param ind: Index of joints.
	:type ind: array_like, shape(len(v)) optional
	
	:raises: 
	   M3Exception if chain is not supported or v.shape is not (ndof) and
	   v.shape is not ind.shape and ind is not None.
	  	
	:See Also: 	   
	   :meth:`M3Humanoid.get_pwm`
	   :meth:`M3Humanoid.set_mode_pwm`	   
	   
	:Examples:

	To set joint 3 pwm duty cycle to 50%:
	
	>>> from m3.humanoid import M3Humanoid
	>>> bot = M3Humanoid('bob')
	>>> pwm = [50]
	>>> dof = [3]
	>>> bot.set_mode_pwm('right_arm',dof)
	>>> bot.set_pwm('right_arm', pwm, dof)
	"""
	if ind is not None:
	    self.__assert_list_size(v, len(ind))
	else:
	    chain_attr = getattr(self, chain)
	    self.__assert_list_size(v, chain_attr.ndof)	
	self.set_float_array(self.get_command(chain).pwm_desired,nu.array(v,float),ind)	    	
	    
	
    def set_theta_rad(self, chain, v , ind=None):
	"""
	Sets joint controller commands for selected chain to desired angle.  A list of joint indexes can be 
	supplied to set specific joint angles, or the index
	can be omitted if the length of theta is equal to the number of degrees of freedom for that chain.
	
	.. Note:: Joint must be in theta or theta_gc mode for controller to track desired angle commands.
	
	:param chain: Desired chain.
	:type chain: 'right_arm', 'left_arm', 'torso', 'head'
	:param theta: Joint angle commands in radians.
	:type theta: array_like
	:param ind: Index of joints.
	:type ind: array_like, shape(len(theta)), optional
	
	:raises: 
	   M3Exception if chain is not supported or theta.shape is not (ndof) and
	   theta.shape is not ind.shape and ind is not None.
	  	
	:See Also: 
	   :meth:`M3Humanoid.set_theta_deg`	   
	   :meth:`M3Humanoid.get_theta_rad`
	   :meth:`M3Humanoid.set_theta_sim_rad`
	   :meth:`M3Humanoid.set_mode_theta`	   
	   :meth:`M3Humanoid.set_mode_theta_gc`	   
	   
	:Examples:

	To set all joint angle commands with one list:
	
	>>> from m3.humanoid import M3Humanoid
	>>> bot = M3Humanoid('bob')
	>>> q_a = [0., .1, .2, .3, .4, .3, .2]
	>>> bot.set_mode_theta_gc('right_arm')
	>>> bot.set_theta_rad('right_arm', q_a)	
	
	To change a subset of joint angle commands:
	
	>>> q_b = [.5, .6]
	>>> index = [3, 4]
	>>> bot.set_theta_rad('right_arm', q_b, index)	
	"""
	self.set_theta_deg(chain, deg2rad(nu.array(v,float)), ind)
		
    def set_theta_deg(self, chain, v, ind=None):
	"""
	Sets joint controller commands for selected chain to desired angle.  A list of joint indexes can be 
	supplied to set specific joint angles, or the index
	can be omitted if the length of theta is equal to the number of degrees of freedom for that chain.
	
	.. Note:: Joint must be in theta or theta_gc mode for controller to track desired angle commands.
	
	:param chain: Desired chain.
	:type chain: 'right_arm', 'left_arm', 'torso', 'head'
	:param theta: Joint angle commands in degrees.
	:type theta: array_like
	:param ind: Index of joints.
	:type ind: array_like, shape(len(theta)), optional
	
	:raises: chain
	   M3Exception if chain is not supported or theta.shape is not (ndof) and
	   theta.shape is not ind.shape and ind is not None.
	  	
	:See Also: 
	   :meth:`M3Humanoid.set_theta_rad`	   
	   :meth:`M3Humanoid.get_theta_deg`
	   :meth:`M3Humanoid.set_theta_sim_deg`
	   :meth:`M3Humanoid.set_mode_theta`	   
	   :meth:`M3Humanoid.set_mode_theta_gc`	  
	   
	:Examples:

	To set all joint angle commands with one list:
	
	>>> from m3.humanoid import M3Humanoid
	>>> bot = M3Humanoid('bob')
	>>> q_a = [0., 10., 20., 30., 40., 30., 20.]
	>>> bot.set_mode_theta_gc('right_arm')
	>>> bot.set_theta_deg('right_arm', q_a)	
	
	To change a subset of joint angle commands:
	
	>>> q_b = [50., 60.]
	>>> index = [3, 4]
	>>> bot.set_theta_deg('right_arm', q_b, index)	
	"""
	self.__assert_chain(chain)
	if ind is not None:
	    self.__assert_list_size(v, len(ind))
	else:
	    chain_attr = getattr(self, chain)
	    self.__assert_list_size(v, chain_attr.ndof)
	self.set_float_array(self.get_command(chain).q_desired,v,ind)
	
    def set_theta_proportion(self, chain, v, ind=None):
	thetas = []		
	
	
	max_theta = self.get_joints_max_deg(chain)
	min_theta = self.get_joints_min_deg(chain)
	
	if ind is not None:
		self.__assert_list_size(v, len(ind))
		for i in range(len(ind)):
			thetas.append(max_theta[ind[i]] - min_theta[ind[i]])
			thetas[-1] *= v[i]
			thetas[-1] = min_theta[ind[i]] + thetas[-1]
	else:			
		chain_attr = getattr(self, chain)
		self.__assert_list_size(v, chain_attr.ndof)		
		for i in range(len(v)):
			thetas.append(max_theta[i] - min_theta[i])
			thetas[-1] *= v[i]
			thetas[-1] = min_theta[i] + thetas[-1]
			#print "desired theta " + str(i) + " = " + str(thetas[-1]) + ", " + str(v[i])
		#print
		#for i,th in enumerate(self.get_theta_deg(chain)):
		#	print "measured theta " + str(i) + " = " + str(th) 
		#print
	
	
	self.set_theta_deg(chain, thetas, ind)
	#self.set_float_array(self.command.chain.desired_theta, thetas, ind)
	
	
    def set_thetadot_rad(self, chain, v , ind=None):
	"""
	Sets joint controller commands for selected chain to desired velocity (rad/s).  A list of joint indexes can be 
	supplied to set specific joint angles, or the index
	can be omitted if the length of thetadot is equal to the number of degrees of freedom for that chain.
	
	.. Note:: Joint must be in theta_mj or theta_gc_mj mode for controller to track desired velocity commands.
	
	:param chain: Desired chain.
	:type chain: 'right_arm', 'left_arm', 'torso', 'head'
	:param thetadot: Joint velocity commands in radians/second.
	:type thetadot: array_like
	:param ind: Index of joints.
	:type ind: array_like, shape(len(thetadot)), optional
	
	:raises: 
	   M3Exception if chain is not supported or thetadot.shape is not (ndof) and
	   theta.shape is not ind.shape and ind is not None.
	  	
	:See Also: 
	   :meth:`M3Humanoid.set_thetadot_rad`	   
	   :meth:`M3Humanoid.get_thetadot_deg`
	   :meth:`M3Humanoid.set_mode_theta_mj`	   
	   :meth:`M3Humanoid.set_mode_theta_gc_mj`	  
	   
	:Examples:

	To set all joint angle commands with one list:
	
	>>> from m3.humanoid import M3Humanoid
	>>> bot = M3Humanoid('bob')
	>>> q_a = [0., .10, .20, .30, .40, .30, .20]
	>>> qdot_a = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1] #rad/s
	>>> bot.set_mode_theta_mj('right_arm')
	>>> bot.set_theta_rad('right_arm', q_a)	
	>>> bot.set_thetadot_rad('right_arm', qdot_a)	
	
	To change a subset of joint angle commands:
	
	>>> qdot_b = [.10, .10]
	>>> index = [3, 4]
	>>> bot.set_thetadot_rad('right_arm', qdot_b, index)	
	"""
	self.set_thetadot_deg(chain, deg2rad(nu.array(v,float)), ind)
	
    def set_thetadot_deg(self, chain, v, ind=None):
	"""
	Sets joint controller commands for selected chain to desired velocity (deg/s).  A list of joint indexes can be 
	supplied to set specific joint angles, or the index
	can be omitted if the length of thetadot is equal to the number of degrees of freedom for that chain.
	
	.. Note:: Joint must be in theta_mj or theta_gc_mj mode for controller to track desired velocity commands.
	
	:param chain: Desired chain.
	:type chain: 'right_arm', 'left_arm', 'torso', 'head'
	:param theta: Joint velocity commands in degrees/second.
	:type thetadot: array_like
	:param ind: Index of joints.
	:type ind: array_like, shape(len(thetadot)), optional
	
	:raises: 
	   M3Exception if chain is not supported or theta.shape is not (ndof) and
	   theta.shape is not ind.shape and ind is not None.
	  	
	:See Also: 
	   :meth:`M3Humanoid.set_thetadot_rad`	   
	   :meth:`M3Humanoid.get_thetadot_deg`
	   :meth:`M3Humanoid.set_mode_theta_mj`	   
	   :meth:`M3Humanoid.set_mode_theta_gc_mj`	  
	   
	:Examples:

	To set all joint angle commands with one list:
	
	>>> from m3.humanoid import M3Humanoid
	>>> bot = M3Humanoid('bob')
	>>> q_a = [0., 10., 20., 30., 40., 30., 20.]
	>>> qdot_a = [10., 10., 10., 10., 10., 10., 10.] #deg/s
	>>> bot.set_mode_theta_mj('right_arm')
	>>> bot.set_theta_deg('right_arm', q_a)	
	>>> bot.set_thetadot_deg('right_arm', qdot_a)	
	
	To change a subset of joint angle commands:
	
	>>> qdot_b = [10., 10.]
	>>> index = [3, 4]
	>>> bot.set_thetadot_deg('right_arm', qdot_b, index)	
	"""
	self.__assert_chain(chain)
	if ind is not None:
	    self.__assert_list_size(v, len(ind))
	else:
	    chain_attr = getattr(self, chain)
	    self.__assert_list_size(v, chain_attr.ndof)
	self.set_float_array(self.get_command(chain).qdot_desired,v,ind)
	
    def set_stiffness(self, chain, v,ind=None):
	"""
	Sets joint controller stiffness for selected chain to desired values.  A list of joint indexes can be 
	supplied to set specific joint stiffness values, or the index
	can be omitted if the length of v is equal to the number of degrees of freedom for that chain.  Stiffness
	must be between 0 and 1.  A stiffness of 0 effectively disables the joint angle reference command, while a
	stiffness of 1 would make the joint the least backdrivable.
	
	.. Note:: Joint must be in theta_gc mode for stiffness to effect joint output.
	
	:param chain: Desired chain.
	:type chain: 'right_arm', 'left_arm', 'torso', 'head'
	:param v: Joint stiffness values.
	:type v: array_like, 0 <= v[i] <=1
	:param ind: Index of joints.
	:type ind: array_like, shape(len(v)), optional
	
	:raises: 
	   M3Exception if chain is not supported or v.shape is not (ndof) and
	   v.shape is not ind.shape and ind is not None.
	  	
	:See Also: 
	   :meth:`M3Humanoid.set_theta_deg`
	   :meth:`M3Humanoid.set_mode_theta_gc`	  
	   
	:Examples:

	To set all joint stiffness values with one list:
	
	>>> from m3.humanoid import M3Humanoid
	>>> bot = M3Humanoid('bob')
	>>> v = [0.5]*7
	>>> bot.set_mode_theta_gc('right_arm')
	>>> bot.set_stiffness('right_arm', v)	
	
	To change a subset of joint stiffness values:
	
	>>> v_2 = [0.2]*2
	>>> index = [3, 4]
	>>> bot.set_stiffness('right_arm',v_2, index)	
	"""
	self.__assert_chain(chain)
	if ind is not None:
	    self.__assert_list_size(v, len(ind))
	else:
	    chain_attr = getattr(self, chain)
	    self.__assert_list_size(v, chain_attr.ndof)	
	self.set_float_array(self.get_command(chain).q_stiffness,v,ind)
	
    def set_slew_rate(self, chain, v,ind=None):
	"""
	Sets joint slew rate for selected chain to desired values in degrees/sec.  A list of joint indexes can be 
	supplied to set specific joint slew rate values, or the index
	can be omitted if the length of v is equal to the number of degrees of freedom for that chain.
	
	.. Note:: Joint must be in theta, theta_gc, theta_mj, or theta_gc_mj mode for slew rate to effect joint output.
	
	:param chain: Desired chain.
	:type chain: 'right_arm', 'left_arm', 'torso', 'head'
	:param v: slew rate values in degrees/sec.
	:type v: array_like
	:param ind: Index of joints.
	:type ind: array_like, shape(len(v)), optional
	
	:raises: 
	   M3Exception if chain is not supported or v.shape is not (ndof) and
	   v.shape is not ind.shape and ind is not None.
	  	
	:See Also: 
	   :meth:`M3Humanoid.set_slew_rate_proportion`	   
	"""
	self.__assert_chain(chain)
	if ind is not None:
	    self.__assert_list_size(v, len(ind))
	else:
	    chain_attr = getattr(self, chain)
	    self.__assert_list_size(v, chain_attr.ndof)	
	self.set_float_array(self.get_command(chain).q_slew_rate,v,ind)
	
    def set_slew_rate_proportion(self, chain, v,ind=None):
	"""
	Sets joint slew rate for selected chain to desired proportion of maximum slew rate defined in config files.
	A list of joint indexes can be 
	supplied to set specific slew rate values, or the index
	can be omitted if the length of v is equal to the number of degrees of freedom for that chain.  Proportion
	must be between 0 and 1.  A value of 0 effectively disables the joint angle reference command, while a
	stiffness of 1 would make the joint slew rate the maximum value.
	
	.. Note:: Joint must be in theta or theta_gc mode for slew rate to effect joint output.
	
	:param chain: Desired chain.
	:type chain: 'right_arm', 'left_arm', 'torso', 'head'
	:param v: Joint slew rate values.
	:type v: array_like, 0 <= v[i] <=1
	:param ind: Index of joints.
	:type ind: array_like, shape(len(v)), optional
	
	:raises: 
	   M3Exception if chain is not supported or v.shape is not (ndof) and
	   v.shape is not ind.shape and ind is not None.
	  	
	:See Also: 
	   :meth:`M3Humanoid.set_slew_rate`
	"""
	slew_rates = []
	self.__assert_chain(chain)
	chain_attr = getattr(self, chain)	
	
	if ind is not None:
	    self.__assert_list_size(v, len(ind))
	    for i in range(len(ind)):
	    	slew_rates.append(chain_attr.max_slew_rates[ind[i]]*v[i])
	else:
	    chain_attr = getattr(self, chain)
	    self.__assert_list_size(v, chain_attr.ndof)	
	    for i in range(len(v)):
		slew_rates.append(chain_attr.max_slew_rates[i]*v[i])	
	self.set_float_array(self.get_command(chain).q_slew_rate,slew_rates,ind)
	
    def set_mode(self, chain, v,ind=None):
	"""
	Sets joint controller mode for selected chain to desired option.  A list of joint indexes can be 
	supplied to set specific joint mode values, or the index
	can be omitted if the length of v is equal to the number of degrees of freedom for that chain.  Valid controller
	mode options are defined in ``m3.m3.joint_array_pb2`` and include JOINT_ARRAY_MODE_OFF, JOINT_ARRAY_MODE_PWM, JOINT_ARRAY_MODE_SPLINED_TRAJ,	
	JOINT_ARRAY_MODE_SPLINED_TRAJ_GC, JOINT_ARRAY_MODE_THETA, JOINT_ARRAY_MODE_THETA_GC, JOINT_ARRAY_MODE_THETA_GC_MJ, JOINT_ARRAY_MODE_THETA_MJ,
	JOINT_ARRAY_MODE_TORQUE, JOINT_ARRAY_MODE_TORQUE_GC.
	
	:param chain: Desired chain.
	:type chain: 'right_arm', 'left_arm', 'torso', 'head'
	:param v: Joint controller mode values.
	:type v: array_like, []
	:param ind: Index of joints.
	:type ind: array_like, shape(len(v)), optional
	
	:raises: 
	   M3Exception if chain is not supported or v.shape is not (ndof) and
	   v.shape is not ind.shape and ind is not None.
	  	
	:See Also: 	   
	   :meth:`M3Humanoid.set_mode_torque`
	   :meth:`M3Humanoid.set_mode_off`
	   :meth:`M3Humanoid.set_mode_torque_gc`
	   :meth:`M3Humanoid.set_mode_theta`
	   :meth:`M3Humanoid.set_mode_theta_mj`
	   :meth:`M3Humanoid.set_mode_theta_gc`
	   :meth:`M3Humanoid.set_mode_theta_gc_mj`
	   :meth:`M3Humanoid.set_mode_splined_traj`
	   :meth:`M3Humanoid.set_mode_splined_traj_gc`
	   
	:Examples:

	To set all joint controller mode values with one list:
	
	>>> from m3.humanoid import M3Humanoid
	>>> import m3.joint_array_pb2 as ja
	>>> bot = M3Humanoid('bob')
	>>> v = [ja.JOINT_ARRAY_MODE_THETA_GC]*7
	>>> bot.set_mode('right_arm',v)
	
	To change a subset of joint controller mode values:
	
	>>> v_2 = [ja.JOINT_ARRAY_MODE_THETA_GC]*2
	>>> index = [3, 4]
	>>> bot.set_mode('right_arm',v_2, index)	
	"""
	self.__assert_chain(chain)
	if ind is not None:
	    self.__assert_list_size(v, len(ind))
	else:
	    chain_attr = getattr(self, chain)
	    self.__assert_list_size(v, chain_attr.ndof)	
	self.set_int_array(self.get_command(chain).ctrl_mode,v,ind)
	
    def set_mode_off(self, chain, ind=None):
	"""
	Sets joint controller mode for selected chain to off.  A list of joint indexes can be 
	supplied to set specific joints, or the index
	can be omitted to turn all controllers in that chain off.
	
	:param chain: Desired chain.
	:type chain: 'right_arm', 'left_arm', 'torso', 'head'
	:param ind: Index of joints.
	:type ind: array_like, shape < ndof, optional
	
	:raises: 
	   M3Exception if chain is not supported or ind.shape is not < ndof
	  	
	:See Also: 	   
	   :meth:`M3Humanoid.set_mode`
	   :meth:`M3Humanoid.set_mode_torque`
	   :meth:`M3Humanoid.set_mode_torque_gc`
	   :meth:`M3Humanoid.set_mode_theta`
	   :meth:`M3Humanoid.set_mode_theta_mj`
	   :meth:`M3Humanoid.set_mode_theta_gc`
	   :meth:`M3Humanoid.set_mode_theta_gc_mj`
	   :meth:`M3Humanoid.set_mode_splined_traj`
	   :meth:`M3Humanoid.set_mode_splined_traj_gc`
	   
	:Examples:

	To turn all joint controllers off:
	
	>>> from m3.humanoid import M3Humanoid	
	>>> bot = M3Humanoid('bob')	
	>>> bot.set_mode_off('right_arm')	
	
	To turn a subset of joint controllers off:
		
	>>> index = [3, 4]
	>>> bot.set_mode_off('right_arm',index)	
	"""	
	if ind is None:
	    v = [mab.JOINT_ARRAY_MODE_OFF] * self.get_num_dof(chain)
	else:
	    v = [mab.JOINT_ARRAY_MODE_OFF] * len(ind)
	self.set_mode(chain, v, ind)
    
    def set_mode_pwm(self, chain, ind=None):
	"""
	Sets joint controller mode for selected chain to pwm control.  A list of joint indexes can be 
	supplied to set specific joints, or the index
	can be omitted to set all controllers in that chain to pwm control.
	
	:param chain: Desired chain.
	:type chain: 'right_arm', 'left_arm', 'torso', 'head'
	:param ind: Index of joints.
	:type ind: array_like, shape < ndof, optional
	
	:raises: 
	   M3Exception if chain is not supported or ind.shape is not < ndof
	  	
	:See Also: 	   
	   :meth:`M3Humanoid.set_mode`
	   :meth:`M3Humanoid.set_mode_off`
	   :meth:`M3Humanoid.set_mode_torque_gc`
	   :meth:`M3Humanoid.set_mode_theta`
	   :meth:`M3Humanoid.set_mode_theta_mj`
	   :meth:`M3Humanoid.set_mode_theta_gc`
	   :meth:`M3Humanoid.set_mode_theta_gc_mj`
	   :meth:`M3Humanoid.set_mode_splined_traj`
	   :meth:`M3Humanoid.set_mode_splined_traj_gc`
	   
	:Examples:

	To set all joint controllers to toque mode:
	
	>>> from m3.humanoid import M3Humanoid	
	>>> bot = M3Humanoid('bob')	
	>>> bot.set_mode_torque('right_arm')	
	
	To set a subset of joint controllers to torque mode:
		
	>>> index = [3, 4]
	>>> bot.set_mode_torque('right_arm',index)
	"""	
	if ind is None:
	    v = [mab.JOINT_ARRAY_MODE_PWM] * self.get_num_dof(chain)
	else:
	    v = [mab.JOINT_ARRAY_MODE_PWM] * len(ind)
	self.set_mode(chain, v, ind)
	
    def set_mode_torque(self, chain, ind=None):
	"""
	Sets joint controller mode for selected chain to torque control.  A list of joint indexes can be 
	supplied to set specific joints, or the index
	can be omitted to set all controllers in that chain to torque control.
	
	:param chain: Desired chain.
	:type chain: 'right_arm', 'left_arm', 'torso', 'head'
	:param ind: Index of joints.
	:type ind: array_like, shape < ndof, optional
	
	:raises: 
	   M3Exception if chain is not supported or ind.shape is not < ndof
	  	
	:See Also: 	   
	   :meth:`M3Humanoid.set_mode`
	   :meth:`M3Humanoid.set_mode_off`
	   :meth:`M3Humanoid.set_mode_torque_gc`
	   :meth:`M3Humanoid.set_mode_theta`
	   :meth:`M3Humanoid.set_mode_theta_mj`
	   :meth:`M3Humanoid.set_mode_theta_gc`
	   :meth:`M3Humanoid.set_mode_theta_gc_mj`
	   :meth:`M3Humanoid.set_mode_splined_traj`
	   :meth:`M3Humanoid.set_mode_splined_traj_gc`
	   
	:Examples:

	To set all joint controllers to toque mode:
	
	>>> from m3.humanoid import M3Humanoid	
	>>> bot = M3Humanoid('bob')	
	>>> bot.set_mode_torque('right_arm')	
	
	To set a subset of joint controllers to torque mode:
		
	>>> index = [3, 4]
	>>> bot.set_mode_torque('right_arm',index)
	"""	
	if ind is None:
	    v = [mab.JOINT_ARRAY_MODE_TORQUE] * self.get_num_dof(chain)
	else:
	    v = [mab.JOINT_ARRAY_MODE_TORQUE] * len(ind)
	self.set_mode(chain, v, ind)	
	
    def set_mode_torque_gc(self, chain, ind=None):
	"""
	Sets joint controller mode for selected chain to torque control with gravity compensation.  A 
	list of joint indexes can be supplied to set specific joints, or the index
	can be omitted to set all controllers in that chain to torque control with gravity compenstation.
	
	:param chain: Desired chain.
	:type chain: 'right_arm', 'left_arm', 'torso', 'head'
	:param ind: Index of joints.
	:type ind: array_like, shape < ndof, optional
	
	:raises: 
	   M3Exception if chain is not supported or ind.shape is not < ndof
	  	
	:See Also: 	   
	   :meth:`M3Humanoid.set_mode`
	   :meth:`M3Humanoid.set_mode_torque`
	   :meth:`M3Humanoid.set_mode_off`
	   :meth:`M3Humanoid.set_mode_theta`
	   :meth:`M3Humanoid.set_mode_theta_mj`
	   :meth:`M3Humanoid.set_mode_theta_gc`
	   :meth:`M3Humanoid.set_mode_theta_gc_mj`
	   :meth:`M3Humanoid.set_mode_splined_traj`
	   :meth:`M3Humanoid.set_mode_splined_traj_gc`
	   
	:Examples:

	To set all joint controllers to toque mode with gravity compenstation:
	
	>>> from m3.humanoid import M3Humanoid	
	>>> bot = M3Humanoid('bob')	
	>>> bot.set_mode_torque_gc('right_arm')	
	
	To set a subset of joint controllers to torque mode with gravity compenstation:
		
	>>> index = [3, 4]
	>>> bot.set_mode_torque_gc('right_arm',index)
	"""	
	if ind is None:
	    v = [mab.JOINT_ARRAY_MODE_TORQUE_GC] * self.get_num_dof(chain)
	else:
	    v = [mab.JOINT_ARRAY_MODE_TORQUE_GC] * len(ind)
	self.set_mode(chain, v, ind)
    
    def set_mode_torque_shm(self, chain, ind=None):
	"""
	Sets joint controller mode for selected chain to torque control using the shared memory interface.  A 
	list of joint indexes can be supplied to set specific joints, or the index
	can be omitted to set all controllers in that chain to torque_shm control.
	
	:param chain: Desired chain.
	:type chain: 'right_arm', 'left_arm', 'torso', 'head'
	:param ind: Index of joints.
	:type ind: array_like, shape < ndof, optional
	
	:raises: 
	   M3Exception if chain is not supported or ind.shape is not < ndof
	  	
	:See Also: 	   
	   :meth:`M3Humanoid.set_mode`
	   :meth:`M3Humanoid.set_mode_torque`
	   :meth:`M3Humanoid.set_mode_off`
	   :meth:`M3Humanoid.set_mode_theta`
	   :meth:`M3Humanoid.set_mode_theta_mj`
	   :meth:`M3Humanoid.set_mode_theta_gc`
	   :meth:`M3Humanoid.set_mode_theta_gc_mj`
	   :meth:`M3Humanoid.set_mode_splined_traj`
	   :meth:`M3Humanoid.set_mode_splined_traj_gc`
	   
	:Examples:

	To set all joint controllers to toque shared memory mode:
	
	>>> from m3.humanoid import M3Humanoid	
	>>> bot = M3Humanoid('bob')	
	>>> bot.set_mode_torque_shm('right_arm')	
	
	To set a subset of joint controllers to torque shared memory mode:
		
	>>> index = [3, 4]
	>>> bot.set_mode_torque_shm('right_arm',index)
	"""	
	if ind is None:
	    v = [mab.JOINT_ARRAY_MODE_TORQUE_SHM] * self.get_num_dof(chain)
	else:
	    v = [mab.JOINT_ARRAY_MODE_TORQUE_SHM] * len(ind)
	self.set_mode(chain, v, ind)
	
    def set_mode_theta(self, chain, ind=None):
	"""
	Sets joint controller mode for selected chain to joint angle control.  A 
	list of joint indexes can be supplied to set specific joints, or the index
	can be omitted to set all controllers in that chain to joint angle.
	
	:param chain: Desired chain.
	:type chain: 'right_arm', 'left_arm', 'torso', 'head'
	:param ind: Index of joints.
	:type ind: array_like, shape < ndof, optional
	
	:raises: 
	   M3Exception if chain is not supported or ind.shape is not < ndof
	  	
	:See Also: 	   
	   :meth:`M3Humanoid.set_mode`
	   :meth:`M3Humanoid.set_mode_torque`
	   :meth:`M3Humanoid.set_mode_off`
	   :meth:`M3Humanoid.set_mode_torque_gc`
	   :meth:`M3Humanoid.set_mode_theta_mj`
	   :meth:`M3Humanoid.set_mode_theta_gc`
	   :meth:`M3Humanoid.set_mode_theta_gc_mj`
	   :meth:`M3Humanoid.set_mode_splined_traj`
	   :meth:`M3Humanoid.set_mode_splined_traj_gc`
	   
	:Examples:

	To set all joint controllers to joint angle control:
	
	>>> from m3.humanoid import M3Humanoid	
	>>> bot = M3Humanoid('bob')	
	>>> bot.set_mode_theta('right_arm')	
	
	To set a subset of joint controllers to joint angle control:
		
	>>> index = [3, 4]
	>>> bot.set_mode_theta('right_arm',index)
	"""	
	if ind is None:
	    v = [mab.JOINT_ARRAY_MODE_THETA] * self.get_num_dof(chain)
	else:
	    v = [mab.JOINT_ARRAY_MODE_THETA] * len(ind)
	self.set_mode(chain, v, ind)	
	
    def set_mode_theta_gc(self, chain, ind=None):
	"""
	Sets joint controller mode for selected chain to joint angle control with gravity compensation.  A 
	list of joint indexes can be supplied to set specific joints, or the index
	can be omitted to set all controllers in that chain to joint angle with gravity compensation.
	
	:param chain: Desired chain.
	:type chain: 'right_arm', 'left_arm', 'torso', 'head'
	:param ind: Index of joints.
	:type ind: array_like, shape < ndof, optional
	
	:raises: 
	   M3Exception if chain is not supported or ind.shape is not < ndof
	  	
	:See Also: 	   
	   :meth:`M3Humanoid.set_mode`
	   :meth:`M3Humanoid.set_mode_torque`
	   :meth:`M3Humanoid.set_mode_off`
	   :meth:`M3Humanoid.set_mode_torque_gc`
	   :meth:`M3Humanoid.set_mode_theta`
	   :meth:`M3Humanoid.set_mode_theta_mj`
	   :meth:`M3Humanoid.set_mode_theta_gc_mj`
	   :meth:`M3Humanoid.set_mode_splined_traj`
	   :meth:`M3Humanoid.set_mode_splined_traj_gc`
	   
	:Examples:

	To set all joint controllers to joint angle control with gravity compensation:  
	
	>>> from m3.humanoid import M3Humanoid	
	>>> bot = M3Humanoid('bob')	
	>>> bot.set_mode_theta_gc('right_arm')	
	
	To set a subset of joint controllers to joint angle control with gravity compensation:
		
	>>> index = [3, 4]
	>>> bot.set_mode_theta_gc('right_arm',index)
	"""
	if ind is None:
	    v = [mab.JOINT_ARRAY_MODE_THETA_GC] * self.get_num_dof(chain)
	else:
	    v = [mab.JOINT_ARRAY_MODE_THETA_GC] * len(ind)
	self.set_mode(chain, v, ind)	
	
    def set_mode_theta_mj(self, chain, ind=None):
	"""
	Sets joint controller mode for selected chain to joint angle control with minimum jerk filtering.  A 
	list of joint indexes can be supplied to set specific joints, or the index
	can be omitted to set all controllers in that chain to joint angle with minimum jerk filtering.
	
	:param chain: Desired chain.
	:type chain: 'right_arm', 'left_arm', 'torso', 'head'
	:param ind: Index of joints.
	:type ind: array_like, shape < ndof, optional
	
	:raises: 
	   M3Exception if chain is not supported or ind.shape is not < ndof
	  	
	:See Also: 	   
	   :meth:`M3Humanoid.set_mode`
	   :meth:`M3Humanoid.set_mode_torque`
	   :meth:`M3Humanoid.set_mode_off`
	   :meth:`M3Humanoid.set_mode_torque_gc`
	   :meth:`M3Humanoid.set_mode_theta`
	   :meth:`M3Humanoid.set_mode_theta_gc`
	   :meth:`M3Humanoid.set_mode_theta_gc_mj`
	   :meth:`M3Humanoid.set_mode_splined_traj`
	   :meth:`M3Humanoid.set_mode_splined_traj_gc`
	   
	:Examples:

	To set all joint controllers to joint angle control with minimum jerk filtering:  
	
	>>> from m3.humanoid import M3Humanoid	
	>>> bot = M3Humanoid('bob')	
	>>> bot.set_mode_theta_mj('right_arm')	
	
	To set a subset of joint controllers to joint angle control with minimum jerk filtering:
		
	>>> index = [3, 4]
	>>> bot.set_mode_theta_mj('right_arm',index)
	"""
	if ind is None:
	    v = [mab.JOINT_ARRAY_MODE_THETA_MJ] * self.get_num_dof(chain)
	else:
	    v = [mab.JOINT_ARRAY_MODE_THETA_MJ] * len(ind)
	self.set_mode(chain, v, ind)
	
    def set_mode_theta_gc_mj(self, chain, ind=None):
	"""
	Sets joint controller mode for selected chain to joint angle control with gravity compensation and
	minimum jerk filtering.  A list of joint indexes can be supplied to set specific joints, or the index
	can be omitted to set all controllers in that chain to joint angle with minimum jerk filtering.
	
	:param chain: Desired chain.
	:type chain: 'right_arm', 'left_arm', 'torso', 'head'
	:param ind: Index of joints.
	:type ind: array_like, shape < ndof, optional
	
	:raises: 
	   M3Exception if chain is not supported or ind.shape is not < ndof
	  	
	:See Also: 	   
	   :meth:`M3Humanoid.set_mode`
	   :meth:`M3Humanoid.set_mode_torque`
	   :meth:`M3Humanoid.set_mode_off`
	   :meth:`M3Humanoid.set_mode_torque_gc`
	   :meth:`M3Humanoid.set_mode_theta`
	   :meth:`M3Humanoid.set_mode_theta_mj`
	   :meth:`M3Humanoid.set_mode_theta_gc`
	   :meth:`M3Humanoid.set_mode_splined_traj`
	   :meth:`M3Humanoid.set_mode_splined_traj_gc`
	   
	:Examples:

	To set all joint controllers to joint angle control with gravity compensation and minimum jerk filtering:  
	
	>>> from m3.humanoid import M3Humanoid	
	>>> bot = M3Humanoid('bob')	
	>>> bot.set_mode_theta_gc_mj('right_arm')	
	
	To set a subset of joint controllers to joint angle control with gravity compensation and minimum jerk filtering:
		
	>>> index = [3, 4]
	>>> bot.set_mode_theta_gc_mj('right_arm',index)
	"""
	if ind is None:
	    v = [mab.JOINT_ARRAY_MODE_THETA_GC_MJ] * self.get_num_dof(chain)
	else:
	    v = [mab.JOINT_ARRAY_MODE_THETA_GC_MJ] * len(ind)
	self.set_mode(chain, v, ind)
	
    def set_mode_splined_traj_gc(self, chain, ind=None):
	"""
	Sets joint controller mode for selected chain to splined trajectory control with gravity compensation.  A 
	list of joint indexes can be supplied to set specific joints, or the index
	can be omitted to set all controllers in that chain to splined trajectory control with gravity compensation.
	
	:param chain: Desired chain.
	:type chain: 'right_arm', 'left_arm', 'torso', 'head'
	:param ind: Index of joints.
	:type ind: array_like, shape < ndof, optional
	
	:raises: 
	   M3Exception if chain is not supported or ind.shape is not < ndof
	  	
	:See Also: 	   
	   :meth:`M3Humanoid.set_mode`
	   :meth:`M3Humanoid.set_mode_torque`
	   :meth:`M3Humanoid.set_mode_off`
	   :meth:`M3Humanoid.set_mode_torque_gc`
	   :meth:`M3Humanoid.set_mode_theta`
	   :meth:`M3Humanoid.set_mode_theta_mj`
	   :meth:`M3Humanoid.set_mode_theta_gc`
	   :meth:`M3Humanoid.set_mode_theta_gc_mj`
	   :meth:`M3Humanoid.set_mode_splined_traj`
	   
	:Examples:

	To set all joint controllers to splined trajectory control with gravity compensation:  
	
	>>> from m3.humanoid import M3Humanoid	
	>>> bot = M3Humanoid('bob')	
	>>> bot.set_mode_splined_traj_gc('right_arm')	
	
	To set a subset of joint controllers to splined trajectory control with gravity compensation:
		
	>>> index = [3, 4]
	>>> bot.set_mode_splined_traj_gc('right_arm',index)
	"""
	if ind is None:
	    v = [mab.JOINT_ARRAY_MODE_SPLINED_TRAJ_GC] * self.get_num_dof(chain)
	else:
	    v = [mab.JOINT_ARRAY_MODE_SPLINED_TRAJ_GC] * len(ind)
	self.set_mode(chain, v, ind)
	
    def set_mode_splined_traj(self, chain, ind=None):
	"""
	Sets joint controller mode for selected chain to splined trajectory control.  A 
	list of joint indexes can be supplied to set specific joints, or the index
	can be omitted to set all controllers in that chain to splined trajectory control.
	
	:param chain: Desired chain.
	:type chain: 'right_arm', 'left_arm', 'torso', 'head'
	:param ind: Index of joints.
	:type ind: array_like, shape < ndof, optional
	
	:raises: 
	   M3Exception if chain is not supported or ind.shape is not < ndof
	  	
	:See Also: 	   
	   :meth:`M3Humanoid.set_mode`
	   :meth:`M3Humanoid.set_mode_torque`
	   :meth:`M3Humanoid.set_mode_off`
	   :meth:`M3Humanoid.set_mode_torque_gc`
	   :meth:`M3Humanoid.set_mode_theta`
	   :meth:`M3Humanoid.set_mode_theta_mj`
	   :meth:`M3Humanoid.set_mode_theta_gc`
	   :meth:`M3Humanoid.set_mode_theta_gc_mj`
	   :meth:`M3Humanoid.set_mode_splined_traj`
	   :meth:`M3Humanoid.set_mode_splined_traj_gc`
	   
	:Examples:

	To set all joint controllers to splined trajectory control:  
	
	>>> from m3.humanoid import M3Humanoid	
	>>> bot = M3Humanoid('bob')	
	>>> bot.set_mode_splined_traj('right_arm')	
	
	To set a subset of joint controllers to splined trajectory control:
		
	>>> index = [3, 4]
	>>> bot.set_mode_splined_traj('right_arm',index)
	"""
	if ind is None:
	    v = [mab.JOINT_ARRAY_MODE_SPLINED_TRAJ] * self.get_num_dof(chain)
	else:
	    v = [mab.JOINT_ARRAY_MODE_SPLINED_TRAJ] * len(ind)
	self.set_mode(chain, v, ind)	
	
    def set_payload_com(self, chain, com):
	"""
	Sets the estimated payload center-of-mass in the wrist frame for the realtime inverse dynamic model.
	
	:param chain: Desired chain.
	:type chain: 'right_arm', 'left_arm', 'torso', 'head'
	:param com: XYZ in meters of center-of-mass for payload in wrist frame.
	:type com: array_like, shape (3)
	
	:raises: 
	   M3Exception if chain is not supported or com.shape is not (3)
	  	
	:See Also: 	   
	   :meth:`M3Humanoid.set_payload_mass`	   
	   
	:Examples:

	To set center-of-mass for payload in right wrist frame:
	
	>>> from m3.humanoid import M3Humanoid	
	>>> bot = M3Humanoid('bob')	
	>>> com = [0.1,0.2,0.3]
	>>> bot.set_payload_com('right_arm', com)
	"""
	self.__assert_chain(chain)
	self.__assert_list_size(com, 3)
	self.get_param(chain).payload_com[0]=float(com[0])
	self.get_param(chain).payload_com[1]=float(com[1])
	self.get_param(chain).payload_com[2]=float(com[2])
	
    def set_payload_inertia(self, chain, inertia):
	"""
	Sets the estimated payload inertia in the wrist frame for the realtime inverse dynamic model.
	
	:param chain: Desired chain.
	:type chain: 'right_arm', 'left_arm', 'torso', 'head'
	:param inertia: [Ixx,Ixy,Ixz,Iyy,Iyz,Izz] for payload in wrist frame.
	:type inertia: array_like, shape (6)
	
	:raises: 
	   M3Exception if chain is not supported or inertia.shape is not (6)
	  	
	:See Also: 	   
	   :meth:`M3Humanoid.set_payload_com`
	   :meth:`M3Humanoid.set_payload_mass`	   
	"""
	self.__assert_chain(chain)
	self.__assert_list_size(inertia, 6)
	for i in range(6):
	    self.get_param(chain).payload_inertia[i]=float(inertia[i])	

    def set_payload_mass(self, chain, m): #Kg
	"""
	Sets the estimated payload mass for the realtime inverse dynamic model.
	
	:param chain: Desired chain.
	:type chain: 'right_arm', 'left_arm', 'torso', 'head'
	:param m: mass in Kg for payload.
	:type m: float or int
	
	:raises:
	   M3Exception if chain is not supported or mass it not type float or int
	  	
	:See Also: 	   
	   :meth:`M3Humanoid.set_payload_mass`
	   
	:Examples:

	To set mass for payload in right arm:
	
	>>> from m3.humanoid import M3Humanoid	
	>>> bot = M3Humanoid('bob')	
	>>> mass = 0.1
	>>> bot.set_payload_mass('right_arm', mass)
	"""
	self.__assert_chain(chain)
	self.get_param(chain).payload_mass=m	    
    
    def get_tool_twist(self, chain):
	"""
	Gets twist of tool frame origin for selected chain with reference to the world frame.  Returns twist represented as
	a 6 element array where [:3] is the velocity at the origin of the tool frame in m/s and
	[3:] is the angular velocity of the tool frame origin in deg/sec.
	
	:param chain: Desired chain.
	:type chain: 'right_arm', 'left_arm', 'torso', 'head'
	:returns: twist of tool frame for chain with respect to world frame.
	:rtype: array, shape (6)
	
	:raises: M3Exception if chain is not supported
	
	:See Also: 
	   :meth:`M3Humanoid.get_tool_velocity`	   
	   :meth:`M3Humanoid.get_tool_angular_velocity_deg`
	   :meth:`M3Humanoid.get_tool_angular_velocity_rad`
	   :meth:`M3Humanoid.get_tool_position`
	   :meth:`M3Humanoid.get_tool_roll_pitch_yaw_deg`
	   :meth:`M3Humanoid.get_tool_roll_pitch_yaw_rad`
	"""
	self.__assert_chain(chain)
	return nu.array(self.get_status(chain).end_twist,float)
    
    def get_tool_velocity(self, chain):
	"""
	Gets velocity of tool frame origin for selected chain with reference to the world frame.  
	
	:param chain: Desired chain.
	:type chain: 'right_arm', 'left_arm', 'torso', 'head'
	:returns: velocity of tool frame for chain with respect to world frame in m/s.
	:rtype: array, shape (3)
	
	:raises: M3Exception if chain is not supported
	
	:See Also: 
	   :meth:`M3Humanoid.get_tool_twist`	   
	   :meth:`M3Humanoid.get_tool_angular_velocity_deg`
	   :meth:`M3Humanoid.get_tool_angular_velocity_rad`
	   :meth:`M3Humanoid.get_tool_position`
	   :meth:`M3Humanoid.get_tool_roll_pitch_yaw_deg`
	   :meth:`M3Humanoid.get_tool_roll_pitch_yaw_rad`
	"""	
	return self.get_tool_twist(chain)[:3]
    
    def get_tool_angular_velocity_deg(self, chain):
	"""
	Gets angular velocity of tool frame origin for selected chain with reference to the world frame in deg/sec.
	
	:param chain: Desired chain.
	:type chain: 'right_arm', 'left_arm', 'torso', 'head'
	:returns: angular velocity of tool frame for chain with respect to world frame in deg/sec.
	:rtype: array, shape (3)
	
	:raises: M3Exception if chain is not supported
	
	:See Also: 
	   :meth:`M3Humanoid.get_tool_velocity`	   
	   :meth:`M3Humanoid.get_tool_twist`
	   :meth:`M3Humanoid.get_tool_position`
	   :meth:`M3Humanoid.get_tool_roll_pitch_yaw_deg`
	   :meth:`M3Humanoid.get_tool_roll_pitch_yaw_rad`
	"""
	return self.get_tool_twist(chain)[3:]
    
    def get_tool_angular_velocity_rad(self, chain):
	"""
	Gets angular velocity of tool frame origin for selected chain with reference to the world frame in deg/sec.
	
	:param chain: Desired chain.
	:type chain: 'right_arm', 'left_arm', 'torso', 'head'
	:returns: angular velocity of tool frame for chain with respect to world frame in deg/sec.
	:rtype: array, shape (3)
	
	:raises: M3Exception if chain is not supported
	
	:See Also: 
	   :meth:`M3Humanoid.get_tool_velocity`	   
	   :meth:`M3Humanoid.get_tool_twist`
	   :meth:`M3Humanoid.get_tool_position`
	   :meth:`M3Humanoid.get_tool_roll_pitch_yaw_deg`
	   :meth:`M3Humanoid.get_tool_roll_pitch_yaw_rad`
	"""
	return deg2rad(self.get_tool_angular_velocity_deg(chain))
    
    def get_tool_position(self, chain):
	"""
	Gets XYZ position in meters of tool frame origin for selected chain with reference to the world frame.  
	
	:param chain: Desired chain.
	:type chain: 'right_arm', 'left_arm', 'torso', 'head'
	:returns: position of tool frame for chain with respect to world frame in meters.
	:rtype: array, shape (3)
	
	:raises: M3Exception if chain is not supported
	
	:See Also: 
	   :meth:`M3Humanoid.get_tool_velocity`	   
	   :meth:`M3Humanoid.get_tool_twist`
	   :meth:`M3Humanoid.get_tool_angular_velocity_deg`
	   :meth:`M3Humanoid.get_tool_angular_velocity_rad`
	   :meth:`M3Humanoid.get_tool_roll_pitch_yaw_deg`
	   :meth:`M3Humanoid.get_tool_roll_pitch_yaw_rad`
	"""	
	T = self.__get_tool_2_world_kdl(chain)
	p = self.kdl_to_list_vector(T.p)
	return nu.array(p,float)
    
    def get_tool_position_sim(self, chain):
	'''
	:See Also: 
	   :meth:`M3Humanoid.get_tool_position`
	   :meth:`M3Humanoid.get_tool_roll_pitch_yaw_deg_sim`
	   :meth:`M3Humanoid.get_tool_2_world_transform_sim`	   
	'''
	T = self.__get_tool_2_world_kdl_sim(chain)
	p = self.kdl_to_list_vector(T.p)
	return nu.array(p,float)
    
    def get_tool_roll_pitch_yaw_rad_sim(self, chain):
	'''
	:See Also: 
	   :meth:`M3Humanoid.get_tool_position_sim`
	   :meth:`M3Humanoid.get_tool_roll_pitch_yaw_rad`
	   :meth:`M3Humanoid.get_tool_roll_pitch_yaw_deg_sim`
	   :meth:`M3Humanoid.get_tool_2_world_transform_sim`	   
	'''
	T = self.__get_tool_2_world_kdl_sim(chain)	
	return nu.array(T.M.GetRPY(),float)
    
    def get_tool_roll_pitch_yaw_deg_sim(self, chain):
	'''
	:See Also: 
	   :meth:`M3Humanoid.get_tool_position_sim`
	   :meth:`M3Humanoid.get_tool_roll_pitch_yaw_deg`
	   :meth:`M3Humanoid.get_tool_roll_pitch_yaw_rad_sim`
	   :meth:`M3Humanoid.get_tool_2_world_transform_sim`	   
	'''
	return rad2deg(self.get_tool_roll_pitch_yaw_rad_sim(chain))
    
    def get_tool_roll_pitch_yaw_rad(self, chain):
	"""
	Gets roll, pitch, and yaw of tool frame in radians for selected chain with reference to the world frame.
	
	.. Note:: roll, pitch, and yaw angles in this context are considered X-Y-Z fixed angles around a fixed
	   reference frame (the world frame).  Starting with the tool frame coincident with the world frame,
	   the RPY values returned describe the tool frame orientation as follows:
	   
	   * First rotate the tool frame by roll angle around X-axis of world frame.
	   * Next rotate the tool frame by pitch angle around Y-axis of world frame.	   
	   * Finally rotate the tool frame by yaw angle around Z-axis of world frame.	   
	
	:param chain: Desired chain.
	:type chain: 'right_arm', 'left_arm', 'torso', 'head'
	:returns: roll, pitch, and yaw of tool frame in radians for chain with respect to world frame.
	:rtype: array, shape (3)
	
	:raises: M3Exception if chain is not supported
	
	:See Also: 
	   :meth:`M3Humanoid.get_tool_velocity`	   
	   :meth:`M3Humanoid.get_tool_twist`
	   :meth:`M3Humanoid.get_tool_angular_velocity_deg`
	   :meth:`M3Humanoid.get_tool_angular_velocity_rad`
	   :meth:`M3Humanoid.get_tool_roll_pitch_yaw_deg`
	   :meth:`M3Humanoid.get_tool_position`
	"""
	T = self.__get_tool_2_world_kdl(chain)	
	return nu.array(T.M.GetRPY(),float)
    
	
    
    def get_tool_roll_pitch_yaw_deg(self, chain):
	"""
	Gets roll, pitch, and yaw of tool frame in degrees for selected chain with reference to the world frame.
	
	.. Note:: roll, pitch, and yaw angles in this context are considered X-Y-Z fixed angles around a fixed
	   reference frame (the world frame).  Starting with the tool frame coincident with the world frame,
	   the RPY values returned describe the tool frame orientation as follows:
	   
	   * First rotate the tool frame by roll angle around X-axis of world frame.
	   * Next rotate the tool frame by pitch angle around Y-axis of world frame.	   
	   * Finally rotate the tool frame by yaw angle around Z-axis of world frame.	   
	
	:param chain: Desired chain.
	:type chain: 'right_arm', 'left_arm', 'torso', 'head'
	:returns: roll, pitch, and yaw of tool frame in degrees for chain with respect to world frame.
	:rtype: array, shape (3)
	
	:raises: M3Exception if chain is not supported
	
	:See Also: 
	   :meth:`M3Humanoid.get_tool_velocity`	   
	   :meth:`M3Humanoid.get_tool_twist`
	   :meth:`M3Humanoid.get_tool_angular_velocity_deg`
	   :meth:`M3Humanoid.get_tool_angular_velocity_rad`
	   :meth:`M3Humanoid.get_tool_roll_pitch_yaw_rad`
	   :meth:`M3Humanoid.get_tool_position`
	"""
	return rad2deg(self.get_tool_roll_pitch_yaw_rad(chain))
	
    
    def get_torque_gravity(self, chain):
	"""
	Gets torque values of gravity compensation controller in mN*m for a chain.
	
	:param chain: Desired chain.
	:type chain: 'right_arm', 'left_arm', 'torso', 'head'
	:returns: Torque values in mN*m for selected chain of gravity compensation controller.
	:rtype:	array, shape (ndof)
	
	:raises: M3Exception if chain is not supported.
	"""
	return nu.array(self.get_status(chain).G,float)
        
     
    #  expecting 'right' or 'left'
    def __get_eye_rotation(self, eye):	
	self.__assert_eye(eye)
	g = nu.zeros([9])	
	g[:]=getattr(self.status, eye + '_eye_rot')[:]
	g.resize([3,3])
	return g    
    
    def __assert_eye(self, eye):
	if eye != 'right' and eye != 'left':
	    raise m3t.M3Exception('eye: ' + eye + 'not supported. Expecting ''left'' or ''right''.')
		
    def get_eye_roll_pitch_yaw_rad(self, eye):
	"""
	Gets roll, pitch, and yaw of camera frame in radians for selected eye with reference to the world frame.
	
	.. Note:: roll, pitch, and yaw angles in this context are considered X-Y-Z fixed angles around a fixed
	   reference frame (the world frame).  Starting with the tool frame coincident with the world frame,
	   the RPY values returned describe the tool frame orientation as follows:
	   
	   * First rotate the tool frame by roll angle around X-axis of world frame.
	   * Next rotate the tool frame by pitch angle around Y-axis of world frame.	   
	   * Finally rotate the tool frame by yaw angle around Z-axis of world frame.	   
	
	:param eye: Desired eye.
	:type eye: 'right', 'left'
	:returns: roll, pitch, and yaw of camera frame in radians for chain with respect to world frame.
	:rtype: array, shape (3)
	
	:raises: M3Exception if eye is not supported
	
	:See Also: 
	   :meth:`M3Humanoid.get_eye_roll_pitch_yaw_deg`
	   :meth:`M3Humanoid.get_eye_position`	   
	   :meth:`M3Humanoid.get_eye_2_world_transform`	   
	"""
	return nu.array(self.kdl_to_list_vector(self.__get_eye_rotation_kdl(eye).GetRPY()),float)
    
    def __get_eye_rotation_kdl(self, eye):		
	return self.numpy_to_kdl_rotation(self.__get_eye_rotation(eye))
    
    
    
    def get_eye_roll_pitch_yaw_deg(self, eye):
	"""
	Gets roll, pitch, and yaw of camera frame in degrees for selected eye with reference to the world frame.
	
	.. Note:: roll, pitch, and yaw angles in this context are considered X-Y-Z fixed angles around a fixed
	   reference frame (the world frame).  Starting with the tool frame coincident with the world frame,
	   the RPY values returned describe the tool frame orientation as follows:
	   
	   * First rotate the tool frame by roll angle around X-axis of world frame.
	   * Next rotate the tool frame by pitch angle around Y-axis of world frame.	   
	   * Finally rotate the tool frame by yaw angle around Z-axis of world frame.	   
	
	:param eye: Desired eye.
	:type eye: 'right', 'left'
	:returns: roll, pitch, and yaw of camera frame in degrees for chain with respect to world frame.
	:rtype: array, shape (3)
	
	:raises: M3Exception if eye is not supported
	
	:See Also: 
	   :meth:`M3Humanoid.get_eye_roll_pitch_yaw_rad`
	   :meth:`M3Humanoid.get_eye_position`
	   :meth:`M3Humanoid.get_eye_2_world_transform`	   
	"""
	return rad2deg(self.get_eye_roll_pitch_yaw_rad(eye))
				   
    def get_eye_position(self, eye):
	"""
	Gets XYZ position in meters of camera frame origin for selected eye with reference to the world frame.  
	
	:param eye: Desired chain.
	:type eye: 'right', 'left'
	:returns: position of camera frame for eye with respect to world frame in meters.
	:rtype: array, shape (3)
	
	:raises: M3Exception if chain is not supported
	
	:See Also: 
	   :meth:`M3Humanoid.get_eye_roll_pitch_yaw_rad`
	   :meth:`M3Humanoid.get_eye_roll_pitch_yaw_deg`
	   :meth:`M3Humanoid.get_eye_2_world_transform`
	"""
	self.__assert_eye(eye)
	g = nu.zeros([3])	
	g[:]=getattr(self.status, eye + '_eye_pos')[:]	
	return g
    
    def get_image_2_world_transform(self,eye):
	"""
	Describes transform from pixel units to world coordinates assuming a known depth
	along the ray through the camera focal point.  Units are in meters.
	
	:param eye: Desired eye to use for camera frame.
	:type eye: 'right', 'left'
	:returns: Description of image frame with respect to world frame.
	:rtype:	numpy.array, shape (4,4)
	      
	:raises: M3Exception if chain is not supported.
	
	:See Also: 
	   :meth:`M3Humanoid.get_world_2_image_transform`
	   :meth:`M3Humanoid.get_eye_2_world_transform`
	   :meth:`M3Humanoid.get_world_2_eye_transform`
	     
	:Examples:
	
	Get the tranform describing the image plane in reference to world frame.
	
	>>> from m3.humanoid import M3Humanoid
	>>> bot = M3Humanoid('bob')	
	>>> T = bot.get_image_2_world_transform('right')	
	>>> T	
	array([[ 1.,  0.,  0.,  1.],
	       [ 0.,  1.,  0.,  2.],
	       [ 0.,  0.,  1.,  3.],
	       [ 0.,  0.,  0.,  1.]])
	       
	Convert pixel (u,v) to world at depth r from focal point.
	x_i = Numeric.array([u,v,r,1.0])
	x_w = nu.matrixmultiply(T,x_i)[0:3]
	"""
	pass
    
    def get_world_2_image_transform(self):
	pass
    
    def eye_2_world(self,eye,p):
	"""
	Maps a point in the eye frame to the world frame.  Units assumed to be in meters.
	
	:param eye: Desired eye to use.
	:type eye: 'right', 'left'
	:param p: Description of vector with respect to eye frame.
	:type p: array_like, shape (3)
	
	:rtype: numpy.array, shape (3)
	:returns: Description of vector with respect to world frame.
		
	:raises: M3Exception if p_t.shape is not (3) or eye is not supported
	
	:See Also: 
	   :meth:`M3Humanoid.get_world_2_eye_transform`
	   :meth:`M3Humanoid.get_eye_2_world_transform`
	   :meth:`M3Humanoid.world_2_eye`
	
	:Examples:
	
	Map the point [1,2,3] from right eye frame to world frame.
	
	>>> from m3.humanoid import M3Humanoid
	>>> bot = M3Humanoid('bob')
	>>> import m3.rt_proxy as m3p
	>>> proxy = m3p.M3RtProxy()
	>>> proxy.start()
	>>> proxy.subscribe_status(bot)
	>>> proxy.step()
	>>> p_eye = [1.,2.,3.]
	>>> p_world = bot.eye_2_world('right', p_eye)	    
	"""
	T=self.__get_eye_2_world_transform_kdl(eye)
	p_kdl=self.list_to_kdl_vector(p)
	return  nu.array(self.kdl_to_list_vector(T*p_kdl),float)
	
    def world_2_eye(self,eye,p):
	"""
	Maps a point in the world frame to the eye frame.  Units assumed to be in meters.
	
	:param eye: Desired eye to use.
	:type eye: 'right', 'left'
	:param p: Description of vector with respect to world frame.
	:type p: array_like, shape (3)
	
	:rtype: numpy.array, shape (3)
	:returns: Description of vector with respect to eye frame.
		
	:raises: M3Exception if p_t.shape is not (3) or eye is not supported
	
	:See Also: 
	   :meth:`M3Humanoid.get_world_2_eye_transform`
	   :meth:`M3Humanoid.get_eye_2_world_transform`
	   :meth:`M3Humanoid.eye_2_world`
	
	:Examples:
	
	Map the point [1,2,3] from world frame to right eye frame.
	
	>>> from m3.humanoid import M3Humanoid
	>>> bot = M3Humanoid('bob')
	>>> import m3.rt_proxy as m3p
	>>> proxy = m3p.M3RtProxy()
	>>> proxy.start()
	>>> proxy.subscribe_status(bot)
	>>> proxy.step()
	>>> p_world = [1.,2.,3.]
	>>> p_eye = bot.world_2_eye('right', p_world)	    
	"""
	T=self.__get_world_2_eye_transform_kdl(eye)
	p_kdl=self.list_to_kdl_vector(p)
	return  nu.array(self.kdl_to_list_vector(T*p_kdl),float)

    def get_eye_2_world_transform(self, eye):
	"""
	Describes the coordinate system attached at the camera relative to the world frame.  This same
	transform can also be used to map descriptions in the camera frame to the world frame.  The position and
	orientation of the camera are described by this transform.  Translation component is in meters.
	
	:param eye: Desired eye to use for camera frame.
	:type eye: 'right', 'left'
	:returns: Description of camera frame with respect to world frame.
	:rtype:	numpy.array, shape (4,4)
	      
	:raises: M3Exception if chain is not supported.
	
	:See Also: 
	   :meth:`M3Humanoid.get_eye_roll_pitch_yaw_rad`
	   :meth:`M3Humanoid.get_eye_roll_pitch_yaw_deg`
	   :meth:`M3Humanoid.get_eye_position`
	     
	:Examples:
	
	Get the tranform describing the right camera frame in reference to world frame.
	
	>>> from m3.humanoid import M3Humanoid
	>>> bot = M3Humanoid('bob')	
	>>> T = bot.get_eye_2_world_transform('right')	
	>>> T	
	array([[ 1.,  0.,  0.,  1.],
	       [ 0.,  1.,  0.,  2.],
	       [ 0.,  0.,  1.,  3.],
	       [ 0.,  0.,  0.,  1.]])
	       
	Extract position and rotation components.
	
	>>> R = bot.get_rot_from_transform(T)
	>>> R
	array([[ 1.,  0.,  0.],
	       [ 0.,  1.,  0.],
	       [ 0.,  0.,  1.]])
	>>> p = bot.get_pos_from_transform(T)
	>>> p
	array([ 1.,  2.,  3.])
	"""	
	return self.rot_and_pos_2_transform(self.__get_eye_rotation(eye), self.get_eye_position(eye))
    
    def __get_eye_2_world_transform_kdl(self, eye):
	R=self.__get_eye_rotation_kdl(eye)
	p=self.get_eye_position(eye)
	T = Frame(R, self.list_to_kdl_vector(p))
	return T
    
    def __get_world_2_eye_transform_kdl(self,eye):
	wTe=self.__get_eye_2_world_transform_kdl(eye)
	eTw=wTe.Inverse()
	return eTw
    
    def set_int_array(self,attr,val,ind=None):
        if ind is None:
            ind=range(len(attr))
	    
	j = 0
	for i in ind:
	    attr[i]=int(val[j])
	    j += 1

    def set_float_array(self,attr,val,ind=None):
        if ind is None:
            ind=range(len(attr))
        j = 0
        for i in ind:
            attr[i]=float(val[j])
            j += 1
    
    def add_splined_traj_via_rad(self,chain, theta_des,thetadot_avg):
	"""Add a desired via point to the queue. 
	-- thetadot_avg is the desired average velocity of the joint with the farthest to travel
	Requires that a NDOF are specified even if not all are in via control mode"""
	self.add_splined_traj_via_deg(chain, rad2deg(nu.array(theta_des),float), rad2deg(nu.array(thetadot_avg),float))	
    
    def add_splined_traj_via_deg(self, chain, theta_des,thetadot_avg):
	chain_attr = getattr(self, chain)
	chain_attr.vias.append([theta_des,thetadot_avg])
    
    def is_splined_traj_complete(self, chain):
	"""Have all the joint vias finished executing on the server"""
	chain_attr = getattr(self, chain)
	return len(chain_attr.vias)==0 and chain_attr.via_idx==self.get_status(chain).completed_spline_idx
	   
    def load_command(self):
	for chain in self.get_available_chains():
	    self.get_command(chain).ClearField('vias')
	    chain_attr = getattr(self, chain)
	    nadd=min(10,len(chain_attr.vias)) #only add 10 per cycle to keep packet size down
	    for n in range(nadd):
		chain_attr.via_idx=chain_attr.via_idx+1
		theta_des=chain_attr.vias[n][0]
		thetadot_avg=chain_attr.vias[n][1]
		self.get_command(chain).vias.add()
		for i in range(chain_attr.ndof):
		    self.get_command(chain).vias[-1].q_desired.append(float(theta_des[i]))
		    self.get_command(chain).vias[-1].qdot_avg.append(float(thetadot_avg[i]))
		self.get_command(chain).vias[-1].idx=chain_attr.via_idx
	    chain_attr.vias=chain_attr.vias[nadd:]