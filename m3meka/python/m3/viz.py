#M3 -- Meka Robotics Robot Components
#Copyright (C) 2010 Meka Robotics
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
import os
import roslib; roslib.load_manifest('m3_defs_ros')
import rospy
import m3.component_factory as m3f
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import subprocess
        
class M3Viz:
    # ToDo: pass in r_hand_ua component to publish joints
    def __init__ (self,proxy,bot,r_hand_ua_num=None,stride_ms=100):
	self.p = subprocess.Popen(['roslaunch', 'm3_defs_ros', 'm3_launch.launch'])
        rospy.init_node("joint_state_publisher")
        pub = rospy.Publisher("/joint_states", JointState)
        time.sleep(4.0)                    
	self.bot=bot
	self.proxy=proxy	
	self.pub = pub
        self.sleep_time = stride_ms/1000.0
	self.joints = []
        self.positions = []
	self.chain_names = self.bot.get_available_chains()        
        self.use_sim = False
        self.r_hand_ua_present = False
	
	self.ndof_finger = 3
	
        self.flex_factor_index = [0.3] * self.ndof_finger 
	self.flex_factor_ring = [0.3] * self.ndof_finger
	self.flex_factor_pinky = [0.3] * self.ndof_finger
	self.flex_factor_thumb = [0.3] * 2
        
        for chain in self.chain_names:
                self.positions += list(self.bot.get_theta_rad(chain))
                self.joints += self.bot.get_joint_names(chain)
        self.ndof_hand_ua = 12
	
	hand_names=self.proxy.get_available_components('m3hand')
            
	self.hands = []
	self.hand_nums = []
	    
	for i in range(len(hand_names)):	    
	    self.hands.append(m3f.create_component(hand_names[i]))
	    self.proxy.subscribe_status(self.hands[i])
	    #self.proxy.publish_command(self.hands[i])
	    if hand_names[i][-2].isdigit():
		self.hand_nums.append(hand_names[i][-2:])
	    else:
		self.hand_nums.append(hand_names[i][-1])

	#r_hand_ua_num = 14
	
	self.ndof_hand_ua = 12
        
	for j in range(len(self.hands)):
	    for i in range(self.ndof_hand_ua):
		self.positions.append(0.0)
		self.joints.append('m3joint_ua_mh'+str(self.hand_nums[j])+'_j'+str(i))
        
	# Thumb: J0,J1,J2
	# Index: J3, J4, J5
	# Ring: J6,J7,J8
	# Pinkie: J9, J10, J11
	
        '''if not r_hand_ua_num is None:
            for i in range(self.ndof_hand_ua):
                self.positions.append(0.0)
                self.joints.append('m3joint_ua_mh'+str(r_hand_ua_num)+'_j'+str(i))
            self.r_hand_ua_present = True'''

    def step(self):
        self.positions = []
	for chain in self.chain_names:
	    if not self.use_sim:
		self.positions += list(self.bot.get_theta_rad(chain))
                #print self.positions
	    else:
		self.positions += list(self.bot.get_theta_sim_rad(chain))
        '''if self.r_hand_ua_present:
            self.positions += list([0.0]*self.ndof_hand_ua)'''
	    
	for i in range(len(self.hands)):		
	    th =self.hands[i].get_theta_rad()
	    
	    #Thumb
	    self.positions.append(-th[0]+1.57) #0
	    self.positions.append(th[1] * self.flex_factor_thumb[0])
	    self.positions.append(th[1] * self.flex_factor_thumb[1])
	    #Index
	    self.positions.append(th[2] * self.flex_factor_index[0])
	    self.positions.append(th[2] * self.flex_factor_index[1])
	    self.positions.append(th[2] * self.flex_factor_index[2])
	    #Ring
	    self.positions.append(th[3] * self.flex_factor_ring[0])
	    self.positions.append(th[3] * self.flex_factor_ring[1])
	    self.positions.append(th[3] * self.flex_factor_ring[2])
	    #Pinkie
	    self.positions.append(th[4] * self.flex_factor_pinky[0])
	    self.positions.append(th[4] * self.flex_factor_pinky[1])
	    self.positions.append(th[4] * self.flex_factor_pinky[2])
            
	if self.pub != None:
	    if not rospy.is_shutdown():
		    header = Header(0,rospy.Time.now(),'0')
		    self.pub.publish(JointState(header, self.joints, self.positions, [0]*len(self.positions), [0]*len(self.positions)))	    
		    return True
	return False

    def turn_sim_on(self):
        self.use_sim = True
    
    def turn_sim_off(self):
        self.use_sim = False
            
    def stop(self):        
        os.system("pkill -P " + str(self.p.pid))
	os.kill(self.p.pid,9)