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
import m3.gui as m3g
import m3.hand as m3h
import m3.toolbox as m3t
import m3.unit_conversion as m3u
import m3.component_factory as m3f
import Numeric as nu
import m3.rt_proxy as m3p
import yaml
import os
import roslib; roslib.load_manifest('m3_defs_ros')
import rospy
from sensor_msgs.msg import JointState
from roslib.msg import Header
import subprocess


class M3Proc:
    def __init__(self):
        self.proxy = m3p.M3RtProxy()
        self.gui = m3g.M3Gui()

    def stop(self):
        self.proxy.stop()
    def start(self):
        self.proxy.start()
	
	self.ndof_finger = 3
	
        self.flex_factor_index = [0.3] * self.ndof_finger 
	self.flex_factor_ring = [0.3] * self.ndof_finger
	self.flex_factor_pinky = [0.3] * self.ndof_finger
	self.flex_factor_thumb = [0.3] * 2
	

	self.p = subprocess.Popen(['roslaunch', 'm3_defs_ros', 'm3_launch.launch'])
	rospy.init_node("joint_state_publisher")
        self.pub = rospy.Publisher("/joint_states", JointState)
        time.sleep(4.0)                    

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
        
	self.positions = []
	self.joints = []
	
	for j in range(len(self.hands)):
	    for i in range(self.ndof_hand_ua):
		self.positions.append(0.0)
		self.joints.append('m3joint_ua_mh'+str(self.hand_nums[j])+'_j'+str(i))
		
	# Thumb: J0,J1,J2
	# Index: J3, J4, J5
	# Ring: J6,J7,J8
	# Pinkie: J9, J10, J11
	
	print 'Starting hand viz.'
	
	while(True):
	    self.positions = []	    
	    self.proxy.step()	    
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
	    
	    if self.pub is not None and not rospy.is_shutdown():
		header = Header(0,rospy.Time.now(),'0')
		self.pub.publish(JointState(header, self.joints, self.positions, [0]*len(self.positions), [0]*len(self.positions)))		
	    else:
		print 'Error...exiting.'
		break
	    time.sleep(0.1)

if __name__ == '__main__':
    t=M3Proc()
    try:
        t.start()
    except (KeyboardInterrupt,EOFError):
        pass
    t.stop(force_safeop=False)
    print 'Exiting hand viz.'



