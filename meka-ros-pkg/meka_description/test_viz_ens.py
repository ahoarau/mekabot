#! /usr/bin/python
# -*- coding: utf-8 -*-


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
import roslib; roslib.load_manifest('meka_description')
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header


rospy.init_node("joint_state_publisher")
pub = rospy.Publisher("/joint_states", JointState)


joints = []
positions = []

joints.append('X')
positions.append(0.0)

joints.append('Y')
positions.append(0.0)

joints.append('yaw')
positions.append(0.0)

joints.append('zlift_joint')
positions.append(0.0)

'''joints.append('left_arm_j0')
positions.append(0.0)
joints.append('left_arm_j1')
positions.append(0.0)
joints.append('left_arm_j2')
positions.append(0.0)
joints.append('left_arm_j3')
positions.append(0.0)
joints.append('left_arm_j4')
positions.append(0.0)
joints.append('left_arm_j5')
positions.append(0.0)
joints.append('left_arm_j6')
positions.append(0.0)'''

joints.append('right_arm_j0')
positions.append(0.0)
joints.append('right_arm_j1')
positions.append(0.0)
joints.append('right_arm_j2')
positions.append(0.0)
joints.append('right_arm_j3')
positions.append(0.0)
joints.append('right_arm_j4')
positions.append(0.0)
joints.append('right_arm_j5')
positions.append(0.0)
joints.append('right_arm_j6')
positions.append(0.0)

joints.append('right_hand_j0')
positions.append(0.0)
joints.append('right_hand_j1')
positions.append(0.0)
joints.append('right_hand_j2')
positions.append(0.0)
joints.append('right_hand_j3')
positions.append(0.0)
joints.append('right_hand_j4')
positions.append(0.0)
joints.append('right_hand_j5')
positions.append(0.0)
joints.append('right_hand_j6')
positions.append(0.0)
joints.append('right_hand_j7')
positions.append(0.0)
joints.append('right_hand_j8')
positions.append(0.0)
joints.append('right_hand_j9')
positions.append(0.0)
joints.append('right_hand_j10')
positions.append(0.0)
joints.append('right_hand_j11')
positions.append(0.0)

'''joints.append('m3joint_ua_mh13_j0')
positions.append(0.0)
joints.append('m3joint_ua_mh13_j1')
positions.append(0.0)
joints.append('m3joint_ua_mh13_j2')
positions.append(0.0)
joints.append('m3joint_ua_mh13_j3')
positions.append(0.0)
joints.append('m3joint_ua_mh13_j4')
positions.append(0.0)
joints.append('m3joint_ua_mh13_j5')
positions.append(0.0)
joints.append('m3joint_ua_mh13_j6')
positions.append(0.0)
joints.append('m3joint_ua_mh13_j7')
positions.append(0.0)
joints.append('m3joint_ua_mh13_j8')
positions.append(0.0)
joints.append('m3joint_ua_mh13_j9')
positions.append(0.0)
joints.append('m3joint_ua_mh13_j10')
positions.append(0.0)
joints.append('m3joint_ua_mh13_j11')
positions.append(0.0)'''

joints.append('head_j0')
positions.append(0.0)
joints.append('head_j1')
positions.append(0.0)
joints.append('head_j2')
positions.append(0.0)
joints.append('head_j3')
positions.append(0.0)
joints.append('head_j4')
positions.append(0.0)
joints.append('head_j5')
positions.append(0.0)
joints.append('head_j6')
positions.append(0.0)





header = Header(0,rospy.Time.now(),'0')
pub.publish(JointState(header, joints, positions, [0]*len(positions), [0]*len(positions)))

try:
    while not rospy.is_shutdown():
	time.sleep(1.0)	
	header = Header(0,rospy.Time.now(),'0')
	pub.publish(JointState(header, joints, positions, [0]*len(positions), [0]*len(positions)))	    
except (KeyboardInterrupt,EOFError,rospy.ROSInterruptException):
    pass




