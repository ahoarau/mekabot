#! /usr/bin/python
# -*- coding: utf-8 -*-


# M3 -- Meka Robotics Robot Components
# Copyright (C) 2010 Meka Robotics
# Author: edsinger@mekabot.com (Aaron Edsinger)

# M3 is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# M3 is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.

# You should have received a copy of the GNU Lesser General Public License
# along with M3.  If not, see <http://www.gnu.org/licenses/>.
import m3
import time
import os
import roslib; roslib.load_manifest('meka_description')
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import m3.nanokontrol as m3k
import m3.gui as m3g
import m3.rt_proxy as m3p
import m3.humanoid as m3h
import m3.omnibase as m3o
import m3.toolbox as m3t
import m3.joint_zlift as m3z

import m3.component_factory as m3f
import time
import math
import numpy as npy
import PyKDL as kdl


if __name__ == '__main__':
    proxy = m3p.M3RtProxy()
    proxy.start()
    
    zlift = None
    zlift_names = proxy.get_available_components('m3joint_zlift')
    if len(zlift_names) != 1:
        print 'Zlift not found. Proceeding...'
    else:
        print 'Zlift names : ', zlift_names
        zlift = m3z.M3JointZLift(zlift_names[0])
        proxy.subscribe_status(zlift)

    omni = None
    base_name = proxy.get_available_components('m3omnibase')
    print 'Base names : ', base_name

    if len(base_name) != 1:
            print 'Omnibase not found. Exiting'
            exit()
    
    omni = m3o.M3OmniBase(base_name[0])
    proxy.subscribe_status(omni)
    if omni == None and zlift == None:
        exit()
        
    bot_name = m3t.get_robot_name()
    print 'bot name : ', bot_name
    bot = m3h.M3Humanoid(bot_name)   
    proxy.subscribe_status(bot)

    all_components = proxy.get_available_components()
    hands = [x for x in all_components if x.find('hand') != -1]
    
    print 'Hands available : ', hands
    if len(hands) > 0:
        hand_name = hands[0]
        print 'Using first hand : ', hand_name
    else:
        print 'No hands found'
        exit()
    hand=m3.hand.M3Hand(hand_name)
    proxy.subscribe_status(hand)
    #proxy.publish_command(hand)
    #proxy.publish_param(hand) 
    print 'M3Hand ndof : ', hand.ndof
    print 'arm ndof : ', bot.get_num_dof('right_arm')
    print 'head ndof : ', bot.get_num_dof('head')
    # Calibrate ZLift
    if zlift is not None:
        time.sleep(0.5)
        proxy.step()
        zlift.calibrate(proxy)
        time.sleep(0.5)
        proxy.step()
    
    # Calibrate Base
    if omni is not None:
        time.sleep(0.5)
        proxy.step()
        omni.calibrate(proxy)
        time.sleep(0.5)
        proxy.step()
        
    ndof_finger = 3
    
    flex_factor_index = [0.3] * ndof_finger 
    flex_factor_ring = [0.3] * ndof_finger
    flex_factor_pinky = [0.3] * ndof_finger
    flex_factor_thumb = [0.3] * 2
    joints = []
    omni.set_local_position(0,0,0,proxy)
    omni.set_global_position(0,0,0,proxy)
    joints.append('X')    
    joints.append('Y')   
    joints.append('yaw')    
    joints.append('zlift_joint')    
    
    joints.append('right_arm_j0')
    joints.append('right_arm_j1')
    joints.append('right_arm_j2')
    joints.append('right_arm_j3')
    joints.append('right_arm_j4')
    joints.append('right_arm_j5')
    joints.append('right_arm_j6')
    
    
    joints.append('right_hand_j0')
    joints.append('right_hand_j1')
    joints.append('right_hand_j2')
    joints.append('right_hand_j3')
    joints.append('right_hand_j4')
    joints.append('right_hand_j5')
    joints.append('right_hand_j6')
    joints.append('right_hand_j7')
    joints.append('right_hand_j8')
    joints.append('right_hand_j9')
    joints.append('right_hand_j10')
    joints.append('right_hand_j11')
    
    
    joints.append('head_j0')
    joints.append('head_j1')
    joints.append('head_j2')
    joints.append('head_j3')
    joints.append('head_j4')
    joints.append('head_j5')
    joints.append('head_j6')
    joints.append('head_j7')
    
    rospy.init_node("m3_joint_state_publisher")
    pub = rospy.Publisher("/joint_states", JointState)
    loop_rate = rospy.Rate(50.0)
    header = Header(0, rospy.Time.now(), '0')
    print 'Entering ROS Node'
    try:
        while not rospy.is_shutdown():
            header = Header(0, rospy.Time.now(), '0')
            positions = []
            # Omnibase state
            omni_pos = omni.get_local_position()
            omni_x = omni_pos[0]
            omni_y = omni_pos[1]
            omni_yaw = math.radians(omni_pos[2])
            zlift_z = zlift.get_pos_m()
            positions.append(omni_x)
            positions.append(omni_y)
            positions.append(omni_yaw)
            positions.append(zlift_z-(0.32))#sol->haut_base + haut_base->capteur(repère 0.0)
            # Arm joint states
            all_arm_joints = bot.get_theta_rad('right_arm')
            for i in xrange(0,bot.get_num_dof('right_arm')):
                positions.append(all_arm_joints[i])
            # Hand joint states
            th = hand.get_theta_rad()
            #Thumb
            positions.append(-th[0]+1.57) #0
            positions.append(th[1] * flex_factor_thumb[0])
            positions.append(th[1] * flex_factor_thumb[1])
            #Index
            positions.append(th[2] * flex_factor_index[0])
            positions.append(th[2] * flex_factor_index[1])
            positions.append(th[2] * flex_factor_index[2])
            #Ring
            positions.append(th[3] * flex_factor_ring[0])
            positions.append(th[3] * flex_factor_ring[1])
            positions.append(th[3] * flex_factor_ring[2])
            #Pinkie
            positions.append(th[4] * flex_factor_pinky[0])
            positions.append(th[4] * flex_factor_pinky[1])
            positions.append(th[4] * flex_factor_pinky[2])

            # Head state
            all_head_joints = bot.get_theta_rad('head')
            for i in xrange(0,bot.get_num_dof('head')):
                positions.append(all_head_joints[i])
            pub.publish(JointState(header, joints, positions, [0] * len(positions), [0] * len(positions)))
            proxy.step()
            loop_rate.sleep()
    except (KeyboardInterrupt, EOFError, rospy.ROSInterruptException):
        proxy.step()
        proxy.stop()
        print 'Exit'
        pass




