#! /usr/bin/python
# -*- coding: utf-8 -*-

# -*- coding: utf-8 -*-
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
import os
import roslib; roslib.load_manifest('m3_defs_ros')
import rospy
#import roslib; roslib.load_manifest('joy')
import roslib; roslib.load_manifest('sensor_msgs')

from sensor_msgs.msg import Joy
import roslib; roslib.load_manifest('m3_client')
from m3_client.msg import M3OmnibaseJoy

#from sensor_msgs.msg import JointState
#from roslib.msg import Header
import subprocess

class M3SixAxis:        
    def __init__(self):        
        self.left = 5
        self.right = 7
        self.fwd = 4
        self.back = 6
        self.turn_left = 8
        self.turn_right = 9
        self.turn_left_2 = 10
        self.turn_right_2 = 11
        
        self.tri_but = 12
        self.circ_but = 13
        self.x_but = 14
        self.sqr_but = 15
        
        #self.scale_xy = 0.75
        #self.scale_rot = 60.0
        
        self.scale_xy = 1.0
        self.scale_rot = 1.0
        self.joy_max = -1.0
 
        
#        self.p1 = subprocess.Popen(['sudo', 'ps3joy.py'])
#        self.p2 = subprocess.Popen(['roslaunch', 'm3_defs_ros', 'joy.launch'])
        
        rospy.init_node('m3_sixaxis', anonymous=True)
        rospy.Subscriber("/joy", Joy, self.callback)
        self.pub = rospy.Publisher('omnibase_joy', M3OmnibaseJoy)
        
        try:            
            rospy.spin()
        except:
            pass
        
    def callback(self, data):
        
        self.pub.publish(self.get_x(data.axes), self.get_y(data.axes), self.get_yaw(data.axes), self.get_button(data.buttons),self.get_z(data.axes))
        
    def get_z(self,axes):
        
        axis_up = float(axes[self.circ_but])/self.joy_max
        axis_down = float(axes[self.sqr_but])/self.joy_max
        if (axis_up > axis_down):
            z = axis_up * self.scale_xy
        else:
            z = -axis_down * self.scale_xy
        return z
        
    def get_y(self,axes):
        
        axis_left = float(axes[self.left])/self.joy_max        
        axis_right = float(axes[self.right])/self.joy_max        
        if (axis_left > axis_right):
            y = -axis_left * self.scale_xy
        else:
            y = axis_right * self.scale_xy
        return y
            
    def get_x(self,axes):
        
        axis_fwd = float(axes[self.fwd])/self.joy_max
        axis_back = float(axes[self.back])/self.joy_max
        if (axis_fwd > axis_back):
            x = axis_fwd * self.scale_xy
        else:
            x = -axis_back * self.scale_xy
        return x
    
    def get_yaw(self,axes):        
        axis_turn_left = float(max(axes[self.turn_left]/self.joy_max,axes[self.turn_left_2]/self.joy_max))
        axis_turn_right = float(max(axes[self.turn_right]/self.joy_max,axes[self.turn_right_2]/self.joy_max))
        if (axis_turn_left > axis_turn_right):
            head = axis_turn_left * self.scale_rot
        else:
            head = -axis_turn_right * self.scale_rot
        return head
            
    def get_button(self,buttons):        
        if buttons[self.tri_but]:
            return 0
        '''elif buttons[self.circ_but]:        
            return 1
        elif buttons[self.sqr_but]:
            return 2
        elif buttons[self.x_but]:
            return 3'''
        return -1

def main():
    six = M3SixAxis()

    
if __name__ == "__main__":  
    main() 