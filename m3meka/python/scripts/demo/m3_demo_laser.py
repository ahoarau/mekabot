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
import m3.component_factory as m3f
import m3.toolbox as m3t
import m3.trajectory as m3jt
import m3.rt_proxy as m3p
import m3.toolbox as m3t
import PyKDL as kdl
import m3.viz as m3v
import rospy
import roslib; roslib.load_manifest('hokuyo_node')
import roslib; roslib.load_manifest('sensor_msgs')
import roslib; roslib.load_manifest('m3_defs_ros')
from sensor_msgs.msg import LaserScan
import subprocess

def callback(data):
    print len(data.ranges), 'range points received.'
    
    

def listener():
    rospy.init_node('m3_laser', anonymous=True)
    rospy.Subscriber("/scan", LaserScan, callback)
    rospy.spin()

def main():
    
    p1 = subprocess.Popen(['roslaunch', 'm3_defs_ros', 'laser.launch'])    
    
    '''proxy = m3p.M3RtProxy()
    bot = m3f.create_component(m3t.get_robot_name()) # creates M3Humanoid class    
    bot.initialize(proxy)

    proxy.step'''
    
    try:
        listener()
    except:
        pass

    
    
    os.system("pkill -P " + str(p1.pid))
    os.kill(p1.pid,9)

if __name__ == "__main__":  
    main() 