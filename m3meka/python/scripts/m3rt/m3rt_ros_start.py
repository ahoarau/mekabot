#! /usr/bin/python

#Copyright  2010, Meka Robotics
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

import m3.rt_proxy as m3p
import m3.toolbox as m3t

PKG = 'm3_client' # this package name

import roslib; roslib.load_manifest(PKG) 
import subprocess
import sys
import os
import yaml
import rospy
import time


# build it up
p = subprocess.Popen(['roslaunch', 'm3_defs_ros', 'ros.launch'])
time.sleep(1.0)
proxy = m3p.M3RtProxy()
proxy.start(start_data_svc=False,start_ros_svc=True)

proxy.make_operational_all()
    
try:
    while True:        
        pass
except:
    print "Exiting.."


# tear it down
proxy.stop()
os.system("pkill -P " + str(p.pid))
os.kill(p.pid,9)
