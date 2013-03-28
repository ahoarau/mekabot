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
#import roslib; roslib.load_manifest('face_detector_mono')
import roslib; roslib.load_manifest('m3_defs_ros')
import roslib; roslib.load_manifest('m3meka_msgs')
#import roslib; roslib.load_manifest('mic_array')
#from face_detector_mono.msg import RectArray
#from mic_array.msg import MicArray
from m3meka_msgs.msg import M3OmnibaseJoy
import subprocess
import os
import time
from threading import Thread

#import time
#import rospy
#import roslib; roslib.load_manifest('face_detector_mono')
#import roslib; roslib.load_manifest('m3_defs_ros')
#from face_detector_mono.msg import RectArray
#import subprocess
#import os
#from threading import Thread

# ######################################################
class M3MicrophoneArrayThread(Thread):
    def __init__ (self,verbose=True,ra=1.0,rm=0.1):
	Thread.__init__(self)
	self.detect=None
	self.detect_time=None
	self.verbose=verbose
	self.slew_mag=m3t.M3Slew()
	self.slew_angle=m3t.M3Slew()
	self.rate_mag=rm
	self.rate_ang=ra
        def __init__(self):
                self.val=0.0
        
    def start(self):
	if self.verbose:
	    print 'Starting M3MicrophoneArrayThread...'
	rospy.init_node('m3_mic_array', anonymous=True,disable_signals=True) #allow Ctrl-C to master process
	rospy.Subscriber("/mic_array", MicArray, self.callback)
	Thread.start(self)
    def stop(self):
	rospy.signal_shutdown('Exiting')
    def run(self):
	rospy.spin()
    def get_detection(self):
	return self.detect,self.detect_time
    def callback(self,data):
	self.energy=data.mic_energy
	self.angle=data.angle
	self.mag=data.mag
	if self.verbose:
	    print 'E',self.energy
	    print 'A',self.angle
	    print 'M',self.mag
	
	    
# ######################################################	    
#Assumes that services _face_detect.launch and pt_grey.launch are running
class M3FaceDetectThread(Thread):
    def __init__ (self,eye,verbose=True):
	Thread.__init__(self)
	self.eye=eye
	self.detect=None
	self.detect_time=None
	self.verbose=verbose
    def start(self):
	if self.verbose:
	    print 'Starting M3FaceDetectThread...'
	rospy.init_node('m3_face_detect', anonymous=True,disable_signals=True) #allow Ctrl-C to master process
	if self.eye=='left':
	    rospy.Subscriber("/facedetect0/faces", RectArray, self.callback)
	if self.eye=='right':
	    rospy.Subscriber("/facedetect1/faces", RectArray, self.callback)
	if self.eye=='middle':
	    rospy.Subscriber("/facedetect2/faces", RectArray, self.callback)
	Thread.start(self)
    def stop(self):
	rospy.signal_shutdown('Exiting')
    def run(self):
	rospy.spin()
    def get_detection(self):
	return self.detect,self.detect_time
    def callback(self,data):
	if not len(data.rects) == 0:
	    if self.verbose:
		print time.time(),': Face detected at:'
		print data.rects
	    self.detect=data.rects
	    self.detect_time=time.time()
	    
#Assumes that services _face_track.launch and pt_grey.launch are running
class M3FaceTrackThread(Thread):
    def __init__ (self,eye,verbose=True):
	Thread.__init__(self)
	self.eye=eye
	self.track=None
	self.track_time=None
	self.verbose=verbose
    def start(self):
	if self.verbose:
	    print 'Starting M3FaceTrackThread...'
	rospy.init_node('m3_face_track', anonymous=True,disable_signals=True) #allow Ctrl-C to master process
	if self.eye=='left':
	    rospy.Subscriber("/facetrack0/faces", RectArray, self.callback)
	if self.eye=='right':
	    rospy.Subscriber("/facetrack1/faces", RectArray, self.callback)
	if self.eye=='middle':
	    rospy.Subscriber("/facetrack2/faces", RectArray, self.callback)
	Thread.start(self)
    def stop(self):
	rospy.signal_shutdown('Exiting')
    def run(self):
	rospy.spin()
    def get_detection(self):
	return self.track,self.track_time
    def callback(self,data):
	if not len(data.rects) == 0:
	    if self.verbose:
		print time.time(),': Face tracked at:'
		print data.rects
	    self.track=data.rects
	    self.track_time=time.time()	    
	    

#Assumes that services joy.launch
class M3OmnibaseJoystickThread(Thread):
    def __init__ (self,verbose=True):
	Thread.__init__(self)
	
	self.jx=0.0
	self.jy=0.0
	self.jyaw=0.0
	self.jz=0.0
	self.jbutton=-1	
	self.verbose=verbose
    def start(self):
	if self.verbose:
	    print 'Starting M3OmnibaseJoystickThread...'
	rospy.init_node('m3_omnibase_sixaxis', anonymous=True,disable_signals=True) #allow Ctrl-C to master process
	
	rospy.Subscriber("/omnibase_joy", M3OmnibaseJoy, self.callback)
	
	Thread.start(self)
    def stop(self):
	rospy.signal_shutdown('Exiting')
    def run(self):
	rospy.spin()
    
    def callback(self,data):

	if self.verbose:
	    print data
	self.jx=data.x
	self.jy=data.y
	self.jyaw=data.yaw
	self.jz = data.z
	self.jbutton=data.button
