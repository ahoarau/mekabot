#!/usr/bin/python
'''
Created on Mar 25, 2013

@author: meka
'''
import roslib
roslib.load_manifest('m3cam')
import rospy
from sensor_msgs.msg import Image
import message_filters
import cv
from cv_bridge import CvBridge, CvBridgeError
import threading
from threading import Thread,Lock
import os
import yaml
import m3.toolbox as m3tools

class M3compress_cameras_launch(Thread):
    def run(self):
        #print '[Using compression]'
        os.system('roslaunch m3cam compress_m3cameras.launch')
class M3cameras(Thread):
    '''
    A set of tools for using the meka's firewire cameras
    '''
    def __init__(self,image_comp=False,sync = False,loop_rate = 20.0):
        Thread.__init__(self)
        #print 'Creating cam object'
        self.mutex = Lock()
        self.has_received_first_left = False
        self.has_received_first_right = False
        self.config_file_name = 'cameras/m3cameras_topics.yml'
        left_topic,right_topic = self.get_ros_topics(image_comp)
        if(image_comp):
            self.m3comp = M3compress_cameras_launch()
            self.m3comp.start()
        self.left_img_cv=[]
        self.right_img_cv=[]
        self.left_topic = left_topic
        self.right_topic = right_topic
        self._stop = threading.Event()
        rospy.init_node('m3cameras_util', anonymous=True)
        self.loopy = rospy.Rate(loop_rate)
        if(sync == False):
            rospy.Subscriber(self.left_topic, Image,self.left_callback,queue_size=1, buff_size=2**24)
            rospy.Subscriber(self.right_topic, Image,self.right_callback,queue_size=1, buff_size=2**24)
        else:
            leftImageSub = message_filters.Subscriber(self.left_topic, Image)
            rightImageSub = message_filters.Subscriber(self.right_topic, Image)
            ts = message_filters.TimeSynchronizer([leftImageSub, rightImageSub], 10)
            ts.registerCallback(self.sync_images_callback)
        #self.run()
    def stop(self):
        #print 'Stop cam thread'
        cv.DestroyAllWindows()
        self._stop.set()
        
    def run(self):
        while not rospy.is_shutdown():
            #print 'waiting'
            self.loopy.sleep()
        self.stop()
    def get_ros_topics(self,image_comp=False):
        ros_config_path = m3tools.get_m3_ros_config_path()
        filename= ros_config_path+self.config_file_name
        f=file(filename,'r')
        config= yaml.safe_load(f.read())
        #print 'config : ',config
        left=None
        right=None
        default_left = '/meka_eyes/left/image_raw'
        default_right = '/meka_eyes/right/image_raw'
        default_left_comp = '/meka_left_comp'
        default_right_comp = '/meka_right_comp'

        if image_comp==False:
            left = config['left']
            right = config['right']
        else:
            left = config['left_comp']
            right = config['right_comp']
        if(left==None or right==None):
            print 'Error while reading ',filename,'using default topics'
        if image_comp==False:
            left=default_left
            right=default_right
        else:
            left=default_left_comp
            right=default_right_comp
        return left,right

    def left_callback(self,left_image_msg):
        try:
            bridge = CvBridge()
            self.mutex.acquire()
            self.left_img_cv = bridge.imgmsg_to_cv(left_image_msg, "bgr8")
            if not self.has_received_first_left:
                print 'Receiving first left images...'
                self.has_received_first_left = True
            self.mutex.release()
        except CvBridgeError, e:
            print e
            
    def right_callback(self,right_image_msg):
        try:
            bridge = CvBridge()
            self.mutex.acquire()
            self.right_img_cv = bridge.imgmsg_to_cv(right_image_msg, "bgr8")
            if not self.has_received_first_right:
                print 'Receiving first right images...'
                self.has_received_first_right = True
            self.mutex.release()
        except CvBridgeError, e:
            print e

    def sync_images_callback(self,left_image_msg,right_image_msg):
        try:
            bridge = CvBridge()
            self.mutex.acquire()
            self.left_img_cv=bridge.imgmsg_to_cv(left_image_msg, "bgr8")
            self.right_img_cv= bridge.imgmsg_to_cv(right_image_msg, "bgr8")
            if not self.has_received_first_left:
                print 'Receiving first left images...'
                self.has_received_first_left = True
            if not self.has_received_first_right:
                print 'Receiving first right images...'
                self.has_received_first_right = True
            self.mutex.release()
        except CvBridgeError, e:
            print e

    def show_image(self,window_name,image):
        try:
            cv.ShowImage(window_name, image)
            cv.WaitKey(3)
        except TypeError,e:
            print e
    
    def show_left(self):
        if not self.mutex.locked() and self.has_received_first_left:
            self.show_image('Left image',self.left_img_cv)
        
    def show_right(self):
        if not self.mutex.locked() and self.has_received_first_right:
            self.show_image('Right image',self.right_img_cv)
        
    def show_both(self):
        self.show_left()
        self.show_right()
    
    def get_left_img(self): 
        if not self.mutex.locked():
            return self.left_img_cv
        
    def get_right_img(self):
        if not self.mutex.locked():
            return self.right_img_cv
        
    def get_imgs(self):
        if not self.mutex.locked():
            return self.left_img_cv,self.right_img_cv
        
        
#if __name__ == '__main__':
#    #How to use that utility
#    print 'Creating thread'
#    #m3cam = M3cameras()
#    m3cam = M3cameras(image_comp=True)
#    print 'starting thread'
#    m3cam.start()
#    print 'Started'
#    import time
#    while not rospy.is_shutdown():
#        time.sleep(1.0/15.0)
#        print 'here'
#        m3cam.show_both()

##!/usr/bin/python
#'''
#Created on Mar 25, 2013
#
#@author: meka
#'''
#import roslib
#roslib.load_manifest('m3cam')
#import rospy
#from sensor_msgs.msg import Image
#import message_filters
#import cv
#from cv_bridge import CvBridge, CvBridgeError
#import threading
#from threading import Thread,Lock
#
#class M3cameras(Thread):
#    '''
#    A set of tools for using the meka's firewire cameras
#    '''
#    def __init__(self,sync_mode = False,loop_rate = 20.0,left_topic = '/meka_eyes/left/image_raw',right_topic = '/meka_eyes/right/image_raw'):
#        Thread.__init__(self)
#        self.mutex = Lock()
#        print 'Creating camera_ws thread'
#        self.has_received_first_left = False
#        self.has_received_first_right = False
#
#        self.left_img_cv=[]
#        self.right_img_cv=[]
#        self.left_topic = left_topic
#        self.right_topic = right_topic
#        self._stop = threading.Event()
#        rospy.init_node('m3cameras', anonymous=True)
#        self.loopy = rospy.Rate(loop_rate)
#        if(sync_mode == False):
#            rospy.Subscriber(self.left_topic, Image,self.left_callback,queue_size=1, buff_size=2**24)
#            rospy.Subscriber(self.right_topic, Image,self.right_callback,queue_size=1, buff_size=2**24)
#        else:
#            leftImageSub = message_filters.Subscriber(self.left_topic, Image)
#            rightImageSub = message_filters.Subscriber(self.right_topic, Image)
#            ts = message_filters.TimeSynchronizer([leftImageSub, rightImageSub], 10)
#            ts.registerCallback(self.sync_images_callback)
#    def stop(self):
#        cv.DestroyAllWindows()
#        self._stop.set()
#        
#    def run(self):
#        while not rospy.is_shutdown():
#            self.loopy.sleep()
#            
#    def left_callback(self,left_image_msg):
#        try:
#            bridge = CvBridge()
#            self.mutex.acquire()
#            self.left_img_cv = bridge.imgmsg_to_cv(left_image_msg, "bgr8")
#            if not self.has_received_first_left:
#                print 'Receiving left images...'
#                self.has_received_first_left = True
#            self.mutex.release()
#        except CvBridgeError, e:
#            print e
#            
#    def right_callback(self,right_image_msg):
#        try:
#            bridge = CvBridge()
#            self.mutex.acquire()
#            self.right_img_cv = bridge.imgmsg_to_cv(right_image_msg, "bgr8")
#            if not self.has_received_first_right:
#                print 'Receiving right images...'
#                self.has_received_first_right = True
#            self.mutex.release()
#        except CvBridgeError, e:
#            print e
#
#    def sync_images_callback(self,left_image_msg,right_image_msg):
#        try:
#            bridge = CvBridge()
#            self.mutex.acquire()
#            self.left_img_cv=bridge.imgmsg_to_cv(left_image_msg, "bgr8")
#            self.right_img_cv= bridge.imgmsg_to_cv(right_image_msg, "bgr8")
#            if not self.has_received_first_left:
#                print 'Receiving left images...'
#                self.has_received_first_left = True
#            if not self.has_received_first_right:
#                print 'Receiving right images...'
#                self.has_received_first_right = True
#            self.mutex.release()
#        except CvBridgeError, e:
#            print e
#
#    def show_image(self,window_name,image):
#        try:
#            cv.ShowImage(window_name, image)
#            cv.WaitKey(3)
#        except TypeError,e:
#            print e
#            pass
#    def show_left(self):
#        if not self.mutex.locked() and self.has_received_first_left:
#            self.show_image('Left image',self.left_img_cv)
#    def show_right(self):
#        if not self.mutex.locked() and self.has_received_first_right:
#            self.show_image('Right image',self.right_img_cv)
#    def show_both(self):
#        self.show_left()
#        self.show_right()
#    
#    def get_left_img(self): 
#        if not self.mutex.locked():
#            return self.left_img_cv
#    
#    def get_right_img(self):
#        if not self.mutex.locked():
#            return self.right_img_cv
#    
#    def get_imgs(self):
#        if not self.mutex.locked():
#            return self.left_img_cv,self.right_img_cv
#        
##if __name__ == '__main__':
##    #How to use that utility
##    m3cam = M3cameras(sync_mode=False,left_topic='/meka_left_comp',right_topic='/meka_right_comp')
##    m3cam.start()
##    import time
##    while 1:
##        time.sleep(0.1)
##        m3cam.show_both()