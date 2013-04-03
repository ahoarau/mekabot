#!/usr/bin/python
'''
Created on Mar 25, 2013

@author: meka
'''
import roslib
roslib.load_manifest('m3perception')
import rospy
from sensor_msgs.msg import Image
import message_filters
import cv
from cv_bridge import CvBridge, CvBridgeError
import threading
from threading import Thread,Lock
import os
import yaml
import time
import m3.toolbox as m3tools
#class Lock():
#    def __init__(self):
#        self.lock=False
#    def acquire(self):
#        self.lock=True
#    def release(self):
#        self.lock=False
#    def locked(self):
#        return self.lock

class M3compress_topic(Thread):
    def __init__(self,topic,compress_mode = 'compressed'):
        Thread.__init__(self)
        assert not isinstance(topic, list)
        self.topic_in = topic
        self.topic_out = topic+'_comp' 
        self.compress_mode = compress_mode
        #print 'topic in : ',self.topic_in
        #print 'topic out : ',self.topic_out
        #print 'compress mode : ',self.compress_mode
    def run(self):
        ##print '[Using compression]'
        cmd = 'rosrun image_transport republish ' +self.compress_mode+' in:='+self.topic_in+' out:='+self.topic_out
        #print 'cmd system : ',cmd
        os.system(cmd)

class M3image_subscriber(Thread):
    '''
    Generic tool for using meka's embedded vision
    '''
    def __init__(self,topics,use_compression=True,loop_rate = 30.0):
        Thread.__init__(self)
        if not isinstance(topics, list):
            topics = [topics]
        #print 'Creating m3util thread'
        #print 'number of topics : ',len(topics)
        #print 'topic : ',topic
        print 'Thread ',self,' subscribing to : ',topics
        self.mutex = [Lock()]*len(topics)
        self.topics = topics
        self.has_received_first = [False]*len(topics)
        
        if(use_compression):
            self.start_compression_threads()
        
        self.images=[]
        self._stop = threading.Event()
        rospy.init_node('m3cameras_util', anonymous=True)
        self.loopy = rospy.Rate(loop_rate)
        if len(topics) == 1:
            #print 'only one subscriber  ',self.topics
            rospy.Subscriber(self.topics[0], Image,self.callback,queue_size=1, buff_size=2**24)
        else:
            sub=[]
            for topic in topics:
                #print 'subscribing to ',topic
                sub.append(message_filters.Subscriber(topic, Image))
            #print 'sub : ',sub
            ts = message_filters.TimeSynchronizer(sub, 20)
            ts.registerCallback(self.callback)
        self.start()
    def start_compression_threads(self):
        self.comp=[]
        #print 'comp : ',self.comp
        for i in xrange(0,len(self.topics)):
            isdepth = self.topics[i].find('/depth') >=0
            if not isdepth:
                self.comp.append(M3compress_topic(self.topics[i],compress_mode='compressed'))
            else:
                self.comp.append(M3compress_topic(self.topics[i],compress_mode='compressedDepth'))
        #print 'comp : ',self.comp
        for i in xrange(0,len(self.topics)):
            self.topics[i] = self.comp[i].topic_out
            self.comp[i].start()
            
    def stop(self):
        cv.DestroyAllWindows()
        self._stop.set()        
        print self,' stopped'

        
    def run(self):
        while not rospy.is_shutdown():
            #print self,' waiting'
            self.loopy.sleep()
        self.stop()
            
    def callback(self,*msg):
        for i in range(len(self.topics)):
            try:
                bridge = CvBridge()
                self.mutex[i].acquire()
                enc = msg[i].encoding
                if enc=='rgb8':
                    enc='bgr8'
                #print 'ENCODING : ',enc
                if not self.has_received_first[i]:
                    #print 'Receiving first image...'
                    self.images.append( bridge.imgmsg_to_cv(msg[i],enc))
                    self.has_received_first[i] = True
                else:
                    self.images[i] = bridge.imgmsg_to_cv(msg[i],enc)
                self.mutex[i].release()
            except CvBridgeError, e:
                print e
    def get_image(self,i=0):
        if not self.mutex[i].locked():
            return self.images[i]
        
    def show_image(self,window_name,image):
        if not rospy.is_shutdown(): 
            try:
                cv.ShowImage(window_name, image)
                cv.WaitKey(3)
            except TypeError,e:
                print e

    def show(self):
        for i in range(len(self.topics)): 
            if not self.mutex[i].locked() and self.has_received_first[i]:
                ##print 'showing image[',i,'], name[',self.topics[i],']'
                self.show_image(self.topics[i]+str(i), self.images[i])
            #else:
                #print 'mutex[',i,'] locked'
