#!/usr/bin/python
'''
Created on Mar 25, 2013

@author: meka
'''
import m3.toolbox as m3tools
from m3perception import m3image_subscriber as m3is
import yaml

class M3cameras:
    def __init__(self,use_compression=True):
        config_filename = 'cameras/m3cameras_topics.yml'
        left_topic,right_topic = self.get_ros_config(config_filename)
        self.left_th = m3is.M3image_subscriber(left_topic,use_compression,loop_rate=15.0)
        self.right_th = m3is.M3image_subscriber(right_topic,use_compression,loop_rate=15.0)
    def get_ros_config(self,config_filename):
        ros_config_path = m3tools.get_m3_ros_config_path()
        filename= ros_config_path+config_filename
        f=file(filename,'r')
        config= yaml.safe_load(f.read())
        left = config['left']
        right = config['right']
        assert not (left==None or left==[])
        assert not (right==None or right==[])
        return left,right
    def get_left(self):
        return self.left_th.get_image()
    def get_right(self):
        return self.right_th.get_image()
    def show_left(self):
        self.left_th.show()
    def show_right(self):
        self.right_th.show()
    def show_both(self):
        self.show_left()
        self.show_right()
    
#if __name__ == '__main__':
#    #How to use that utility
#    m3cams = M3cameras()
#    m3kinect = M3kinect()
#    while not rospy.is_shutdown():
#        time.sleep(1.0/30.0)
#        m3cams.show_both()
#        m3kinect.show_rgb()
#        m3kinect.show_depth()
#        m3kinect.show_depth_degistered()



