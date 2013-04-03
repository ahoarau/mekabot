#!/usr/bin/python
'''
Created on Mar 25, 2013

@author: meka
'''
import m3.toolbox as m3tools
from m3perception import m3image_subscriber as m3is
import yaml

class M3kinect:
    def __init__(self,use_compression=True):
        config_filename = 'kinect/m3kinect_topics.yml'
        rgb_topic,depth_topic,depth_registered_topic= self.get_ros_config(config_filename)
        self.rgb_th = m3is.M3image_subscriber(rgb_topic,use_compression)
        self.depth_th = m3is.M3image_subscriber(depth_topic,use_compression)
        self.depth_registered_th = m3is.M3image_subscriber(depth_registered_topic,use_compression)
        
    def get_ros_config(self,config_filename):
        ros_config_path = m3tools.get_m3_ros_config_path()
        filename= ros_config_path+config_filename
        f=file(filename,'r')
        config= yaml.safe_load(f.read())
        rgb = config['rgb']
        depth = config['depth']
        depth_registered = config['depth_registered']
        assert not (depth==None or depth==[])
        assert not (depth_registered==None or depth_registered==[])
        assert not (rgb==None or rgb==[])
        return rgb,depth,depth_registered
    def get_rgb(self):
        return self.rgb_th.get_image()
    def get_depth(self):
        return self.depth_th.get_image()
    def get_depth_registered(self):
        return self.depth_registered_th.get_image()
    def show_rgb(self):
        self.rgb_th.show()
    def show_depth(self):
        self.depth_th.show()
    def show_depth_degistered(self):
        self.depth_registered_th.show()

    def is_shutdown(self):
        return self.rgb_th.is_alive() or self.depth_th.is_alive() or self.depth_registered_th.is_alive()
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



