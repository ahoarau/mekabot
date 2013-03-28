#! /usr/bin/python
# -*- coding: utf-8 -*-
import yaml
import m3.toolbox as m3t
import m3.gui as m3g
import math
import sys
from time import sleep
import roslib; roslib.load_manifest('mic_array')
import rospy
from mic_array.msg import MicArray
from mic_array.srv import MicArrayParam


class mic_array_param:
    def __init__(self):
        self.gui = m3g.M3Gui(stride_ms=125)
        self.num_chan = 6
        self.gains = [1.0] * self.num_chan
        self.bias = [0.0] * self.num_chan
        self.window = [1.0]
        self.threshold = [1.0]
        self.slew_rate = [0.0]
        self.save=False
        self.save_last=False
        self.zero=False
        self.zero_last=False
        self.zero_bias = [0.0]*self.num_chan
        self.mic_energy = [1.0]*self.num_chan

        # get yml file name from param server and parse for values
        self.config_file_name = rospy.get_param("/mic_array_config")
        try:
            f=file(self.config_file_name,'r')
            self.config= yaml.safe_load(f.read())
        except (IOError, EOFError):
            print 'Config file not present:',config_file_name
            sys.exit()
            
        if self.config.has_key('gains'):
            for i in range(self.num_chan):
                self.gains[i] = self.config['gains'][i]
        if self.config.has_key('bias'):
            for i in range(self.num_chan):
                self.bias[i] = self.config['bias'][i]
        if self.config.has_key('threshold'):            
                self.threshold[0] = self.config['threshold']
        if self.config.has_key('window_time'):            
                self.window[0] = self.config['window_time']
        if self.config.has_key('slew_rate'):            
                self.slew_rate[0] = self.config['slew_rate']*100.0
        rospy.init_node('mic_array_param', anonymous=True)  
        rospy.Subscriber("/mic_array", MicArray, self.callback)
        rospy.wait_for_service('mic_array_param')
        
    def start(self):
        self.gui.add('M3GuiSliders','Gains', (self,'gains'),range(len(self.gains)),[0.0,2.0],m3g.M3GuiWrite,column=1)
        self.gui.add('M3GuiSliders','Bias', (self,'bias'),range(len(self.gains)),[-20.0,20.0],m3g.M3GuiWrite,column=1)
        self.gui.add('M3GuiSliders','Slew', (self,'slew_rate'),range(1),[0.0,1000.0],m3g.M3GuiWrite,column=1)
        #self.gui.add('M3GuiSliders','Window', (self,'window'),range(len(self.window)),[-1.0,1.0],m3g.M3GuiWrite,column=1)
        #self.gui.add('M3GuiSliders','Threshold', (self,'threshold'),range(len(self.threshold)),[0.0,300.0],m3g.M3GuiWrite,column=1)
        self.gui.add('M3GuiToggle', 'Zero',      (self,'zero'),[],[['On','Off']],m3g.M3GuiWrite,column=1)
        self.gui.add('M3GuiToggle', 'Save',      (self,'save'),[],[['On','Off']],m3g.M3GuiWrite,column=1)
        self.gui.start(self.step)
        
    def step(self):
        if (self.zero and not self.zero_last):
            print 'e:', self.mic_energy
            for i in range(self.num_chan):
                self.zero_bias[i] = math.sqrt(self.mic_energy[i])/self.gains[i] + self.bias[i]
            print 'z:', self.zero_bias
        self.zero_last = self.zero
        if (self.save and not self.save_last):
            for i in range(self.num_chan):
                self.config['gains'][i] = self.gains[i]
                if self.zero:
                    self.config['bias'][i] = self.zero_bias[i]
                else:
                    self.config['bias'][i] = self.bias[i]
            self.config['threshold'] = self.threshold[0]
            self.config['window_time'] = self.window[0]
            self.config['slew_rate'] = self.slew_rate[0]/100.0
            try:
                f=file(self.config_file_name,'w')
                print 'Saving...',self.config_file_name
                f.write(yaml.safe_dump(self.config, default_flow_style=False,width=200))
                f.close()
            except (IOError, EOFError):
                print 'Config file not present:',self.config_file_name
                return
        self.save_last=self.save
        
        try:
            mic_array_param = rospy.ServiceProxy('mic_array_param', MicArrayParam)
            if self.zero:
                #print self.mic_energy
                resp = mic_array_param(self.gains,self.zero_bias ,self.window[0],self.threshold[0],self.slew_rate[0]/100.0)
            else:
                resp = mic_array_param(self.gains,self.bias,self.window[0],self.threshold[0],
                self.slew_rate[0]/100.0)
            #print resp.response
            
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        
        sleep(0.1)

    def callback(self,data):
        #print data.mic_energy
        for i in range(self.num_chan):
            self.mic_energy[i] = data.mic_energy[i]
        

t=mic_array_param()
try:
    t.start()
except (KeyboardInterrupt):
    pass

    
