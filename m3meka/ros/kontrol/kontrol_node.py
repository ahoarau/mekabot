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
import roslib; roslib.load_manifest('kontrol')
import rospy
#from kontrol.msg import Kontrol
import subprocess
import sys
sys.path.insert(0, '../pyalsa')
import getopt
import os
import struct
import alsaseq
import traceback
import time
from kontrol.msg import Kontrol
from alsaseq import *
from aseqdump import *


class KontrolPub:
    
    def update_state(self, param, value):
        for i in range(len(self.slider_idx)):
            if self.slider_idx[i] == param:
                self.slider_val[i] = value
                return
            
        for i in range(len(self.knob_idx)):
            if self.knob_idx[i] == param:
                self.knob_val[i] = value
                return
        
        for i in range(len(self.button_idx)):
            if self.button_idx[i] == param:
                self.button_val[i] = value
                return
            
    
    def handle_event(self, event):
        #print "%3d:%-3d" % ((event.source)[0], (event.source)[1]),
        type = event.type
        data = event.get_data()
        
        if type == SEQ_EVENT_CONTROLLER:
            #print "Control change         %2d %3d %3d" % \
            #    (data['control.channel'], data['control.param'], data['control.value'])
            self.update_state(data['control.param'], data['control.value'])
    
    def __init__(self):
        self.slider_idx = [2, 3, 4, 5, 6, 8, 9, 12, 13]
        self.knob_idx = [14, 15, 16, 17, 18, 19, 20, 21, 22]
        self.button_idx = [23, 33, 24, 34, 25, 35, 26, 36, 27, 37, 28, 38, 29, 39, 30, 40, 31, 41]

        self.slider_val = [0]*len(self.slider_idx)
        self.knob_val = [0]*len(self.knob_idx)
        self.button_val = [0]*len(self.button_idx)

        rospy.init_node('kontrol_node', anonymous=True)
        
        self.pub = rospy.Publisher('kontrol', Kontrol)
        
        sequencer = init_seq()
        ports = [(20,0)]
        end_delay = 2
        
        source_port = create_source_port(sequencer)             
        
        connect_ports(sequencer, source_port, ports)
    
        sequencer.mode = SEQ_NONBLOCK
    
        
        print 'Move Active Sliders to Initialize Positions (Midi is non-polling)'
        print 'Press Ctrl+C to end.'
    
        while True:
            try:
                eventlist = sequencer.receive_events(timeout=100, maxevents = 1)
                for event in eventlist:
                    self.handle_event(event)
                    msg = Kontrol(self.slider_val, self.knob_val, self.button_val)
                    self.pub.publish(msg)
            except:
            #except KeyboardInterrupt:
                pass
                break
        
        try:            
            rospy.spin()
        except:
            pass
        

def main():
    kontrolNode = KontrolPub()

    
if __name__ == "__main__":  
    main() 