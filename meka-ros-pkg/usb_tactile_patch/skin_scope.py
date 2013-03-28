#! /usr/bin/env python


import numpy as np
import roslib; roslib.load_manifest('usb_tactile_patch')
import rospy
from usb_tactile_patch.msg import UsbTactilePatch
from std_msgs.msg import Int32
import cv
import Gnuplot

#yrange: [min,max]
class TaxelScope():
        def __init__(self,n=12,xwidth=200,yrange=None):
                self.n=n
                self.y=[]
                for i in range(n):
                        self.y.append([0.0]*xwidth)
                self.x=range(xwidth)
                self.g = Gnuplot.Gnuplot(persist = 1)
                self.g.title('12Ch Taxel Plot')
                self.g('set data style lines')
                self.g('set term x11 noraise')
                if yrange is not None:
                        self.g('set yrange ['+str(yrange[0])+':'+str(yrange[1])+']')
        def plot(self,y):
                d=[]
                for i in range(self.n):
                        self.y[i].pop(0)
                        self.y[i].append(y[i])
                        d.append(Gnuplot.Data(self.x,self.y[i]))
                self.g.plot(*d)

initial_reading = True

num_values = 12
raw_value = [0]*num_values
initial_value = [0]*num_values

def callback_tactile(data):
        global surf_value
        global initial_reading
        global initial_value

        if initial_reading:
                initial_reading = False    
                for i in range(num_values):
                        initial_value[i] = data.tactile_value[i]
                print 'initial reading!', data.tactile_value

        for i in range(num_values):
                raw_value[i] = data.tactile_value[i] - initial_value[i]
        #print 'Peak',max(raw_value)


rospy.init_node('skin_scope', anonymous=True)
rospy.Subscriber("/usb_tactile_patch", UsbTactilePatch, callback_tactile)

print "Starting node_leaf scope.  Press Ctrl-C to exit."
scope=TaxelScope()
while not rospy.is_shutdown():

        scope.plot(raw_value)
        #print raw_value
        rospy.sleep(0.01)

