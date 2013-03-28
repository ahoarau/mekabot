#! /usr/bin/python
# -*- coding: utf-8 -*-

import Gnuplot
from time import sleep
from datetime import datetime
import struct
import math
import sys
import roslib; roslib.load_manifest('mic_array')
import rospy
from mic_array.msg import MicArray
from mic_array.srv import MicArrayParam

num_chan = 6
x0 = [0.0]*num_chan
y0 = [0.0]*num_chan
xd = [0.0]*num_chan
yd = [0.0]*num_chan            

e = None

def callback(data):
    xd_p = []
    yd_p = []
    for i in range(num_chan):
        xd_p.append(xd[i] * data.mic_energy[i])
        yd_p.append(yd[i] * data.mic_energy[i])
    e.plot(zip(x0, y0, xd_p, yd_p))
    m=data.mag
    a=data.angle
    e.replot([[0,0,m*math.cos(math.radians(a)),m*math.sin(math.radians(a))]])

    

yrange = [-100,100]
xrange = [-100,100]
e = Gnuplot.Gnuplot(persist = 0)
e.title('mic energy')
e('set data style vectors')
#self.e('set style line 1 linecolor rgb "blue"')
e('set term x11 noraise')
e('set xrange ['+str(xrange[0])+':'+str(xrange[1])+']')
e('set yrange ['+str(yrange[0])+':'+str(yrange[1])+']')

for i in range(num_chan):
    t = (2*math.pi/num_chan)*i
    xd[i] = math.sin(t)
    yd[i] = math.cos(t)

e.plot(zip(x0,y0,xd,yd))

rospy.init_node('mic_array_plot', anonymous=True)
rospy.Subscriber("/mic_array", MicArray, callback)

try:            
    rospy.spin()
except (KeyboardInterrupt):
    pass