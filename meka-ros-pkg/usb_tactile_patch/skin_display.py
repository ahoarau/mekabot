#! /usr/bin/env python

from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib import colors
from matplotlib.ticker import LinearLocator, FixedLocator, FormatStrFormatter
import matplotlib.pyplot as plt
import numpy as np
import roslib; roslib.load_manifest('usb_tactile_patch')
import rospy
from usb_tactile_patch.msg import UsbTactilePatch
from std_msgs.msg import Int32
import cv


max_surf= 7500 #default
max_max_surf = 15000#65535
min_max_surf = 0#1000


'''max_max_surf = 30
min_max_surf = 2'''
#max_surf= 20 #default

initial_reading = True

num_values = 12
surf_value = [0]*num_values
initial_value = [0]*num_values

def callback_kb(data):
    global max_surf

    max_surf = min_max_surf + int(data.data) * int((max_max_surf - min_max_surf)/100)
    

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
        surf_value[i] = data.tactile_value[i] - initial_value[i]
    print 'Peak',max(surf_value)
        
    
plt.interactive(True)    
    

fig = plt.figure()
#ax = fig.gca(projection='3d') for matplotlib ver 1.01
ax = Axes3D(fig)


rospy.init_node('skin_display', anonymous=True)
rospy.Subscriber("/usb_tactile_patch", UsbTactilePatch, callback_tactile)
rospy.Subscriber("/leaf_zoom", Int32, callback_kb)

size = 6
scale = 4

bigger = max_surf*np.ones((size*scale, size*scale))
row = np.zeros((len(bigger),len(bigger)))
col = np.zeros((len(bigger),len(bigger)))
for row_num in range(len(bigger)):
        for col_num in range(len(bigger)):
            row[row_num,col_num] = row_num
            col[row_num,col_num] = col_num
surf = ax.plot_surface(row, col,bigger, rstride=1, cstride=1, cmap=cm.jet, linewidth=0, antialiased=False)
ax.set_zlim3d(0,max_surf)
plt.draw()
my_cmap = plt.get_cmap()

print "Starting node_leaf visualizer.  Press Ctrl-C to exit."

while not rospy.is_shutdown():
    '''if max(surf_value)>max_surf:
        max_surf=max(surf_value)'''
    
    var = 1 * np.array([[0, 0, 0, 0, 0, 0],
        [0, surf_value[7], surf_value[8], surf_value[10], 0, 0],
        [0, surf_value[6], surf_value[9], surf_value[11], 0, 0],
        [0, surf_value[3], surf_value[0], surf_value[5], 0, 0],
        [0, surf_value[4], surf_value[1], surf_value[2], 0, 0],
        [0, 0, 0, 0, 0, 0]],np.int32)
        
    #palette = plt.matplotlib.colors.LinearSegmentedColormap('jet3',plt.cm.datad['jet'],max_surf)

    surf.remove()    
    
    big_cv = cv.CreateImage((size*scale,size*scale), cv.IPL_DEPTH_16U, 1)
    var_cv = cv.CreateImage((size,size), cv.IPL_DEPTH_16U, 1)

    for i in range(len(var)):
        for j in range(len(var)):
            var_cv[i,j] = var[i,j]
       
    cv.Resize(var_cv, big_cv, cv.CV_INTER_CUBIC)
    
    for i in range(len(bigger)):
        for j in range(len(bigger)):
            if big_cv[i,j] > max_surf:
                bigger[i,j] = max_surf
            else:
                bigger[i,j] = big_cv[i,j]
                
                
    for i in range(len(bigger)):
        bigger[i,-3] = max_surf
        bigger[i,-2] = max_surf
        bigger[i,-1] = max_surf
       
    surf = ax.plot_surface(row, col, bigger, rstride=1, cstride=1, cmap=cm.jet,linewidth=0, antialiased=False)
    
    ax.set_zlim3d(0,max_surf)    
    ax.set_ylim3d(0,len(bigger)-4)
    
    plt.draw()\
        
    rospy.sleep(0.08)

    
