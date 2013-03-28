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

import os
import sys
import time
import yaml
import m3.unit_conversion as m3u
import m3.toolbox as m3t
#import Numeric as nu
import numpy as nu
import math
import copy
import random
import m3.toolbox_ros as m3tr

class M3OmniBaseJoy:
    def __init__(self):
        pass
    def start(self,proxy,omni,zlift=None):
        self.omni=omni
        self.zlift=zlift
        #max_lin_vel = 0.2  # m/s (0.6)
        #max_lin_acc = 0.15 #0.4 # m/s^2  (0.2)  --- 1.0 gives really good performance but saturates motors...
        #max_rot_vel = 20   # deg/s
        #max_rot_acc = 30 # 100 for better    # deg/s^2
        
        max_lin_vel = 0.6  # m/s (0.6)
        max_lin_acc = 0.4 #0.2 #0.3 #0.4 # m/s^2  (0.2)  --- 1.0 gives really good performance but saturates motors...
        max_rot_vel = 35   # deg/s
        max_rot_acc = 60 #30 #60 # 100 for better    # deg/s^2


        #max_lin_vel = 0.25  # m/s (0.6)
        #max_lin_acc = 1.0 #0.4 # m/s^2  (0.2)  --- 1.0 gives really good performance but saturates motors...
        #max_rot_vel = 35   # deg/s
        #max_rot_acc = 160  # 100 for better    # deg/s^2
        if omni is not None:
            omni.set_local_position(0,0,0,proxy)
            omni.set_global_position(0,0,0,proxy)
            omni.set_max_linear_accel(max_lin_acc)
            omni.set_max_linear_velocity(max_lin_vel)
            omni.set_max_rotation_velocity(max_rot_vel)
            omni.set_max_rotation_accel(max_rot_acc)
            omni.set_mode_joystick()
            omni.set_joystick_x(0)
            omni.set_joystick_y(0)
            omni.set_joystick_yaw(0)
            omni.set_joystick_button(-1)
        #omni.set_mode_op_space_force()
        if zlift is not None:
            self.zlift_min=0.0
            self.zlift_max=600.0
            self.des_zlift=0.0
            self.zlift_first=True
        proxy.step()
        time.sleep(4)
        self.f=m3tr.M3OmnibaseJoystickThread(verbose=False)
        print '---------------------------'
        print 'Now starting joystick control.'
        #print "Depress X to quit.\n"
        print '---------------------------'
        self.f.start()
    def stop(self):
        self.f.stop()
    def step(self):
        if self.omni is not None:
            self.omni.set_joystick_x(self.f.jx)
            self.omni.set_joystick_y(self.f.jy)
            self.omni.set_joystick_yaw(self.f.jyaw)
            self.omni.set_joystick_button(int(self.f.jbutton))  
        if self.zlift is not None:
            if self.zlift_first:
                self.des_zlift=self.zlift.get_pos_mm()
                self.zlift_first=False
            self.des_zlift=max(self.zlift_min,min(self.zlift_max,self.des_zlift+self.f.jz))
            #print 'Desired Z-Lift',self.des_zlift
            #print 'ZL',self.f.jx,self.f.jy,self.f.jyaw,self.f.jz
            self.zlift.set_mode_theta_gc()
            self.zlift.set_slew_rate_proportion(1.0)
            self.zlift.set_stiffness(1.0)
            self.zlift.set_pos_mm(self.des_zlift)
