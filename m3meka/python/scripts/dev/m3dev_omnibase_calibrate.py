#! /usr/bin/python

#Copyright  2008, Meka Robotics
#All rights reserved.
#http://mekabot.com

#Redistribution and use in source and binary forms, with or without
#modification, are permitted. 


#THIS SOFTWARE IS PROVIDED BY THE Copyright HOLDERS AND CONTRIBUTORS
#"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#Copyright OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES INCLUDING,
#BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#POSSIBILITY OF SUCH DAMAGE.

import time
import m3.joint_zlift as m3z
import m3.toolbox as m3t
import m3.unit_conversion as m3u
import numpy as nu
import m3.rt_proxy as m3p
import m3.component_factory as m3f
import m3.omnibase as m3o

proxy = m3p.M3RtProxy()
proxy.start()
proxy.make_operational_all()

#Setup Components
zlift=None
zlift_names=proxy.get_available_components('m3joint_zlift')
if len(zlift_names)!=1:
    print 'Zlift not found. Proceeding...'
else:
    zlift=m3z.M3JointZLift(zlift_names[0])
    proxy.subscribe_status(zlift)
    proxy.publish_command(zlift)
    proxy.publish_param(zlift) 

omni=None
base_name=proxy.get_available_components('m3omnibase')
if len(base_name)!=1:
        print 'Omnibase not found. Proceeding...'
else:
    omni=m3o.M3OmniBase(base_name[0])
    proxy.publish_param(omni) # we need this for calibration
    proxy.subscribe_status(omni)
    proxy.publish_command(omni)
    
if omni==None and zlift==None:
    exit()
    
if zlift is not None:
    pwr_rt=m3t.get_joint_pwr_component_name(zlift_names[0])
else:
    pwr_rt=m3t.get_omnibase_pwr_component_name(base_name[0])
    
pwr=m3f.create_component(pwr_rt)
proxy.publish_command(pwr)
pwr.set_motor_power_on()
proxy.make_operational_all()
proxy.step()
time.sleep(0.5)
proxy.step()

zlift_shm_names=proxy.get_available_components('m3joint_zlift_shm')
if len(zlift_shm_names) > 0:
  proxy.make_safe_operational(zlift_shm_names[0])

omnibase_shm_names=proxy.get_available_components('m3omnibase_shm')
if len(omnibase_shm_names) > 0:
  proxy.make_safe_operational(omnibase_shm_names[0])

humanoid_shm_names=proxy.get_available_components('m3humanoid_shm')
if len(humanoid_shm_names) > 0:
  proxy.make_safe_operational(humanoid_shm_names[0])


#Calibrate ZLift
if zlift is not None:
    if zlift.calibrate(proxy): 
        #Position ZLift
        zlift.set_mode_theta_gc()
        print 'Set resting Z-Lift position (0-700) [300]'
        des=max(0,min(700,m3t.get_float(300)))
        zlift.set_slew_rate_proportion(1.0)
        zlift.set_stiffness(1.0)
        zlift.set_pos_mm(des)
        proxy.step()
        print 'Setting position to',des,'. Hit return when done'
        raw_input()
        zlift.set_mode_off()
        proxy.step()

#Calibrate Base
if omni is not None:
    time.sleep(0.5)
    proxy.step()
    omni.calibrate(proxy)
    time.sleep(0.5)
    
    pwr.set_motor_power_off()
    proxy.step()
    time.sleep(0.5)

proxy.stop()

