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

proxy = m3p.M3RtProxy()
proxy.start()
proxy.make_operational_all()

#Setup Components
zlift_names=proxy.get_available_components('m3joint_zlift')
if len(zlift_names)!=1:
    print 'Invalid number of zlift components available'
    proxy.stop()
    exit()
zl=m3z.M3JointZLift(zlift_names[0])
proxy.subscribe_status(zl)
proxy.publish_command(zl)
proxy.publish_param(zl) 


pwr_rt=m3t.get_joint_pwr_component_name(zlift_names[0])
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


if not zl.calibrate(proxy):
    proxy.stop()
    exit()
    
    
    
##Main loop
zl.set_mode_theta_gc()
des=zl.get_pos_mm()
zl.set_slew_rate_proportion(1.0)
zl.set_stiffness(1.0)
zl.set_pos_mm(des)
proxy.step()
try:
    while True:
        zl.set_pos_mm(des)
        proxy.step()
        print '-----------------------------'
        print 'u: up 2mm'
        print 'd: down 2mm'
        print 'e: enter position'
        print 'q: quit'
        print '-----------------------------'
        print
        k=m3t.get_keystroke()
        if k=='u':
            des=min(700.0,des+2)
        if k=='d':
            des=max(0,des-2)
        if k=='e':
            print 'Enter position (0-700mm)'
            des=max(0,min(700,m3t.get_float()))
        print
        print 'Des (mm)',des
        print 'Pos (mm)',zl.get_pos_mm()
        print
        if k=='q':
            break
except (KeyboardInterrupt):
    pass
zl.set_mode_off()
proxy.step()
proxy.stop()

