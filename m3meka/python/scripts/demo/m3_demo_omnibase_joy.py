#! /usr/bin/python

#Copyright  2010, Meka Robotics
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
import numpy as nu
from m3.unit_conversion import *
import m3.component_factory as m3f
import m3.rt_proxy as m3p
import m3.toolbox as m3t
import m3.omnibase as m3o
import m3.toolbox_ros as m3tr
import m3.joint_zlift as m3z
import m3.toolbox_omnibase_b1 as m3to
import roslib; roslib.load_manifest('m3meka_msgs')
import rospy
from m3meka_msgs.msg import M3OmnibaseJoy

def main():   
    
    proxy = m3p.M3RtProxy()
    proxy.start()    
    base_name=proxy.get_available_components('m3omnibase')
    omni=None
    if len(base_name)!=1:
            print 'Omnibase not available. Proceeding without it...'
    else:
        print 'Use OmniBase [y]?'
        if m3t.get_yes_no('y'):
            omni=m3o.M3OmniBase(base_name[0])
            proxy.publish_param(omni) # we need this for calibration
            proxy.subscribe_status(omni)
            proxy.publish_command(omni)
    
    pwr_name=[m3t.get_omnibase_pwr_component_name(base_name[0])]
    #proxy.get_available_components('m3pwr')
    #if len(pwr_name)>1:
            #pwr_name=m3t.user_select_components_interactive(pwr_name,single=True)
    pwr=m3f.create_component(pwr_name[0])
    
    proxy.subscribe_status(pwr)
    proxy.publish_command(pwr) 

    zlift_names=proxy.get_available_components('m3joint_zlift')
    zl=None
    if len(zlift_names)==1:    
	print 'Use Zlift [y]?'
	if m3t.get_yes_no('y'):
		zl=m3z.M3JointZLift(zlift_names[0])
		proxy.subscribe_status(zl)
		proxy.publish_command(zl)
		proxy.publish_param(zl) 

    proxy.make_operational(pwr_name[0])
    proxy.step()
    if omni is not None:
        omni.set_mode_off()
    pwr.set_motor_power_on()    
    proxy.make_operational_all()
    
    zlift_shm_names=proxy.get_available_components('m3joint_zlift_shm')
    if len(zlift_shm_names) > 0:
      proxy.make_safe_operational(zlift_shm_names[0])

    omnibase_shm_names=proxy.get_available_components('m3omnibase_shm')
    if len(omnibase_shm_names) > 0:
      proxy.make_safe_operational(omnibase_shm_names[0])

    humanoid_shm_names=proxy.get_available_components('m3humanoid_shm')
    if len(humanoid_shm_names) > 0:
      proxy.make_safe_operational(humanoid_shm_names[0])
    
    proxy.step()
    time.sleep(0.5)
    if omni is not None:
        proxy.step()
        omni.calibrate(proxy)
        time.sleep(0.5)
    
    if zl is not None:
        if not zl.calibrate(proxy):
            zl=None
            print 'ZLift failed to calibrate'
    
    if omni is None and zl is None:
        exit()
        
    print "Turn motor power on to Omnibase and press any key."
    raw_input()      
    
    joy=m3to.M3OmniBaseJoy()
    joy.start(proxy,omni,zl)
    k = 0
    try:
        while True:
            joy.step()
            #print 'Bus Current:', pwr.get_bus_torque()
            p = omni.get_local_position()
            #omni.set_op_space_forces(f.jx*200.0, f.jy*200.0, f.jyaw*50.0)
            k += 1
            if k == 100:
	      print '-----------Local Pos-------'
	      print 'X:', p[0]
	      print 'Y:', p[1]
	      print 'Yaw:', p[2]
	      #print '---------------------------'
	      k = 0
            '''print '-------Joystick Pos-------'
            print 'jx:',f.jx
            print 'jy:', f.jy
            print 'jyaw:', f.jyaw
            print 'button:', f.jbutton
            print '---------------------------'           
            #print 'Bus voltage',omni.get_bus_voltage()'''
            '''tqs = omni.get_motor_torques()
            print tqs[0], tqs[2], tqs[4], tqs[6]'''
            #print omni.get_steer_torques()
            proxy.step()
    except KeyboardInterrupt:
        pass
    
    joy.stop()
    if omni is not None:
        omni.set_mode_off()
    pwr.set_motor_power_off()
    
    proxy.step()
    proxy.stop()

# allow use as a module or standalone script  
if __name__ == "__main__":  
    main()  

