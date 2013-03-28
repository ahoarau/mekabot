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
import m3.gripper as m3h
import m3.toolbox as m3t
import m3.unit_conversion as m3u
import Numeric as nu
import m3.rt_proxy as m3p
import m3.component_factory as m3f

def set_position_mm(a,b,n):
    gripper.set_stiffness([1.0,1.0])
    gripper.set_slew_rate_proportion([1.0,1.0])
    gripper.set_mode_theta_gc()
    gripper.set_pos_mm([a,b])
    for i in range(n):
        proxy.step()
        print 'Force (g)',gripper.get_force_g()
        print 'Position (mm)',gripper.get_pos_mm()
        time.sleep(0.2)
    
def set_force_g(a,b,n):
    gripper.set_mode_torque()
    gripper.set_force_g([a,b])
    proxy.step()
    for i in range(n):
        proxy.step()
        print 'Force (g)',gripper.get_force_g()
        print 'Position (mm)',gripper.get_pos_mm()
        time.sleep(0.2)
    
proxy = m3p.M3RtProxy()
proxy.start()
proxy.make_operational_all()

#Setup Components
chain_names=proxy.get_available_components('m3gripper')
gripper_name=m3t.user_select_components_interactive(chain_names,single=True)
gripper=m3h.M3Gripper(gripper_name[0])
proxy.publish_command(gripper)
proxy.subscribe_status(gripper)
if len(proxy.get_available_components('m3pwr')):
    pwr_name=proxy.get_available_components('m3pwr')[0]
    pwr=m3f.create_component(pwr_name)
    proxy.publish_command(pwr)
    pwr.set_motor_power_on()
      
while True:
    proxy.step()
    print '--------------'
    print 'f: grip'
    print 'o: open'
    print 'd: demo'
    print 'q: quit'
    print '--------------'
    print
    k=m3t.get_keystroke()
    if k=='q':
        break
    if k=='f':
        set_force_g(200,200,5) #Force closure
    if k=='o':
        set_position_mm(0,0,5) #Postion open
    if k=='d':
        set_position_mm(0,0,10) #Postion open
        set_force_g(200,200,8) #Force closure
        set_position_mm(0,0,10) #Postion open
        set_position_mm(40,0,5) #Postion open
        set_force_g(200,200,8) #Force closure
        set_position_mm(0,0,8) #Postion open
        set_position_mm(0,40,8) #Postion open
        set_force_g(200,200,8) #Force closure
        set_force_g(200,-200,8) #Push range
        set_force_g(-200,200,8) #Push range
        set_force_g(200,-200,8) #Push range
        set_position_mm(0,0,20) #Postion open
        gripper.set_mode_off()
proxy.stop()
  
