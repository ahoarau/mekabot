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
import m3.rt_proxy as m3p
import m3.toolbox as m3t
import m3.component_factory as m3f
import Numeric as nu
#import m3uta.ledx2xn_ears as m3l
import m3.ledx2xn_ec as m3l
import m3.pwr

# ######################################################	
proxy = m3p.M3RtProxy()
proxy.start()


led_name=proxy.get_available_components('m3ledx2xn_ec')[0]
led=m3l.M3LedX2XNEc(led_name)
proxy.publish_command(led)

pwr_name=proxy.get_available_components('m3pwr')[0]
pwr_ec_name=pwr_name.replace('m3pwr_','m3pwr_ec_')
proxy.make_operational(pwr_name)
proxy.make_operational(pwr_ec_name)
proxy.make_operational(led_name)
pwr=m3.pwr.M3Pwr(pwr_name)
proxy.publish_command(pwr)
pwr.set_motor_power_on()



rgb_blue=[[0,0,500]]*12
rgb_purp=[[200,0,200]]*12
rgb_off=[[0,0,0]]*12
rgb_green=[[0,500,0]]*12

proxy.step()

led.enable_leds()
proxy.step()
print 'Test direct mode [n]?'
if m3t.get_yes_no('n'):
    for i in range(10):
        led.set_mode_direct('branch_a',rgb_blue)
        led.set_mode_direct('branch_b',rgb_blue)
        print 'blue'
        proxy.step()
        time.sleep(0.5)
        led.set_mode_direct('branch_a',rgb_purp)
        led.set_mode_direct('branch_b',rgb_purp)
        print 'purple'
        proxy.step()
        time.sleep(0.5)
    
print 'Test slew mode [n]?'
if m3t.get_yes_no('n'):
    for i in range(5):
        for i in range(100):
            led.set_mode_slew('branch_a',5.0,rgb_blue)
            led.set_mode_slew('branch_b',5.0,rgb_blue)
            proxy.step()
            time.sleep(0.01)
        for i in range(100):
            led.set_mode_slew('branch_a',5.0,rgb_green)
            led.set_mode_slew('branch_b',5.0,rgb_green)
            proxy.step()
            time.sleep(0.01)
        for i in range(100):
            led.set_mode_direct('branch_a',rgb_purp)
            led.set_mode_direct('branch_b',rgb_purp)
            proxy.step()
            time.sleep(0.01)

print 'Test circulate mode [y]?'
if m3t.get_yes_no('y'):
    
    t=time.time()
    print 'Enter run time [10.0]'
    rt=m3t.get_float(10.0)
    while time.time()-t<rt:
        led.set_mode_circulate('branch_a',6.0,3.0,rgb_blue[0],rgb_green[0])
        led.set_mode_circulate('branch_b',6.0,3.0,rgb_purp[0],rgb_green[0])
        print time.time()-t ,'/',rt
        proxy.step()
        time.sleep(0.1)


pwr.set_motor_power_off()
led.disable_leds()
proxy.step()
proxy.stop()
