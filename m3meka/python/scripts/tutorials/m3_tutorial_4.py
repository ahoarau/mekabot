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

import m3.rt_proxy as m3p
import m3.humanoid
import m3.toolbox as m3t
import yaml

proxy = m3p.M3RtProxy()
proxy.start()

bot_name = m3t.get_robot_name() 
bot = m3.humanoid.M3Humanoid(bot_name)

proxy.subscribe_status(bot)
proxy.publish_command(bot)
proxy.make_operational_all()

fn = 'poses.yml'
f = file(fn, 'r')
vias =  yaml.safe_load(f.read())

print '-------------'
print len(vias), " vias loaded."
print '-------------'
k = 0

bot.set_slew_rate('right_arm',[5]*7)
bot.set_mode_theta_gc('right_arm')
bot.set_motor_power_on()
proxy.step()

while k < len(vias):
    print "Hit <SPACE> to move to via ", k+1, " with positions: ", vias[k], " or <Q> to quit."
    key = m3t.get_keystroke()    
    if key == ' ':
        t = vias[k]        
        bot.set_theta_deg('right_arm', t)
        proxy.step()
        k += 1
    if key == 'q':
        print 'Exiting..'
        break
    
bot.set_mode_off('right_arm')
bot.set_motor_power_off()
proxy.step()
proxy.stop()

    