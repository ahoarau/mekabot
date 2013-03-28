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
import m3.unit_conversion as m3u
import Numeric as nu
import m3.pwr

# ######################################################	
  
proxy = m3p.M3RtProxy()
proxy.start()

bot_name=m3t.get_robot_name()
bot=m3f.create_component(bot_name)
proxy.publish_param(bot) 
proxy.subscribe_status(bot)
proxy.publish_command(bot)
proxy.make_operational_all()

proxy.step() #Initialize data
bot.set_motor_power_on()
bot.set_mode_theta('head')
bot.set_theta_deg('head',[0.0]*bot.get_num_dof('head'))
bot.set_slew_rate_proportion('head',[0.5]*bot.get_num_dof('head'))
proxy.step()

print 'Enable motor power. Hit enter when head has achieved zero posture'
raw_input()


bot.set_motor_power_off()
proxy.step()
proxy.stop()
