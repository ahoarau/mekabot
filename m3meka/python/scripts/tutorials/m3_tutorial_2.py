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

vias = []

print "Hit <SPACE> to record right arm posture, <Q> to save postures and quit"

while True:
    key = m3t.get_keystroke()
    if key == ' ':
        proxy.step()
        t = bot.get_theta_deg('right_arm')
        vias.append(m3t.float_list(t))
        print 'Record of: ', vias[-1]
        
    if key == 'q':
        fn = 'poses.yml'
        f=file(fn, 'w')
        f.write(yaml.safe_dump(vias))
        print 'File saved as: ', fn
        break
    

