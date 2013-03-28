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

import m3.rt_proxy as m3p
import m3.component_factory as m3f
import m3.toolbox as m3t

proxy = m3p.M3RtProxy()

proxy.start()

bot_name=m3t.get_robot_name()

bot=m3f.create_component(bot_name)		

slew_rate = [0.8]*7
stiffness = [0.8]*7

bot.initialize(proxy)
proxy.step()

print '------------------------------------------------------------------------------------------------'
print 'This demo switches between a M3 THETA_GC controller and a Shared Memory TORQUE_SHM controller.'
print 'Pose the arm in a start posture now'
print 'Hit enter to continue'
print '------------------------------------------------------------------------------------------------'

raw_input()
proxy.step()

k = 'a'
while k != 'q':
  th = bot.get_theta_deg('right_arm')
  print 'Holding arm at posture',th
  bot.set_theta_deg('right_arm',th)
  bot.set_mode_theta_gc('right_arm',[0])
  bot.set_stiffness('right_arm',stiffness)
  bot.set_slew_rate_proportion('right_arm',slew_rate)
  #proxy.make_safe_operational('torque_shm_0')
  proxy.step()  

  print ""
  print "Starting bot in Mode Theta GC."
  print "After starting TorqueShm Process, "
  print "press any key to swith RIGHT_ARM J0 to mode TorqueShm,"
  print "or Q to quit."
  
  k=m3t.get_keystroke()
  if k == 'q':
    break
  print 'setting to tq_shm'
  bot.set_mode_torque_shm('right_arm',[0])
  proxy.step()
  print 'set tq_shm'
  print ""
  print "Switching RIGHT_ARM J0 to mode TorqueShm,"  
  print "press any key to swith RIGHT_ARM J0 back to mode ThetaGC,"
  print "or Q to quit."
  k=m3t.get_keystroke()
  
bot.set_mode_off('right_arm')
proxy.step()
proxy.stop()