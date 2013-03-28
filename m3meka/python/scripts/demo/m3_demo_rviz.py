#! /usr/bin/python
# -*- coding: utf-8 -*-

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
import m3.toolbox as m3t
import m3.component_factory as m3f
import m3.viz as m3v
import time
	
proxy = m3p.M3RtProxy()
proxy.start()
bot_name=m3t.get_robot_name()
if bot_name == "":
	print 'Error: no robot components found:', bot_name
	exit()
bot=m3f.create_component(bot_name)


#r_hand_ua_name = None
#hand_ua_names = proxy.get_available_components('m3hand_ua')
#if len(hand_ua_names) > 0:
#    r_hand_ua_name = hand_ua_names[0]
#r_hand_ua = m3f.create_component(r_hand_ua_name)
#proxy.subscribe_status(r_hand_ua)

bot.initialize(proxy)
viz = m3v.M3Viz(proxy, bot, 8)

print
print 'Ctrl-C to exit...'
print
try:
	while viz.step():
		proxy.step()
		time.sleep(0.125)
except (KeyboardInterrupt,EOFError):
	pass
print 'Exiting RVIZ'
viz.stop()
proxy.stop()

