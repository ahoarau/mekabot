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
import time
import m3.viz as m3v
from threading import Thread

class viz_thread(Thread):
    def __init__ (self,stride_ms=100):
        Thread.__init__(self)
        self.stride_ms = stride_ms
        self.proxy = m3p.M3RtProxy()
        self.proxy.start(True,True)
        bot_name = m3t.get_robot_name() 
        bot = m3.humanoid.M3Humanoid(bot_name)
        self.proxy.subscribe_status(bot)
        self.viz = m3v.M3Viz(self.proxy,bot)        
        self.done = False                        
        
    def run(self):
	while not self.done:	
            self.proxy.step()        
            self.viz.step()
            time.sleep(self.stride_ms/1000.0)
        self.viz.stop()
        self.proxy.stop()
            
    def stop_viz(self):
        self.done = True
                
t = viz_thread()

t.start()
print '------------------------------'
print 'RVIZ loaded.  Hit <Q> to exit.'

k = 'a'
while not (k == 'q' or k =='Q'):
    k=m3t.get_keystroke()

t.stop_viz()
