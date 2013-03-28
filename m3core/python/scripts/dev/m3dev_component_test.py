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
import m3.rt_proxy as m3p
import m3.toolbox_core as m3t
import m3.component_factory as m3f
import math

class M3Proc:
	def __init__(self):
		self.proxy = m3p.M3RtProxy()		
		self.components = []

	def stop(self):
		self.proxy.stop()
	def start(self):
		self.proxy.start()
		cnames=self.proxy.get_available_components()
                
                print 'Available components:', cnames
                
		self.names=m3t.user_select_components_interactive(cnames)
		
		self.components=[]
		for name in self.names:
			self.components.append(m3f.create_component(name))			
			self.proxy.subscribe_status(self.components[-1])
			self.proxy.publish_command(self.components[-1]) 
			self.proxy.publish_param(self.components[-1]) 
			
                self.proxy.make_operational_all()
		
		self.proxy.step()

	
	def step(self):
		self.proxy.step()

if __name__ == '__main__':
        print "Starting..."
	t=M3Proc()
        t.start()
        i = 0
	try:
		while True:
                    time.sleep(0.05)
                    print 'stepping..', i
                    i += 1
                    t.step()
	except (KeyboardInterrupt,EOFError):
		pass
        print "Stopping..."
	t.stop()
        print "Stopped."


