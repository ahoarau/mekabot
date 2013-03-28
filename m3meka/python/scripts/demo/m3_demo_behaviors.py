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
import m3.toolbox_beh as m3b

class DemoBehaviors:
    def __init__(self):
        self.ts=time.time()
        self.tl=time.time()
    def always_print(self):
        print 'Always',time.time()-self.ts
        return m3b.res_continue
        
    def every_print(self):
        print 'Every',time.time()-self.ts
        return m3b.res_continue #m3b.res_finished #run once
        
    def random_print(self):
        print 'Random',time.time()-self.ts
        return m3b.res_continue #allow to timeout
    
    def whenever_print(self):
        print 'Whenever',time.time()-self.ts
        return m3b.res_continue #allow to timeout
    
    def whenever_cond(self): #strobe every 3 secs
        ret = time.time()-self.tl>3.0
        if ret:
            self.tl=time.time()
        return ret
    
beh=m3b.M3BehaviorEngine()
db=DemoBehaviors()
beh.define_resource('foo')
beh.define_resource('bar')

beh.always('foo','always_printer',priority=0,action=db.always_print)
beh.every('foo','every_printer',priority=1,action=db.every_print, period=1.0, timeout=0.5, inhibit=2.0)
#beh.whenever('foo','whenever_printer',priority=2,action=db.whenever_print,cond=db.whenever_cond,timeout=0.1)
#beh.random('foo','random_printer',priority=3,action=db.random_print,chance=0.1,timeout=0.25)
for i in range(1000):
    beh.step()
	
	