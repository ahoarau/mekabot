#M3 -- Meka Robotics Robot Components
#Copyright (c) 2010 Meka Robotics
#Author: edsinger@mekabot.com (Aaron Edsinger)

#M3 is free software: you can redistribute it and/or modify
#it under the terms of the GNU Lesser General Public License as published by
#the Free Software Foundation, either version 3 of the License, or
#(at your option) any later version.

#M3 is distributed in the hope that it will be useful,
#but WITHOUT ANY WARRANTY; without even the implied warranty of
#MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#GNU Lesser General Public License for more details.

#You should have received a copy of the GNU Lesser General Public License
#along with M3.  If not, see <http://www.gnu.org/licenses/>.

import math
import time
import numpy as nu
import m3.unit_conversion as m3u
import random


res_continue=0
res_finished=1

class M3BehaviorEngine:
    def __init__(self,rate=.015): #50hz
        self.behaviors={}
        self.resources={}
        self.resources['step']={'inactive':{},'active':{},'current':[]}
        if rate<.015:
            print 'Warning: the Python timer system cannot support behavior rates below 0.015s. Proceeding with rate 0.015'
            rate=0.015
        self.rate=rate
        self.time_last=time.time()
    def step(self,verbose=False):
        #find behaviors ready to execute
        b_ready={}
        for r in self.resources.keys():
            if r=='step':
                continue
            b_ready[r]=[]
            for name in self.resources[r]['active']:
                #Always. Ready to run every cycle
                if self.behaviors[name]['type']=='always':
                    b_ready[r].append(name)
                    
                #Every: Ready to run every N seconds.
                if self.behaviors[name]['type']=='every':
                    if time.time()>self.behaviors[name]['time_trigger'] and self.behaviors[name]['trigger']==0 \
                       and time.time()>self.behaviors[name]['time_deinhibit']:
                        self.behaviors[name]['trigger']=1
                        self.behaviors[name]['time_trigger']+=self.behaviors[name]['period']
                        self.behaviors[name]['time_detrigger']=time.time()+self.behaviors[name]['timeout']
                        
                #Whenever: Ready to run whenever condition is met. 
                if self.behaviors[name]['type']=='whenever':
                    if apply(self.behaviors[name]['cond']) and time.time()>self.behaviors[name]['time_deinhibit']:
                        self.behaviors[name]['trigger']=1
                        self.behaviors[name]['time_trigger']=time.time()
                        self.behaviors[name]['time_detrigger']=time.time()+self.behaviors[name]['timeout']

                #Random: Ready to run randomly. 
                if self.behaviors[name]['type']=='random':
                    if random.random()<self.behaviors[name]['chance'] and not self.behaviors[name]['trigger'] \
                       and time.time()>self.behaviors[name]['time_deinhibit']:
                        self.behaviors[name]['trigger']=1
                        self.behaviors[name]['time_trigger']=time.time()
                        self.behaviors[name]['time_detrigger']=time.time()+self.behaviors[name]['timeout']
                        
                #Every/Whenever/Random: handle timeouts. After timeout seconds, de-trigger self
                if self.behaviors[name]['type']=='whenever' or self.behaviors[name]['type']=='every'\
                   or self.behaviors[name]['type']=='random':
                    if  self.behaviors[name]['trigger']:
                        if time.time()>self.behaviors[name]['time_detrigger']:
                            self.behaviors[name]['trigger']=0
                            self.behaviors[name]['time_deinhibit']=time.time()+self.behaviors[name]['inhibit']
                            if self.behaviors[name]['type']=='every':
                                while self.behaviors[name]['time_trigger']<self.behaviors[name]['time_deinhibit']: #keep synchronous
                                    self.behaviors[name]['time_trigger']+=self.behaviors[name]['period']
                    #Ready to run if triggered
                    if self.behaviors[name]['trigger']:
                        b_ready[r].append(name)
                
        
        #For each reasource, find highest priority ready behavior and run                
        for r in self.resources.keys():
            if r=='step':
                continue
            name_max=None
            priority_max=-1
            if sum([x==self.resources[r]['current'] for x in b_ready[r]]): #last behavior still valid, default to this
                name_max=self.resources[r]['current']
                priority_max=self.behaviors[name_max]['priority']
            name_curr = name_max
            for name in b_ready[r]:
                if self.behaviors[name]['priority']>priority_max:
                    priority_max=self.behaviors[name]['priority']
                    name_max=name
            if name_max is not None and name_max!=name_curr and verbose:
                        print 'Switching to behavior: ',name_max
            if name_max is not None:
                result=apply(self.behaviors[name_max]['action'])
                self.resources[r]['current']=name_max
                #Always: do nothing with results
                #Every/Whenever/Random: de-trigger self based on result
                if self.behaviors[name_max]['type']=='every' or self.behaviors[name_max]['type']=='whenever'\
                   or self.behaviors[name_max]['type']=='random':
                    if result==res_finished:
                        self.behaviors[name_max]['trigger']=0
                        self.behaviors[name_max]['time_deinhibit']=time.time()+self.behaviors[name_max]['inhibit']
        
        #Run all 'step' behaviors every step no matter what
        
        for b in self.resources['step']['active'].keys():
            result=apply(self.behaviors[b]['action'])
            
        #Sleep to sustain correct rate
        time.sleep(max(.01,self.rate-(time.time()-self.time_last)))
        #print 'dt',(time.time()-self.time_last)
        self.time_last=time.time()
        
    def define_resource(self,name):
        if name=='step':
            print 'Unavailable resource name: step'
            return
        if not self.resources.has_key(name):
            self.resources[name]={'inactive':{},'active':{},'current':None}
            
    def enable_behavior(self,resource,name):
        n=resource+'_'+name
        a=self.resources[resource]['active'].has_key(n)
        i=self.resources[resource]['inactive'].has_key(n)
        if i and not a: #move from inactive to active
            self.resources[resource]['active'][n]=None
            self.resources[resource]['inactive'].pop(n)
    
    def disable_behavior(self,resource,name):
        if name=='step':
            print 'Unavailable resource name: step'
            return
        n=resource+'_'+name
        a=self.resources[resource]['active'].has_key(n)
        i=self.resources[resource]['inactive'].has_key(n)
        if a and not i: #move from active to inactive
            self.resources[resource]['inactive'][n]=None
            self.resources[resource]['active'].pop(n)
    
    def set_priority(self,resource,name,val):
        try:
            n=resource+'_'+name
            p=self.behaviors[n]['priority']
            self.behaviors[n]['priority']=val
            return p
        except KeyError:
            return 0

        
    
    def restore_priority(self,resource,name):
        try:
            n=resource+'_'+name
            self.behaviors[n]['priority']=self.behaviors[n]['priority_orig']
        except KeyError:
            pass
        
    def always(self,resource,name,priority,action):
        n=resource+'_'+name
        beh={'action':action,
             'priority':priority,
             'priority_orig':priority,
             'type':'always',
             'resource':resource,
             'inhibit':0.0}
        self.behaviors[n]=beh
        self.resources[resource]['active'][n]=None
        
    def every(self,resource,name,priority,action,period,timeout=1.0,inhibit=0.0):
        if name=='step':
            print 'Unavailable resource name: step'
            return
        n=resource+'_'+name
        beh={'action':action,
             'priority':priority,
             'priority_orig':priority,
             'type':'every',
             'resource':resource,
             'period':period,
             'time_trigger':time.time()+period,
             'time_detrigger':time.time()+period+timeout,
             'time_deinhibit':time.time(),
             'timeout':timeout,
             'trigger':0,
             'inhibit':inhibit}
        self.behaviors[n]=beh
        self.resources[resource]['active'][n]=None
    
    def whenever(self,resource,name,priority,action,cond,timeout=1.0,inhibit=0.0):
        if name=='step':
            print 'Unavailable resource name: step'
            return
        n=resource+'_'+name
        beh={'action':action,
             'priority':priority,
             'priority_orig':priority,
             'type':'whenever',
             'resource':resource,
             'cond':cond,
             'time_trigger':time.time(),
             'time_detrigger':time.time()+timeout,
             'time_deinhibit':time.time(),
             'timeout':timeout,
             'inhibit':inhibit,
             'trigger':0}
        self.behaviors[n]=beh
        self.resources[resource]['active'][n]=None
    
    def random(self,resource,name,priority,action,chance,timeout=1.0,inhibit=0.0):
        if name=='step':
            print 'Unavailable resource name: step'
            return
        n=resource+'_'+name
        beh={'action':action,
             'priority':priority,
             'priority_orig':priority,
             'type':'random',
             'resource':resource,
             'chance':chance,
             'time_trigger':time.time(),
             'time_detrigger':time.time()+timeout,
             'time_deinhibit':time.time(),
             'timeout':timeout,
             'inhibit':inhibit,
             'trigger':0}
        self.behaviors[n]=beh
        self.resources[resource]['active'][n]=None
    
    def until(self,resource,name,priority,action,condition,timeout):
        pass
    

