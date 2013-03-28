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
import m3.toolbox as m3t
import m3.component_factory as m3f
import m3.rt_proxy as m3p
import yaml
import random
import m3.humanoid 
import m3.head as m3h
import Numeric as nu

bot=m3.humanoid.M3Humanoid('m3humanoid_mr0')

# #####################################################################
led_moods={'hot_pink': [[1000,0,0],[1000,0,0],[1000,0,0], [1000,0,0]],
        'green': [[0,300,0],[0,300,0],[0,300,0], [0,300,0]],
        'deep_blue': [[0,0,700],[0,0,700],[0,0,700], [0,0,700]],
        'purple': [[200,0,700],[200,0,700],[200,0,700], [200,0,700]],}

class LedMood():
    def __init__(self):
        pass
    def start(self):
        self.ledid='hot_pink'
    def step(self):
        if random.random()>0.99:
            k=led_moods.keys()
            self.ledid=k[random.randint(0,len(k)-1)]
            print 'Mood',self.ledid
        led.slew_rgb(led_moods[self.ledid],10)
    
# #####################################################################
 
def record_postures():
    #Record posture
    print 'Enter pose name: (q to quit)'
    while True:
        name=m3t.get_string()
	if postures.has_key(name):
	    print 'Pose ',name,'already exists. Overwrite? '
	    if m3t.get_yes_no():
		break
	    else:
		break
	if name!='q' and name !='Q':
	    p=bot.get_theta_deg('head')
	    postures[name]={'theta':list(p),'thetadot':thetadot_default}
	    fn=get_m3_animation_path()+'demo_head_h1_postures.yml'
	    print 'Posture',name,': ',p
	    print 'Writing ',fn
	    f=file(fn,'w')
	    f.write(yaml.safe_dump(postures,width=200))
	    f.close()
	else:
	    print 'Record aborted'

postures={}	
def load_postures():
    fn=m3t.get_m3_animation_path()+'demo_head_h1_postures.yml'
    try:
	f=file(fn,'r')
	postures = yaml.safe_load(f.read())
	f.close()
    except IOError:
	print 'Posture file not present'
    
def calc_error(curr,des):
    return nu.sqrt(sum((nu.array(curr)-nu.array(des))**2))

ep_start=0
def execute_posture(p):
    bot.set_theta_deg('head',p['theta'])
    bot.set_thetadot_deg('head',p['thetadot'])
    e=calc_error(bot.get_theta_deg('head'),p['theta'])/bot.get_num_dof('head')
    print 'E',e
    if e<3.0 or time.time()-ep_start>5.0:
	return True
    return False

	
def display_postures():
    k=postures.keys()
    for i,k in zip(k,range(len(k))):
        print i,': ',k
	
def play_posture():
    k=postures.keys()
    print 'Select posture'
    for idx,kk, in zip(range(len(k)),k):
        print idx,' : ',k
    i=m3t.get_int()
    p=postures[k[i]]
    print 'Display posture: ',k[i]
    done=False
    ep_start=time.time()
    while not done:
        done=execute_posture(postures[k[i]])
    
blink_on=False
blink_ts=0.0
def blinker():
    if random.random()>.9:
        blink_on=True
	blink_ts=time.time()
    if blink_on:
	bot.set_theta_deg('head',[40.0,40.0],[m3h.joints['Right Eyelid'],m3h.joints['Left Eyelid']])
	bot.set_thetadot_deg('head',[80.0,80.0],[m3h.joints['Right Eyelid'],m3h.joints['Left Eyelid']])
	if time.time()-blink_ts>1.0:
	    blink_on=False
	
posture_active=False
pid=None
def random_posture():
    if not posture_active and random.random()>.9:
	posture_active=True
	k=postures.keys()
	pid=k[random.randint(0,len(k)-1)]
	ep_start=time.time()
	print 'Selecting posture',pid
    if posture_active:
	    if execute_posture(postures[k[i]]):
		posture_active=False
	
def animator():
    k=postures.keys()
    print 'Run time [30.0]'
    rt=m3t.get_float(30.0)
    te=time.time()+rt
    while time.time()<te:
	bot.set_theta_deg('head',[0.0]*bot.get_num_dof('head'))
	bliker()
	random_posture()
	ear_lighting()
	proxy.step()
	time.sleep(0.1)
	
class FSA:
    def __init__(self,name,idx):
        self.idx=idx
        self.states=[]
        self.done=True
        self.name=name
    def add_state(self,q,qdot,duration,error,inhibit):
        self.states.append({'theta':q,'thetadot':qdot,'dt':duration,'error':error,'inhibit':inhibit})
    def start(self):
        self.si=-1
        self.done=False
        self.inhibit=False
    def step(self):
        if self.done:
            return self.done
        if self.si==-1:
            self.ts=time.time()
            self.si=0
        if not self.inhibit:
            q=bot.get_theta_deg('head')
            q=[q[i] for i in self.idx]
            e=calc_error(q,self.states[self.si]['theta'])
            #print 'E',e
            timeout = self.states[self.si]['dt'] is not None and time.time()-self.ts>self.states[self.si]['dt']
            errorout= self.states[self.si]['error'] is not None and e<self.states[self.si]['error']
            if  timeout or errorout:
                self.si=self.si+1
            if self.si>=len(self.states):
                self.inhibit=True
                self.ts_inhibit=time.time()
                return self.done
            bot.set_theta_deg('head',self.states[self.si]['theta'],self.idx)
            bot.set_thetadot_deg('head',self.states[self.si]['thetadot'],self.idx)
        if self.inhibit and time.time()-self.ts_inhibit>self.states[self.si-1]['inhibit']:
            self.done=True
            return self.done
        return self.done
    
class Wagger:
    def __init__(self):
        self.idx=[m3h.joints['Right Ear Pitch'],m3h.joints['Right Ear Roll'],
                m3h.joints['Left Ear Pitch'],m3h.joints['Left Ear Roll']]
        self.roll_up_down=FSA('roll_up_down',self.idx)
        self.wag_up_down=FSA('wag_up_down',self.idx)
        self.wag_down=FSA('wag_down',self.idx)
        self.wag_up=FSA('wag_up',self.idx)
        self.neutral=FSA('neutral',self.idx)
        self.roll_up_down.add_state([0.0,45.0,0.0,45.0],[15.0,15.0,15.0,15.0],None,3.0,0.0)
        self.roll_up_down.add_state([0.0,-45.0,0.0,-45.0],[15.0,15.0,15.0,15.0],None,3.0,0.0)
        self.roll_up_down.add_state([0.0,0.0,0.0,0.0],[15.0,15.0,15.0,15.0],None,3.0,2.0)
        self.wag_up_down.add_state([17.0,20.0,17.0,-20.0],[15.0,15.0,15.0,15.0],None,3.0,0.0)
        self.wag_up_down.add_state([-17.0,-20.0,-17.0,20.0],[15.0,15.0,15.0,15.0],None,3.0,0.0)
        self.wag_up_down.add_state([0.0,0.0,0.0,0.0],[15.0,15.0,15.0,15.0],None,3.0,2.0)
        self.wag_down.add_state([-17.0,0.0,-17.0,00.0],[15.0,15.0,15.0,15.0],None,3.0,0.0)
        self.wag_down.add_state([0.0,0.0,0.0,0.0],[15.0,15.0,15.0,15.0],None,3.0,2.0)
        self.wag_up.add_state([17.0,0.0,17.0,00.0],[15.0,15.0,15.0,15.0],None,3.0,0.0)
        self.wag_up.add_state([0.0,0.0,0.0,0.0],[15.0,15.0,15.0,15.0],None,3.0,2.0)
        self.neutral.add_state([0.0,0.0,0.0,0.0],[15.0,15.0,15.0,15.0],None,2.0,1.0)
        self.actions=[ [self.roll_up_down,0.3],
                        [self.neutral,0.5],
                        [self.wag_up_down,0.3],
                        [self.wag_up,0.3],
                        [self.wag_down,0.3]]
        self.pose=None
    def start(self):
        bot.set_mode_theta_mj('head',self.idx)
    def step(self):
        while self.pose is None:
            i=random.randint(0,len(self.actions)-1)
            if random.random()>1.0-self.actions[i][1]:
                self.pose=self.actions[i][0]
                self.pose.start()
                print 'Selected: ',self.pose.name
        if self.pose.step(): 
            self.pose=None
            
class Blinker:
    def __init__(self):
        self.idx=[m3h.joints['Right Eyelid'],m3h.joints['Left Eyelid']]
        self.blink_fast=FSA('blink_fast',self.idx)
        self.blink_slow=FSA('blink_slow',self.idx)
        self.surprise=FSA('surprise',self.idx)
        self.neutral=FSA('neutral',self.idx)
        self.wink=FSA('wink',self.idx)
        self.blink_fast.add_state([30.0,30.0],[150.0,100.0],None,6.0,0.0) #close fast
        self.blink_fast.add_state([0.0,0.0],[150.0,100.0],None,6.0,1.0) #open fast
        self.blink_slow.add_state([30.0,30.0],[30.0,30.0],None,6.0,0.0) #close slow
        self.blink_slow.add_state([30.0,30.0],[30.0,30.0],3.0,None,0.0) #delay slow
        self.blink_slow.add_state([0.0,0.0],[30.0,30.0],None,6.0,1.0) #open slow
        self.neutral.add_state([0.0,0.0],[50.0,50.0],1.0,None,0.0)
        self.surprise.add_state([-8.0,-8.0],[60.0,60.0],None,6.0,0.0) #open fast
        self.surprise.add_state([0.0,0.0],[60.0,60.0],None,6.0,1.0) #close 
        self.wink.add_state([30.0,0.0],[100.0,100.0],None,6.0,0.0) #close fast
        self.wink.add_state([0.0,0.0],[100.0,100.0],None,6.0,1.0) #open fast
        self.actions=[ [self.blink_fast,0.3],
                        [self.neutral,0.5],
                        [self.blink_slow,0.1],
                        [self.surprise,0.1],
                        [self.wink,0.1]]
        self.pose=None
    def start(self):
        bot.set_mode_theta_mj('head',self.idx)
    def step(self):
        while self.pose is None:
            i=random.randint(0,len(self.actions)-1)
            if random.random()>1.0-self.actions[i][1]:
                self.pose=self.actions[i][0]
                self.pose.start()
                print 'Selected: ',self.pose.name
        if self.pose.step(): 
            self.pose=None
  
class Creeper:
    def __init__(self):
        self.idx=[m3h.joints['Lower Neck Pitch'],m3h.joints['Lower Neck Pan']]
        self.neutral=FSA('neutral',self.idx)
        self.stare_left=FSA('stare_left',self.idx)
        self.stare_right=FSA('stare_right',self.idx)
        self.left_right=FSA('left_right',self.idx)
        self.neutral.add_state([-10.0,0.0],[10.0,10.0],3.0,None,0.0) 
        self.stare_left.add_state([-10.0,-40.0],[10.0,10.0],None,10.0,2.0) 
        self.stare_left.add_state([-10.0,0.0],[10.0,10.0],None,10.0,0.0) 
        self.stare_right.add_state([-10.0,-60.0],[10.0,10.0],None,10.0,2.0) 
        self.stare_right.add_state([-10.0,0.0],[10.0,10.0],None,10.0,0.0) 
        self.actions=[  [self.neutral,0.5],
                        [self.stare_left,0.2],
                        [self.stare_right,0.2]]
        self.pose=None
    def start(self):
        bot.set_mode_theta_mj('head',self.idx)
    def step(self):
        while self.pose is None:
            i=random.randint(0,len(self.actions)-1)
            if random.random()>1.0-self.actions[i][1]:
                self.pose=self.actions[i][0]
                self.pose.start()
                print 'Selected: ',self.pose.name
        if self.pose.step(): 
            self.pose=None
                          
class Tilter:
    def __init__(self):
        self.idx=[m3h.joints['Upper Neck Pitch'],m3h.joints['Upper Neck Roll']]
        self.neutral=FSA('neutral',self.idx)
        self.tilt_left=FSA('tilt_left',self.idx)
        self.tilt_right=FSA('tilt_right',self.idx)
        self.tilt_up_down=FSA('tilt_up_down',self.idx)
        self.neutral.add_state([0.1,0.1],[7,7],None,5.0,3.0) 
        self.tilt_left.add_state([-10,-20.0],[4.5,4.5],None,5.0,2.0) 
        self.tilt_left.add_state([0.0,0.0],[4.5,4.5],None,5.0,0.0) 
        self.tilt_right.add_state([-10.0,20.0],[4.5,4.5],None,5.0,2.0) 
        self.tilt_right.add_state([0.0,0.0],[4.5,4.5],None,5.0,0.0) 
        self.tilt_up_down.add_state([15.0,5.0],[4.5,4.5],None,5.0,0.0) 
        self.tilt_up_down.add_state([-25.0,-5.0],[4.5,4.5],None,5.0,2.0) 
        self.tilt_up_down.add_state([0.0,0.0],[4.5,4.5],5.0,5.0,0.0) 
        self.actions=[  [self.neutral,0.5],
                        [self.tilt_up_down,0.2],
                        [self.tilt_left,0.1],
                        [self.tilt_right,0.1]]
        self.pose=None
    def start(self):
        bot.set_mode_theta_mj('head',self.idx)
    def step(self):
        while self.pose is None:
            i=random.randint(0,len(self.actions)-1)
            if random.random()>1.0-self.actions[i][1]:
                self.pose=self.actions[i][0]
                self.pose.start()
                print 'Selected: ',self.pose.name
        if self.pose.step(): 
            self.pose=None
            
class Looker:
    def __init__(self):
        self.idx=[m3h.joints['Eye Tilt'],m3h.joints['Right Eye Pan'],m3h.joints['Left Eye Pan']]
    def start(self):
        self.pose_active=False
        self.delay_active=False
        bot.set_mode_theta_mj('head',self.idx)
    def step(self):
        if not self.pose_active and not self.delay_active:
            pan=2.0*(random.random()-0.5)*35.0
            tilt=2.0*(random.random()-0.5)*28.0
            self.vscale=0.5+random.random()*0.5
            self.des=[tilt,pan,pan]
            self.ts=time.time()
            self.pose_active=True
            if random.random()>0.5:
                self.delay=random.random()*1.0
            else:
                self.delay=0.0
            self.delay_active=False
        bot.set_theta_deg('head',self.des,self.idx)
        bot.set_thetadot_deg('head',[self.vscale*50.0,self.vscale*50.0,self.vscale*50.0],self.idx)
        q=bot.get_theta_deg('head')
        q=[q[i] for i in self.idx]
        e=calc_error(q,self.des)
        if (e<5.0 or time.time()-self.ts>7.0) and not self.delay_active:
            self.pose_active=False
            self.delay_active=True
            self.ts=time.time()
        if self.delay_active and time.time()-self.ts>self.delay:
            self.delay_active=False
 

    
###Look left to right
##look_lr_fast=fsa([m3h.joints['Lower Neck Pitch'],m3h.joints['Lower Neck Pan']])
##look_lr_fast.add_state(q=[-70,0],qdot=[20,20],duration=3.0)
##look_lr_fast.add_state(q=[70,0],qdot=[20,20],duration=3.0)
##look_lr_fast.add_state(q=[0,0],qdot=[20,20],duration=3.0)
##
###Look left to right
##look_lr_slow=fsa([m3h.joints['Lower Neck Pitch'],m3h.joints['Lower Neck Pan']])
##look_lr_slow.add_state(q=[-70,0],qdot=[20,20],duration=3.0)
##look_lr_slow.add_state(q=[70,0],qdot=[20,20],duration=3.0)
##look_lr_slow.add_state(q=[0,0],qdot=[20,20],duration=3.0)

    

# #####################################################################

proxy = m3p.M3RtProxy()
proxy.start()
bot_name=m3t.get_robot_name()
bot=m3f.create_component(bot_name)
proxy.subscribe_status(bot)
proxy.publish_command(bot)
led_name=proxy.get_available_components('m3ledx2_ec')
led=m3f.create_component(led_name[0])
led.enable_leds()
proxy.publish_command(led)
proxy.make_operational_all()
bot.set_motor_power_on()
#load_postures()
look=Looker()
#look.start()
blink=Blinker()
#blink.start()
creep=Creeper()
creep.start()
#wag=Wagger()
#wag.start()
#mood=LedMood()
#mood.start()
#tilt=Tilter()
#tilt.start()

try:
    while (True):
        proxy.step()
        #look.step()
        #blink.step()
        creep.step()
        #wag.step()
        #mood.step()
        #tilt.step()
        time.sleep(0.1)
except (KeyboardInterrupt,EOFError):
    proxy.stop()







