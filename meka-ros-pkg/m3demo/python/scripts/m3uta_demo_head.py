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
import m3.toolbox_head_s2 as m3h
import m3.component_factory as m3f
import m3.ledx2xn_ec as m3l
import m3.head_s2csp_ctrl as m3csp
import m3.unit_conversion as m3u
import m3.toolbox_ros as m3tr
import Numeric as nu
import m3.pwr
import math
import random

# ######################################################	

class LookBehaviors:
    def __init__(self,bot,csp,beh,use_fd):
        self.joints=range(7)
        self.beh=beh
        self.bot=bot 
        self.csp=csp
	self.use_fd=use_fd
	self.csp.enable()
	self.t_rand=time.time()
	self.origin_def=self.csp.get_origin()
	self.t_slew=[m3t.M3Slew(),m3t.M3Slew(),m3t.M3Slew()]
	if use_fd:
	    self.fd=m3tr.M3FaceDetectThread('right',verbose=False)
	    self.fd.start()
	    self.fd_last=time.time()
	    self.fd_target=None
	    self.fd_rects=None
	    self.tbox=m3h.M3HeadToolboxS2(self.bot.get_chain_component_name('head'))
	
    def stop(self):
	self.csp.set_target_world(self.spherical_to_world(0,0))
	if self.use_fd:
	    self.fd.stop()
	
    def spherical_to_world(self,latitude,longitude):
	#Define target on sphere 
	#lat=0, long=0 corresponds to [1,0,0]
	#lat=90, long=0 corresponds to [0,0,1]
	#lat=0, long=90 corresponds to [0,1,0]
	r=1.0
	x=r*math.sin(m3u.deg2rad(90-latitude))*math.cos(m3u.deg2rad(longitude))
	y=r*math.sin(m3u.deg2rad(90-latitude))*math.sin(m3u.deg2rad(longitude))
	z=r*math.cos(m3u.deg2rad(90-latitude))
	return [x,y,z]
    
    def set_eye_slewrate_proportion(self,val):
	self.csp.set_slew_rate_proportion(4,val)
	self.csp.set_slew_rate_proportion(5,val)
	self.csp.set_slew_rate_proportion(6,val)
	
    def zero(self):
	self.csp.set_origin(self.origin_def)
	target=self.spherical_to_world(0,0)
	self.csp.set_target_world(target)
	self.set_eye_slewrate_proportion(0.35)
        return m3b.res_continue
    
    def roll_backforth(self):
	amp=10.0
	freq=0.1 #Hz
        des=amp*math.sin(time.time()*math.pi*2*freq)
	self.csp.set_theta_j2_deg(des)
	
    def roll_zero(self):
	self.csp.set_theta_j2_deg(0)
    
    def random(self):
	self.csp.set_origin(self.origin_def)
	if time.time()-self.t_rand>6.0:
	    self.t_rand=time.time()
	    self.target_rand=[(2*random.random()-1)*60.0,(2*random.random()-1)*60.0]
	self.csp.set_target_world(self.spherical_to_world(self.target_rand[0],self.target_rand[1]))
	self.set_eye_slewrate_proportion(0.15)
	self.beh.restore_priority('eyelids','sleepy')
        return m3b.res_continue
    
    def leftright(self):
	self.csp.set_origin(self.origin_def)
	self.beh.restore_priority('eyelids','sleepy')
	self.set_eye_slewrate_proportion(1.0)
        self.csp.enable()
	freq=0.1
	des=10.0*math.sin(freq*2*math.pi*time.time())
	target=self.spherical_to_world(10,des)
	self.csp.set_target_world(target)
        return m3b.res_continue

    def face_detected(self):
	#print 'face_detected?'
	#if x>cx, increment x pos, else decr x pos, etc...
	self.fd_rects,dt=self.fd.get_detection()
	if self.fd_rects!=None and dt!=self.fd_last:
	    xi=[self.fd_rects[0].x,self.fd_rects[0].y]
	    self.fd_target=self.tbox.image_to_world('right',xi,r=1.0,bot=self.bot)
	    print 'Pixel',xi
	    #print 'Target',self.fd_target
	    #self.fd_target-self.bot.eye_2_world(eye,xe)
	    #print 'Xw',self.fd_target
	    self.fd_last=dt
	    return True
	return False
    
    def facetrack(self):
	#print 'LookFacetrack'
	#self.beh.set_priority('eyelids','sleepy',0)
	self.csp.set_origin([0.0,0.0,0.0])
	if self.fd_target is not None:
	    t=[self.t_slew[0].step(self.fd_target[0],0.5),
	       self.t_slew[1].step(self.fd_target[1],0.5),
	       self.t_slew[2].step(self.fd_target[2],0.5)]
	    self.set_eye_slewrate_proportion(0.15)
	    self.csp.set_target_world(t)
	    print 'Facetrack',self.fd_target
	    return m3b.res_continue
	return m3b.res_finished
	
    
    

    
class EyelidBehaviors:
    def __init__(self,bot,beh):
        self.joints=[7]
	self.beh=beh
        self.blink_close=True
        self.bot=bot
        self.q_limit=m3h.M3HeadToolboxS2_UTA_EyelidQMax(self.bot.get_chain_component_name('head'))
        self.q_max=self.q_limit.q_max
    def stop(self):
        self.bot.set_theta_deg('head',[35.0],self.joints)

    def limit(self,des):
        q=self.bot.get_theta_deg('head')
        l=self.q_limit.step(q[4],des)
        return l

    def neutral(self):
        self.bot.set_mode_theta_mj('head',self.joints)
        self.bot.set_theta_deg('head',[35.0],self.joints)
        self.bot.set_thetadot_deg('head',[150],self.joints)

	self.beh.restore_priority('look','zero')
	self.beh.restore_priority('ears','sad')
	self.beh.restore_priority('led','blue_circ')

        return m3b.res_continue

    def sleepy(self):
        self.bot.set_mode_theta_mj('head',self.joints)
        self.bot.set_theta_deg('head',[self.limit(self.q_max)],self.joints)
        self.bot.set_thetadot_deg('head',[150.0],self.joints)
	self.beh.set_priority('look','zero',5) #ensure eyes fwd when lids closed
	self.beh.set_priority('ears','sad',5)
	self.beh.set_priority('led','blue_circ',5)
        return m3b.res_continue

    def blink(self): #does one blink
	self.beh.restore_priority('look','zero')
	self.beh.restore_priority('ears','sad')
	self.beh.restore_priority('led','blue_circ')
        if self.blink_close:
            des=self.limit(self.q_max)
        else:
            des=30.0
        q=self.bot.get_theta_deg('head')[self.joints[0]]
        err=abs(des-q)
        if self.blink_close and err<10.0:
            self.blink_close=False
        elif not self.blink_close and err<10.0:
            self.blink_close=True
            return m3b.res_finished
        self.bot.set_mode_theta_mj('head',self.joints)
        self.bot.set_theta_deg('head',[des],self.joints)
        self.bot.set_thetadot_deg('head',[300.0],self.joints)
        return m3b.res_continue

class EarBehaviors:
    def __init__(self,bot,beh):
        self.joints=[8,9,10,11]
        self.bot=bot
        self.beh=beh
        self.postures={'neutral':[-30.0,60.0,-30.0,60.0],
                       'happy':[50.0,0.0,50.0,0.0],
                       'sad':[-50.0,200.0,-50.0,200.0],
                       'wiggle':[[-40.0,10.0],[20.0,70.0]]}#zero, amplitude
        self.thetadot={'neutral':[40.0]*4,
                       'happy':[40.0]*4,
                       'sad':[40.0]*4,
                       'wiggle':[60.0]*4}
        self.wiggle_first=True
        self.wiggle_start=None
    def stop(self):
        self.bot.set_theta_deg('head',[0.0]*4,self.joints)
	self.bot.set_thetadot_deg('head',[90.0]*4,self.joints)
	
    def happy(self):
        self.beh.disable_behavior('ears','wiggle')
        self.bot.set_mode_theta_mj('head',self.joints)
        self.bot.set_theta_deg('head',self.postures['happy'],self.joints)
        self.bot.set_thetadot_deg('head',self.thetadot['happy'],self.joints)
        #print 'happy'
        return m3b.res_continue
    def neutral(self):
        self.beh.enable_behavior('ears','wiggle')
        self.bot.set_mode_theta_mj('head',self.joints)
        self.bot.set_theta_deg('head',self.postures['neutral'],self.joints)
        self.bot.set_thetadot_deg('head',self.thetadot['neutral'],self.joints)
        #print 'neutral'
        return m3b.res_continue
    def sad(self):
        self.beh.disable_behavior('ears','wiggle')
        self.bot.set_mode_theta_mj('head',self.joints)
        self.bot.set_theta_deg('head',self.postures['sad'],self.joints)
        self.bot.set_thetadot_deg('head',self.thetadot['sad'],self.joints)
        #print 'sad'
        return m3b.res_continue
    def wiggle(self):
        if self.wiggle_first:
            self.wiggle_start=time.time()
            self.wiggle_first=False
            #print 'Starting wiggle'
        dt=time.time()-self.wiggle_start
        if dt>3.0:
            self.wiggle_first=True
            return m3b.res_finished
        freq=0.75 #Hz
        des_roll=self.postures['wiggle'][0][0]+self.postures['wiggle'][0][1]*math.sin(dt*math.pi*2*freq)
        des_curl=self.postures['wiggle'][1][0]+self.postures['wiggle'][1][1]*math.sin(dt*math.pi*2*freq)
        #print 'Des roll',des_roll
        #print 'Des curl',des_curl
        #print 'DT',dt
        self.bot.set_mode_theta_mj('head',self.joints)
        self.bot.set_theta_deg('head',[des_roll,des_curl,des_roll,des_curl],self.joints)
        self.bot.set_thetadot_deg('head',self.thetadot['wiggle'],self.joints)
        #print 'wiggle'
        return m3b.res_continue
    
class LedBehaviors:
    def __init__(self,led,bot):
        self.led=led
        self.bot=bot
        self.circ_rate=6.0
        self.circ_slew_rate=3.0
        self.colors={'pink':[500,0,100],
                     'pinkpurp':[400,0,200],
                     'purple':[200,0,200],
                     'off':[0,0,0],
                     'blue':[0,0,500],
                     'green':[0,500,0]}
	self.scale=1.0
	
    def scaled_color(self,c):
	return [self.colors[c][0]*self.scale,self.colors[c][1]*self.scale,self.colors[c][2]*self.scale]
    
    def scaler(self):
	freq=0.1
	self.scale=0.625+0.375*math.sin(math.pi*2*freq*time.time())
	print 'scale',self.scale
    def stop(self):
        self.led.disable_leds()

    def led_cool_circulate(self):
        led.set_mode_circulate('branch_a',self.circ_rate,self.circ_slew_rate,self.scaled_color('green'),self.scaled_color('blue'))
        led.set_mode_circulate('branch_b',self.circ_rate,self.circ_slew_rate,self.scaled_color('green'),self.scaled_color('blue'))
        return m3b.res_continue

    def led_blue_circulate(self):
        led.set_mode_circulate('branch_a',self.circ_rate,self.circ_slew_rate,self.scaled_color('blue'),self.scaled_color('blue'))
        led.set_mode_circulate('branch_b',self.circ_rate,self.circ_slew_rate,self.scaled_color('blue'),self.scaled_color('blue'))
        return m3b.res_continue

    def led_pink_circulate(self):
        led.set_mode_circulate('branch_a',self.circ_rate,self.circ_slew_rate,self.scaled_color('pink'),self.scaled_color('pink'))
        led.set_mode_circulate('branch_b',self.circ_rate,self.circ_slew_rate,self.scaled_color('pink'),self.scaled_color('pink'))
        return m3b.res_continue

    def led_hot_circulate(self):
        led.set_mode_circulate('branch_a',self.circ_rate,self.circ_slew_rate,self.scaled_color('pink'),self.scaled_color('blue'))
        led.set_mode_circulate('branch_b',self.circ_rate,self.circ_slew_rate,self.scaled_color('pink'),self.scaled_color('blue'))
        return m3b.res_continue

    def led_off_circulate(self):
        led.set_mode_circulate('branch_a',self.circ_rate,self.circ_slew_rate,self.colors['off'],self.colors['off'])
        led.set_mode_circulate('branch_b',self.circ_rate,self.circ_slew_rate,self.colors['off'],self.colors['off'])
        return m3b.res_continue

# ######################################################    
proxy = m3p.M3RtProxy()
proxy.start()

bot_name=m3t.get_robot_name()
bot=m3f.create_component(bot_name)
proxy.publish_param(bot) 
proxy.subscribe_status(bot)
proxy.publish_command(bot)
proxy.make_operational_all()
bot.set_motor_power_on()

led_name=proxy.get_available_components('m3ledx2xn_ec')[0]
led=m3l.M3LedX2XNEc(led_name)
proxy.publish_command(led)

csp_name=proxy.get_available_components('m3head_s2csp_ctrl')[0]
csp=m3csp.M3HeadS2CSPCtrl(csp_name)
proxy.publish_command(csp)
proxy.publish_param(csp)

proxy.make_operational_all()
led.enable_leds()

beh=m3b.M3BehaviorEngine(rate=.03)
beh_led=LedBehaviors(led,bot)

print 'Use facetracking (Ros services must be started in advance) [n]?'
use_fd=m3t.get_yes_no('n')
beh_look=LookBehaviors(bot,csp,beh,use_fd)

beh_eyelids=EyelidBehaviors(bot,beh)
beh_ears=EarBehaviors(bot,beh)

beh.define_resource('led')
beh.define_resource('look')
beh.define_resource('look_roll')
beh.define_resource('eyelids')
beh.define_resource('ears')

beh.random('ears','sad',priority=1,action=beh_ears.sad,chance=0.05,timeout=15.0, inhibit=6.0)
beh.random('ears','happy',priority=1,action=beh_ears.happy,chance=0.07,timeout=8.0, inhibit=6.0)
beh.random('ears','wiggle',priority=2,action=beh_ears.wiggle,chance=0.15,timeout=10.0, inhibit=6.0)
beh.always('ears','neutral',priority=0,action=beh_ears.neutral)

beh.random('led','cool_circ',priority=2,action=beh_led.led_hot_circulate,chance=0.02,timeout=15.0, inhibit=3.0)
beh.random('led','hot_circ', priority=2,action=beh_led.led_cool_circulate,chance=0.1,timeout=15.0, inhibit=3.0)
beh.random('led','blue_circ', priority=2,action=beh_led.led_blue_circulate,chance=0.08,timeout=15.0, inhibit=3.0)
beh.random('led','pink_circ', priority=2,action=beh_led.led_pink_circulate,chance=0.1,timeout=20.0, inhibit=3.0)
beh.always('led','pink_circ',priority=0,action=beh_led.led_pink_circulate)
beh.always('step','scaler',priority=0,action=beh_led.scaler)

beh.always('eyelids','neutral',priority=0,action=beh_eyelids.neutral)
beh.random('eyelids','blink', priority=1,action=beh_eyelids.blink,chance=0.01,timeout=1.0, inhibit=4.0)
beh.random('eyelids','sleepy', priority=2,action=beh_eyelids.sleepy,chance=0.01,timeout=10.0, inhibit=25.0)

beh.always('look','zero',priority=0,action=beh_look.zero)
if use_fd:
    beh.whenever('look','facetrack',priority=3,action=beh_look.facetrack,cond=beh_look.face_detected,timeout=4.0,inhibit=0.0)
beh.random('look','leftright',priority=2,action=beh_look.leftright,chance=0.1,timeout=5.0, inhibit=6.0)
beh.random('look','random',priority=1,action=beh_look.random,chance=.1,timeout=4.0, inhibit=6.0)


beh.always('look_roll','roll_zero',priority=0,action=beh_look.roll_zero)
beh.random('look_roll','roll_backforth',priority=1,action=beh_look.roll_backforth,chance=0.07,timeout=5.0, inhibit=5.0)

ts=time.time()
proxy.step() #Initialize data
try:
    while True:
        proxy.step() 
        beh.step(verbose=True)
        #print time.time()-ts
        #print
except (KeyboardInterrupt,EOFError):
    pass
print 'Returning to end posture...'

#Cleanup, assume turn-off posture
beh_ears.stop()
beh_led.stop()
beh_eyelids.stop()
beh_look.stop()
proxy.step()
time.sleep(3.0)
print 'Exiting...'
bot.set_motor_power_off()
proxy.step()
time.sleep(0.25)
proxy.stop()
