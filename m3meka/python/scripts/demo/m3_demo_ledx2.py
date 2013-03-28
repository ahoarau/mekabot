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
import m3.gui as m3g
import m3.toolbox as m3t
import m3.component_factory as m3f
import m3.rt_proxy as m3p
import math

class M3Proc:
    def __init__(self):
        self.proxy = m3p.M3RtProxy()
        self.gui = m3g.M3Gui()
    def stop(self):
        self.proxy.stop()
    def start(self):
        self.ts=time.time()
        self.proxy.start()
        chain_names=self.proxy.get_available_components('m3ledx2_ec')
        led_name=m3t.user_select_components_interactive(chain_names,single=True)[0]
        pwr_name=self.proxy.get_available_components('m3pwr')[0]
        pwr_ec_name=self.proxy.get_available_components('m3pwr_ec')[0]
        self.pwr=m3f.create_component(pwr_name)
        self.proxy.publish_command(self.pwr)
        self.led=m3f.create_component(led_name)
        self.proxy.publish_command(self.led)
        self.proxy.subscribe_status(self.led)
        self.proxy.make_operational(led_name)
        self.proxy.make_operational(pwr_name)
        self.proxy.make_operational(pwr_ec_name)
        self.proxy.subscribe_status(self.pwr)
        self.pwr.set_motor_power_on()
        self.branch_a_board_a_r=[0]
        self.branch_a_board_a_g=[0]
        self.branch_a_board_a_b=[0]
        self.branch_a_board_b_r=[0]
        self.branch_a_board_b_g=[0]
        self.branch_a_board_b_b=[0]
        self.branch_b_board_a_r=[0]
        self.branch_b_board_a_g=[0]
        self.branch_b_board_a_b=[0]
        self.branch_b_board_b_r=[0]
        self.branch_b_board_b_g=[0]
        self.branch_b_board_b_b=[0]
        self.slew=[10000]
        #Create gui
        self.run=False
        self.run_last=False
        self.enable=False
        self.pulse=False
        self.status_dict=self.proxy.get_status_dict()
        self.gui.add('M3GuiTree',   'Status',    (self,'status_dict'),[],[],m3g.M3GuiRead,column=3)
        self.gui.add('M3GuiToggle', 'Enable', (self,'enable'),[],[['On','Off']],m3g.M3GuiWrite,column=1)
        self.gui.add('M3GuiToggle', 'Pulse', (self,'pulse'),[],[['On','Off']],m3g.M3GuiWrite,column=1)
        self.gui.add('M3GuiSliders','Slew', (self,'slew'),range(1),[0,25500],m3g.M3GuiWrite,column=1) 
        self.gui.add('M3GuiSliders','BranchA.BoardA.R', (self,'branch_a_board_a_r'),range(1),[0,1023],m3g.M3GuiWrite,column=1) 
        self.gui.add('M3GuiSliders','BranchA.BoardA.G', (self,'branch_a_board_a_g'),range(1),[0,1023],m3g.M3GuiWrite,column=1) 
        self.gui.add('M3GuiSliders','BranchA.BoardA.B', (self,'branch_a_board_a_b'),range(1),[0,1023],m3g.M3GuiWrite,column=1) 
        self.gui.add('M3GuiSliders','BranchA.BoardB.R', (self,'branch_a_board_b_r'),range(1),[0,1023],m3g.M3GuiWrite,column=1) 
        self.gui.add('M3GuiSliders','BranchA.BoardB.G', (self,'branch_a_board_b_g'),range(1),[0,1023],m3g.M3GuiWrite,column=1) 
        self.gui.add('M3GuiSliders','BranchA.BoardB.B', (self,'branch_a_board_b_b'),range(1),[0,1023],m3g.M3GuiWrite,column=1) 
        self.gui.add('M3GuiSliders','BranchB.BoardA.R', (self,'branch_b_board_a_r'),range(1),[0,1023],m3g.M3GuiWrite,column=1) 
        self.gui.add('M3GuiSliders','BranchB.BoardA.G', (self,'branch_b_board_a_g'),range(1),[0,1023],m3g.M3GuiWrite,column=1) 
        self.gui.add('M3GuiSliders','BranchB.BoardA.B', (self,'branch_b_board_a_b'),range(1),[0,1023],m3g.M3GuiWrite,column=1) 
        self.gui.add('M3GuiSliders','BranchB.BoardB.R', (self,'branch_b_board_b_r'),range(1),[0,1023],m3g.M3GuiWrite,column=1) 
        self.gui.add('M3GuiSliders','BranchB.BoardB.G', (self,'branch_b_board_b_g'),range(1),[0,1023],m3g.M3GuiWrite,column=1) 
        self.gui.add('M3GuiSliders','BranchB.BoardB.B', (self,'branch_b_board_b_b'),range(1),[0,1023],m3g.M3GuiWrite,column=1) 
        self.gui.start(self.step)
    def step(self):
        self.proxy.step()
        self.status_dict=self.proxy.get_status_dict()
	if self.enable:
	    self.led.enable_leds()
	else:
	    self.led.disable_leds()
	rgb=[[self.branch_a_board_a_r[0],
	      self.branch_a_board_a_g[0],
	      self.branch_a_board_a_b[0]],
	     [self.branch_a_board_b_r[0],
	      self.branch_a_board_b_g[0],
	      self.branch_a_board_b_b[0]],
	     [self.branch_b_board_a_r[0],
	      self.branch_b_board_a_g[0],
	      self.branch_b_board_a_b[0]],
	     [self.branch_b_board_b_r[0],
	      self.branch_b_board_b_g[0],
	      self.branch_b_board_b_b[0]]]
	if self.pulse:
	    if math.fmod(time.time()-self.ts,12.0)>6.0:
		rgb=[[1023-self.branch_a_board_a_r[0],
		      1023-self.branch_a_board_a_g[0],
		      1023-self.branch_a_board_a_b[0]],
		     [1023-self.branch_a_board_b_r[0],
		      1023-self.branch_a_board_b_g[0],
		      1023-self.branch_a_board_b_b[0]],
		     [1023-self.branch_b_board_a_r[0],
		      1023-self.branch_b_board_a_g[0],
		      1023-self.branch_b_board_a_b[0]],
		     [1023-self.branch_b_board_b_r[0],
		      1023-self.branch_b_board_b_g[0],
		      1023-self.branch_b_board_b_b[0]]]
		
	self.led.slew_rgb(rgb,self.slew[0]/100.0)
	print '---------- RGB -----------'
	print rgb

		    
if __name__ == '__main__':
    t=M3Proc()
    try:
        t.start()
    except (KeyboardInterrupt,EOFError):
        pass
    t.stop()



