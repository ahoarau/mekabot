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
import m3.rt_proxy as m3p
import m3.loadx6 as m3rt
import m3.loadx6_ec as m3ec
import m3.toolbox as m3t
import m3.unit_conversion as m3u
import m3.component_factory as cf
class M3Proc:
	def __init__(self):
		self.proxy = m3p.M3RtProxy()
		self.gui = m3g.M3Gui()
	def stop(self):
		self.proxy.stop()
	def start(self):
		self.proxy.start()
		names=self.proxy.get_available_components('m3loadx6_ec')
		if len(names):
			ec_name=m3t.user_select_components_interactive(names)[0]
			name=ec_name.replace('m3loadx6_ec','m3loadx6')
			self.comp_ec=cf.create_component(ec_name)
			self.proxy.subscribe_status(self.comp_ec)
			self.proxy.make_operational(ec_name)
		else:
			print 'No loadx6_ec component available'
			return
		if self.proxy.is_component_available(name):
			self.comp_rt=cf.create_component(name)
			self.proxy.subscribe_status(self.comp_rt)
			self.proxy.make_operational(name)
		else:
			self.comp_rt=None
		#Create gui
		self.fx=[0]
		self.fy=[0]
		self.fz=[0]
		self.tx=[0]
		self.ty=[0]
		self.tz=[0]
		self.status_dict=self.proxy.get_status_dict()
		self.gui.add('M3GuiTree',   'Status',    (self,'status_dict'),[],[],m3g.M3GuiRead,column=1)
		self.gui.add('M3GuiSliders','Fx (g)',  (self,'fx'),range(1),[-7500,7500],m3g.M3GuiRead)
		self.gui.add('M3GuiSliders','Fy (g)',  (self,'fy'),range(1),[-7500,7500],m3g.M3GuiRead)
		self.gui.add('M3GuiSliders','Fz (g)',  (self,'fz'),range(1),[-7500,7500],m3g.M3GuiRead)
		self.gui.add('M3GuiSliders','Tx (mNm)',  (self,'tx'),range(1),[-7500,7500],m3g.M3GuiRead)
		self.gui.add('M3GuiSliders','Ty (mNm)',  (self,'ty'),range(1),[-7500,7500],m3g.M3GuiRead)
		self.gui.add('M3GuiSliders','Tz (mNm)',  (self,'tz'),range(1),[-7500,7500],m3g.M3GuiRead)
		self.gui.start(self.step)
	def step(self):
		self.proxy.step()
		self.status_dict=self.proxy.get_status_dict()
		print '----------------------------------'
		print 0,self.comp_ec.status.adc_load_0
		print 1,self.comp_ec.status.adc_load_1
		print 2,self.comp_ec.status.adc_load_2
		print 3,self.comp_ec.status.adc_load_3
		print 4,self.comp_ec.status.adc_load_4
		print 5,self.comp_ec.status.adc_load_5
		if self.comp_rt is not None:
			self.fx=[self.comp_rt.get_Fx_Kg()*1000.0]
			self.fy=[self.comp_rt.get_Fy_Kg()*1000.0]
			self.fz=[self.comp_rt.get_Fz_Kg()*1000.0]
			self.tx=[self.comp_rt.get_Tx_mNm()]
			self.ty=[self.comp_rt.get_Ty_mNm()]
			self.tz=[self.comp_rt.get_Tz_mNm()]
			print '----------------------------------'
			print 'Fx (g)',self.fx[0]
			print 'Fy (g)',self.fy[0]
			print 'Fz (g)',self.fz[0]
			print 'Tx (mNm)',self.tx[0]
			print 'Ty (mNm)',self.ty[0]
			print 'Tz (mNm)',self.tz[0]

if __name__ == '__main__':
	t=M3Proc()
	try:
		t.start()
	except (KeyboardInterrupt,EOFError):
		pass
	t.stop()



