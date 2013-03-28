#!/usr/bin/env python

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

import pygtk
pygtk.require('2.0')
import gtk
import gtk.gdk
import gtk.glade
import sys
import os
import gobject
import time
import m3.gui as m3g
import m3.rt_proxy as m3p
import m3.toolbox as m3t
import m3.sea_ec_pb2 as mec
import m3.component_factory as m3f
import Numeric as nu
from threading import Thread



		
class M3GripViz: 	
	def __init__(self,stride_ms=100):
		# Greyscale values are taken from values[] for each step()		
		self.values = []   # must be an 8-bit number! (less than 2^8-1, greater than zero)
		self.np = 22
		self.first_update=False
		self.window = gtk.Window()
		self.window.set_title('M3DemoTacile')
		self.window.set_border_width(5)
		self.window.connect('destroy', self.quit)
		self.sw = gtk.ScrolledWindow()
		self.sw.set_shadow_type(gtk.SHADOW_ETCHED_IN)
		self.sw.set_policy(gtk.POLICY_AUTOMATIC,gtk.POLICY_AUTOMATIC)
		self.vbox = gtk.VBox()		
		self.hbox1=gtk.HBox()
		self.hbox2=gtk.HBox()
		self.hbox3=gtk.HBox()
		self.hbox4=gtk.HBox()
		self.hbox5=gtk.HBox()
		self.hbox6=gtk.HBox()		
		self.hbox7=gtk.HBox()
		self.hbox_blank=gtk.HBox()
		self.vbox.pack_start(self.hbox1, True, True, 0)
		self.vbox.pack_start(self.hbox2, True, True, 0)
		self.vbox.pack_start(self.hbox_blank, True, True, 0)
		self.vbox.pack_start(self.hbox3, True, True, 0)
		self.vbox.pack_start(self.hbox4, True, True, 0)
		self.vbox.pack_start(self.hbox5, True, True, 0)
		self.vbox.pack_start(self.hbox6, True, True, 0)
		self.vbox.pack_start(self.hbox7, True, True, 0)				
		self.imgs = []
		self.pixbufs = []
		self.pixbufs.append(gtk.gdk.Pixbuf(gtk.gdk.COLORSPACE_RGB, False, 8, 100, 50))
		self.pixbufs.append(gtk.gdk.Pixbuf(gtk.gdk.COLORSPACE_RGB, False, 8, 100, 50))
		self.pixbufs.append(gtk.gdk.Pixbuf(gtk.gdk.COLORSPACE_RGB, False, 8, 50, 50))
		self.pixbufs.append(gtk.gdk.Pixbuf(gtk.gdk.COLORSPACE_RGB, False, 8, 50, 50))
		self.pixbufs.append(gtk.gdk.Pixbuf(gtk.gdk.COLORSPACE_RGB, False, 8, 50, 50))
		self.pixbufs.append(gtk.gdk.Pixbuf(gtk.gdk.COLORSPACE_RGB, False, 8, 50, 50))
		self.pixbufs.append(gtk.gdk.Pixbuf(gtk.gdk.COLORSPACE_RGB, False, 8, 100, 50))
		for i in range(15):
			self.pixbufs.append(gtk.gdk.Pixbuf(gtk.gdk.COLORSPACE_RGB, False, 8, 50, 50))
		
		for i in range(self.np):
			self.imgs.append(gtk.Image())
			self.pixbufs[i].fill(0)
			self.imgs[i].set_from_pixbuf(self.pixbufs[i])
			self.values.append(0)
			
		
		self.pixbuf_blank = gtk.gdk.Pixbuf(gtk.gdk.COLORSPACE_RGB, True, 8, 50, 15)
		self.pixbuf_blank.fill(0x0)
		self.img_blanks = []
		self.img_blank = gtk.Image()
		self.pixbuf_blanks = []
		
		for i in range(10):
			self.pixbuf_blanks.append(gtk.gdk.Pixbuf(gtk.gdk.COLORSPACE_RGB, True, 8, 120, 25))
			self.pixbuf_blanks[i].fill(0x0)							
			self.img_blanks.append(gtk.Image())
			self.img_blanks[i].set_from_pixbuf(self.pixbuf_blanks[i])
				
		self.table = gtk.Table(5, 3, False)
		self.window.add(self.vbox)
		
		self.update_source_id = gobject.timeout_add(stride_ms, self.update)
		self.first_update=1
		
	# takes in eight bit value and returns 4 channel RGB+alpha (alpha always zero)
	# note 0 is black, FFFF is white
	def grayscale_to_rgb(self, eight_bit_value):
		return (eight_bit_value+eight_bit_value*(2**8)+eight_bit_value*(2**16))*2**8

	def step(self):
		self.proxy.step()
		q=10*(self.pps.get_taxels()-self.zero)+16000 #Normalize
		self.values=q* 255.0/(2**16)
		for i in range(self.np):
			v=int(self.grayscale_to_rgb(min(255,max(0,int(self.values[i])))))
			self.pixbufs[i].fill(v)
			self.imgs[i].set_from_pixbuf(self.pixbufs[i])
			
			
	
	def start(self,process_cb):
		self.proxy = m3p.M3RtProxy()
		self.proxy.start()
		cnames=self.proxy.get_available_components('m3tactile_pps22_ec')
		if len(cnames)==0:
			print 'No PPS22 sensor present'
			return
		if len(cnames)>1:
			name=m3t.user_select_components_interactive(cnames)[0]
		else:
			name=cnames[0]
		self.pps=m3f.create_component(name)
		self.proxy.subscribe_status(self.pps)
		print 'Place sensor in unloaded state.Hit return when ready'
		raw_input()
		self.proxy.step()
		self.zero=self.pps.get_taxels()
		self.process_cb=process_cb
		gtk.main()
		
	def update(self):
		apply(self.process_cb)
		if (self.first_update):
			self.first_update=0 
			self.build()        
			self.window.set_default_size(500, 450)
			self.window.show_all()
		self.step()
		return True

	def build(self):		
		
		self.hbox1.add(self.imgs[0])
		
		self.hbox2.add(self.imgs[1])
		self.hbox2.add(self.imgs[2])
		self.hbox2.add(self.imgs[3])
		self.hbox2.add(self.imgs[4])
		self.hbox2.add(self.imgs[5])
		self.hbox2.add(self.imgs[6])
				
		self.hbox_blank.add(self.img_blank)
		
		self.hbox3.add(self.img_blanks[0])
		self.hbox3.add(self.imgs[7])
		self.hbox3.add(self.imgs[8])
		self.hbox3.add(self.imgs[9])
		self.hbox3.add(self.img_blanks[1])

		self.hbox4.add(self.img_blanks[2])
		self.hbox4.add(self.imgs[10])
		self.hbox4.add(self.imgs[11])
		self.hbox4.add(self.imgs[12])
		self.hbox4.add(self.img_blanks[3])
		
		self.hbox5.add(self.img_blanks[4])
		self.hbox5.add(self.imgs[13])
		self.hbox5.add(self.imgs[14])
		self.hbox5.add(self.imgs[15])
		self.hbox5.add(self.img_blanks[5])
		
		self.hbox6.add(self.img_blanks[6])
		self.hbox6.add(self.imgs[16])
		self.hbox6.add(self.imgs[17])
		self.hbox6.add(self.imgs[18])
		self.hbox6.add(self.img_blanks[7])
		
		self.hbox7.add(self.img_blanks[8])
		self.hbox7.add(self.imgs[19])
		self.hbox7.add(self.imgs[20])
		self.hbox7.add(self.imgs[21])
		self.hbox7.add(self.img_blanks[9])
		
	def quit(self,win):
		self.proxy.stop()
		print 'Closing M3GripViz'
		gtk.main_quit()

def step():
	pass
		
gui = M3GripViz(stride_ms=200)
gui.start(step)