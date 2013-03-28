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


from time import sleep
import m3.toolbox_core as m3t
import m3.widgets as m3w
import pygtk
pygtk.require('2.0')
import gtk
import gtk.glade
import sys
import os
import gobject

M3GuiRead = True
M3GuiWrite = False

M3GuiServerType = 0
M3GuiServerName = 1
M3GuiServerValue=2
M3GuiServerIndices=3
M3GuiServerArgs=4
M3GuiServerReadonly=5
M3GuiServerColumn=6

# ############################################
class M3GuiServer:
	"""
	Widgets are of the form: add(M3GuiSlider,'Slider0',(C,'x'),[arg1, arg2,...],readonly=True,callback), which
	will add a slider which monitors the value C.x for some class C. A specialized callback can be
	provided to write widget values back to C
	The callback should be of form c(src,dest) which copies the src into the dest in-place"""
	def __init__(self):
		self.get=[]
		self.set=[]
		self.base_widgets=[]

	def add(self,type,name,location,indices=[],args=[],readonly=True,column=1):
		"""Internally,widgets are lists of the form: [WidgetType, name,current_value,args, readonly]"""
		w=[type,name,None,indices,args,readonly,column]
		self.add_widget(w,location,column)
		
	def add_widget(self,w,location,column):
		"""We need to handle sequences/individual vars seperately. Toggles are singles"""
		if w[M3GuiServerType]=='M3GuiSliders' or w[M3GuiServerType]=='M3GuiModes':
			self.get.append(lambda : list(getattr(location[0],location[1]))) #list() safeguards against a std::vector 
		elif w[M3GuiServerType]=='M3GuiTree':
			self.get.append(lambda : m3t.DictListCopy(getattr(location[0],location[1]))) #send out the dictionary as a list (safeguards against a std::vector )
		else:
			self.get.append(lambda : getattr(location[0],location[1]))
			
		if w[M3GuiServerReadonly]:
			self.set.append(lambda y:None) #can't set readonly items
		else:
			if w[M3GuiServerType]=='M3GuiTree': #special case
				self.set.append(lambda y: m3t.DictSet(getattr(location[0],location[1]),y)) #read in the dictionary
			elif w[M3GuiServerType]=='M3GuiToggle':
				self.set.append(lambda y: setattr(location[0],location[1],y))
			else:
				self.set.append(lambda y: m3t.SeqSet(getattr(location[0],location[1]),y,w[M3GuiServerIndices])) #make the set_val callback
				
		w[M3GuiServerValue]=self.get[len(self.base_widgets)]() #set the default value in the widget		
		self.base_widgets.append(w)	
	
	def load_val_to_widgets(self):
		idx=0
		for callback in self.get:
			self.base_widgets[idx][M3GuiServerValue]=callback() 
			idx=idx+1
	def load_val_from_widgets(self):
		idx=0
		for callback in self.set:
			callback(self.base_widgets[idx][M3GuiServerValue])
			idx=idx+1
	def step(self):
		self.load_val_from_widgets()
		self.load_val_to_widgets()  #Now send back local values to widgets

# ########################################################################################

class M3Gui(M3GuiServer): 
	"""Builds the GUI client from the link"""
	def __init__(self,stride_ms=125):
		M3GuiServer.__init__(self)
		self.first_update=False
		self.window = gtk.Window()
		self.window.set_title('M3 GUI')
		self.window.set_border_width(5)
		self.window.connect('destroy', self.quit)
		self.sw = gtk.ScrolledWindow()
		self.sw.set_shadow_type(gtk.SHADOW_ETCHED_IN)
		self.sw.set_policy(gtk.POLICY_AUTOMATIC,gtk.POLICY_AUTOMATIC)
		self.vbox1 = gtk.VBox()
		self.vbox2 = gtk.VBox()
		self.vbox3 = gtk.VBox()
		self.hbox=gtk.HBox()
		self.hbox.pack_start(self.vbox1, True, True, 0)
		self.hbox.pack_start(self.vbox2, True, True, 0)
		self.hbox.pack_start(self.vbox3, True, True, 0)
		self.window.add(self.hbox)
		self.widgets=[]
		self.stride_ms = stride_ms		
		self.first_update=1
		
		
	
	def start(self,process_cb):		
		
		self.process_cb=process_cb
		self.update_source_id = gobject.timeout_add(self.stride_ms, self.update)
		gtk.main()
		
	def build(self):
		"""Fill the window in with widgets from first read"""
		self.d=self.base_widgets #get the widget structure from the link
		for item in self.d:
			w=apply(getattr(m3w,item[0]),[]) #generate the class
			w.load(item) #build the classes widgets
			if item[M3GuiServerColumn]==1:
				self.vbox1.pack_start(w.view,w.expand,w.fill) #add widget to the window
			if item[M3GuiServerColumn]==2:
				self.vbox2.pack_start(w.view,w.expand,w.fill) #add widget to the window
			if item[M3GuiServerColumn]==3:
				self.vbox3.pack_start(w.view,w.expand,w.fill) #add widget to the window
			self.widgets.append(w)
		
	def update(self):
		apply(self.process_cb)
		self.step()
		if (self.first_update):
			self.first_update=0 
			self.build()        
			self.window.set_default_size(330, 220)
			self.window.show_all()
		for idx in range(len(self.widgets)):#item in d:
			item=self.base_widgets[idx]
			w=self.widgets[idx]
			w.load(item) #write the new data to the widgets
			self.base_widgets[idx] = w.retrieve() #read the widgets data
		return True
			
	def quit(self,win):
		print 'Closing M3 Gui'
		gtk.main_quit()
		

		
		
		


	

	

    
