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


import pygtk
pygtk.require('2.0')
import gtk
import gtk.glade
import sys
import os
import gobject



"""Implements a set of widget meta-classes which dynamically add themself to a canvas. The widget values
are mapped on to a sequence (list/numarray) and the index of each sequence item is provided. The widget is defined by passing
in a list of the form :
	['WidgetType', 'WidgetName',[r/w sequence],[[indices],other_args],readonly]
"""

M3GuiBadReadOnly = "This widget does not support reading/writing"
M3GuiType = 0
M3GuiName = 1
M3GuiValue = 2
M3GuiIndices=3
M3GuiArgs = 4
M3GuiReadOnly = 5
M3GuiRead = True
M3GuiWrite = False


# ######################################### M3GuiToggle #########################################################
M3GuiToggleNames= 0
M3GuiToggleNameActive = 0
M3GuiToggleNameNotActive=1

class M3GuiToggle:
	""" DECL: ['M3GuiToggle', 'toggle_name', [value],[indices],[['name_on','name_off']],readonly]
	Generates write only toggle switch
	Defaults at startup to value.
	For now only allow one toggle, no array, thus ignores indices"""
	def __init__(self):
		self.data=None
		self.first_load=True
		self.view = gtk.VBox()
		self.expand = False
		self.fill = False
		self.idx=0
	def load(self,m): 
		"""Load modes into the widget"""	
		if (self.first_load): 
			self.first_load=False
			self.data=m
			if m[M3GuiReadOnly]==True:
				raise M3GuiBadReadOnly
			hbox = gtk.HBox() #Name on left, mode on right
			label = gtk.Label(m[M3GuiName]+': ')
			hbox.pack_start(label, False, False, 0)
			self.button = gtk.ToggleButton(m[M3GuiArgs][M3GuiToggleNames][int(m[M3GuiValue])])
			self.button.set_active(int(m[M3GuiValue]))
			self.button.connect("toggled", self.toggle_callback, '')
			hbox.pack_start(self.button, False, False, 0)
			self.view.pack_start(hbox, False, False, 0)
	def retrieve(self):
		self.data[M3GuiValue]=self.button.get_active()
		return self.data
	def toggle_callback(self,widget, data):
		if widget.get_active():
			widget.set_label(self.data[M3GuiArgs][M3GuiToggleNames][M3GuiToggleNameActive])
		else:
			widget.set_label(self.data[M3GuiArgs][M3GuiToggleNames][M3GuiToggleNameNotActive])
			
# ######################################### M3GuiModes #########################################################

M3GuiModesNames =0
M3GuiModesColumns =1

class M3GuiModes:
	""" DECL: ['M3GuiModes', 'mode_name', [value],[indices], [['mode0', 'mode1'...],columns],readonly]
	Generates a table of write only mode menus from the value sequence 
	Defaults at startup to value."""
	def __init__(self):
		self.data=None
		self.first_load=True
		self.view = gtk.VBox()
		self.expand = False
		self.fill = False
		self.combos=[]
		self.active_last=[]

	def load(self,m): 
		"""Load modes into the widget"""	
		if (self.first_load): 
			self.first_load=False
			self.data_last=[0]*len(m[M3GuiValue])
			if m[M3GuiReadOnly]==True:
				raise M3GuiBadReadOnly
			self.data=m
			num = len(m[M3GuiIndices])
			num_col = m[M3GuiArgs][M3GuiModesColumns]
			idx=0
			for ridx in range(int(num)/int(num_col) +int(num)%int(num_col)):
				hrow = gtk.HBox()
				nc = min(num_col,num-idx)
				for cidx in range(nc):
					hbox = gtk.HBox() #Name on left, mode on right
					label = gtk.Label(' '+m[M3GuiName]+'%d: '%idx)
					hbox.pack_start(label, True, True, 0)
					combo = gtk.combo_box_new_text()
					self.combos.append(combo)
					for mode_name in m[M3GuiArgs][M3GuiModesNames]: #fill combobox with mode names
						combo.append_text(mode_name)
						combo.set_active(int(m[M3GuiValue][idx]))
						self.data_last[idx]=int(m[M3GuiValue][idx])
					hbox.pack_start(combo, True, True, 0)
					hrow.pack_start(hbox,True, True, 0)
					idx=idx+1
				self.view.pack_start(hrow, True, True, 0)
		#Now set widget to data value if it has been changed by client
		idx=0
		for combo in self.combos:
			c=combo.get_active()
			d=self.data[M3GuiValue][self.data[M3GuiIndices][idx]]
			l=self.data_last[idx]
			if l!=d: #client has set the data, force to this
				combo.set_active(data_curr)
			idx=idx+1
	def retrieve(self):
		idx=0
		for combo in self.combos:
			d=self.data[M3GuiValue][self.data[M3GuiIndices][idx]]
			l=self.data_last[idx]
			self.data[M3GuiValue][self.data[M3GuiIndices][idx]]=combo.get_active()
			d=self.data[M3GuiValue][self.data[M3GuiIndices][idx]]
			l=self.data_last[idx]
			self.data_last[idx]=self.data[M3GuiValue][self.data[M3GuiIndices][idx]]
			idx=idx+1
		return self.data	
	

# ######################################### M3GuiSliders #########################################################

M3GuiSlidersMin =0
M3GuiSlidersMax =1

class M3GuiSliders:
	"""DECL: ['M3GuiSliders', 'slider_name',[value],[indices] [min,max,],readonly]
	Generates read/write slider widgets.Defaults at startup to value. Value is a sequence. Slider for each sequence idx"""
	def __init__(self):
		self.data=None
		self.first_load=True
		self.view = gtk.VBox()
		self.expand = False
		self.fill = False
		self.sliders=[]
	def scale_set_default_values(self,scale):
		scale.set_update_policy(gtk.UPDATE_CONTINUOUS)
		scale.set_digits(1)
		scale.set_value_pos(gtk.POS_TOP)
		scale.set_draw_value(True)		
	def load(self,s): 
		"""Load slider values into the widget"""	
		if (self.first_load): #Setup columns from the dictionary
			self.first_load=False
			self.data=s
			sidx=0
			# value, lower, upper, step_increment, page_increment, page_size
			for idx in s[M3GuiIndices]:
				hbox = gtk.HBox() #Name on left, slider on right
				self.view.pack_start(hbox, True, True, 0)
				range = s[M3GuiArgs][M3GuiSlidersMax]-s[M3GuiArgs][M3GuiSlidersMin]
				adj = gtk.Adjustment(s[M3GuiValue][idx], s[M3GuiArgs][M3GuiSlidersMin], s[M3GuiArgs][M3GuiSlidersMax], range/100.0, range/10.0, 0)
				self.sliders.append(adj)
				hscale = gtk.HScale(adj)
				self.scale_set_default_values(hscale)
				label = gtk.Label(s[M3GuiName]+'%d'%idx+': ')
				hbox.pack_start(label, False, False, 0)
				hbox.pack_start(hscale, True, True, 0)
				hscale.show()
				sidx=sidx+1
		self.update(s)	
	def update(self,s):
		if (self.data[M3GuiReadOnly]==True):
			sidx=0
			for idx in self.data[M3GuiIndices]:
				self.sliders[sidx].set_value(s[M3GuiValue][idx]) #update slider pos
				self.data[M3GuiValue][idx]=s[M3GuiValue][idx] #update interal data
				sidx=sidx+1
	def retrieve(self):
		if (self.data[M3GuiReadOnly]==False):
			sidx=0
			for idx in self.data[M3GuiIndices]:
				self.data[M3GuiValue][idx]=self.sliders[sidx].get_value() 		#read in writeable sliders
				sidx=sidx+1
		return self.data
	
# ######################################### M3GuiTree #########################################################

M3GuiTreeMaxColumns = 12

class M3GuiTreeModel:
	"""Model treats values as strings"""
	def __init__(self):
		self.treestore = gtk.TreeStore(str,str,str, str,str,str, str,str,str, str,str,str, bool,bool,str)  #Cols:  TreeValue0-14,Values Editable, Values Visible,TreeName
		self.num_val_cols=0 #number of value columns used
	def build(self,d,read_only,piter=None): #recursive
		"""Build the TreeModel from the dictionary"""
		key_names = d.keys()
		for key in key_names:
			if type(d[key])==type({}):
				new_piter = self.treestore.append(piter,  ['','','', '','','', '','','', '','','',False,False,key]) #Name Row, default not visible, not editable
				self.build(d[key],read_only,new_piter)
			else:
				try:
					if hasattr(d[key],'__setitem__'): #list or array
						self.num_val_cols = min(max(self.num_val_cols,len(d[key])),M3GuiTreeMaxColumns) #size of the numarray
					else:
						self.num_val_cols=min(max(self.num_val_cols,1),M3GuiTreeMaxColumns)
					new_piter = self.treestore.append(piter, ['','','', '','','', '','','', '','','', False,False,key]) #Name Row, default not visible, not editable
					new_piter = self.treestore.append(new_piter, [ '','','', '','','', '','','', '','','', not read_only,True,'']) #Numarray Row, default visible, editable
				except TypeError: #for non-sequence items
					pass
				
	def update(self,d,piter=None): #recursive
		"""Store dictionary d in the Treemodel"""
		if (piter==None):
			piter = self.treestore.get_iter_first()
		key_names = d.keys()
		for key in key_names:
			if type(d[key])==type({}):
				new_piter = self.treestore.iter_children(piter)
				self.update(d[key],new_piter) #recurse on dictionary
				piter = self.treestore.iter_next(piter)
			else:	
				try:
					new_piter = self.treestore.iter_children(piter)
					v=type(d[key])
					val=d[key]
					if hasattr(val,'__getitem__') and (type(val) is not str) and (type(val) is not unicode): #list or array
						i=0 
						for x in val:
							if i<M3GuiTreeMaxColumns:
								self.treestore.set(new_piter,i,'%0.3f'%x)
							i=i+1
					elif type(val)==str or type(val)==unicode:
						self.treestore.set(new_piter,0,val)
					else: 
						self.treestore.set(new_piter,0,'%0.3f'%val)
					piter = self.treestore.iter_next(piter)		 	    
				except TypeError:
					print 'Type Error: M3GuiTreeModel'
					pass
	
class M3GuiTree:
	"""DECL: ['M3GuiTree','Name',d,[],readonly]
	Generates a tree display of a nested dictionary with sequences.
	It will display up to M3GuiTreeMaxColumns columns of data.
	This widget has some issues. It may crash the client if the size is too large."""
	
	def __init__(self):
		self.data={}
		self.first_load=True
		self.expand = True
		self.fill = True
		self.tree_model = M3GuiTreeModel()
		self.view = gtk.TreeView()
		self.view.set_headers_visible(True)
		self.view.set_model(self.tree_model.treestore)
			
	def load(self,d): 
		"""Load dictionary into the widget"""	
		if (self.first_load): #Setup columns from the dictionary
			cell = gtk.CellRendererText()
			#Create Name TreeView Column
			if(d[M3GuiReadOnly]):
				tvcolumn =  gtk.TreeViewColumn('R: '+d[M3GuiName])
			else:
				tvcolumn =  gtk.TreeViewColumn('W: '+d[M3GuiName])
			self.view.append_column(tvcolumn)
			tvcolumn.pack_start(cell, True)
			tvcolumn.add_attribute(cell, 'text', M3GuiTreeMaxColumns+2) #get vals from column2 of the store
			self.data=d
			self.first_load=False
			self.tree_model.build(d[M3GuiValue],d[M3GuiReadOnly])
			self.tree_model.update(d[M3GuiValue])
		
			for x in range(self.tree_model.num_val_cols): #Build the view columns in the TreeView
				cell = gtk.CellRendererText()
				cell.connect('edited', self.edited_cb, self.tree_model.treestore) #Set up editable callback
				cell.set_data('column', x)  #map the cell to this column for the edit callback
				tvcolumn =  gtk.TreeViewColumn('%d'%x)
				self.view.append_column(tvcolumn)
				tvcolumn.pack_start(cell, True)  
				tvcolumn.add_attribute(cell, 'text', x) #get vals from columnx+3 of the store
				tvcolumn.add_attribute(cell, 'editable', M3GuiTreeMaxColumns) #Pull editable value from col 0 of store
				tvcolumn.add_attribute(cell, 'visible',  M3GuiTreeMaxColumns+1) #Pull visible value from col 1 of store
		else:	
			if d[M3GuiReadOnly]:
				self.data=d
				self.tree_model.update(d[M3GuiValue])
	def retrieve(self):
		"""Retrieve the dictionary from the model"""
		return self.data
	
	def get_sequence(self,d,p):
		"""Find the sequence in dictionary,d, matching the model tuple path, p. Recursive"""
		keys=d.keys()
		val = d[keys[p[0]]]
		p=p[1:]
		if (len(p)==1): #found it
			return val
		else:
			return self.get_sequence(val,p)
	
	def set_item(self,d,p,x,col):
		"""set the item to x in dictionary,d, matching the model tuple path, p. Recursive"""
		keys=d.keys()
		k=keys[p[0]]
		val = d[k]
		p=p[1:]
		if (len(p)==1): #found it
			if hasattr(d[k],'__getitem__'): #list
				d[k][col]=x
			else:
				d[k]=x
		else:
			self.set_item(val,p,x,col)
		
	def edited_cb(self,cell, path, new_text, model):
		treeiter = model.get_iter(path)
		tuple_path = model.get_path(treeiter) #Form (0,1,0)
		column = cell.get_data("column")
		try:
			self.set_item(self.data[M3GuiValue],tuple_path,float(new_text),column)
			model.set(treeiter, column,new_text) 
		except ValueError:
			print 'M3GuiTree only allows setting of numerical values'
			
		

