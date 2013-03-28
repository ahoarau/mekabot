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

import yaml
import os 
import m3.toolbox as m3t
import m3.led_matrix_ec_pb2 as mec
from m3.component import M3Component
from PIL import Image
from PIL import ImageColor
import time
import glob

class M3LedMatrixEc(M3Component):
    """EtherCAT interface for mxn led matrix"""
    def __init__(self,name):
	M3Component.__init__(self,name,type='m3led_matrix_ec')
	self.status=mec.M3LedMatrixEcStatus()
	self.command=mec.M3LedMatrixEcCommand()
	self.param=mec.M3LedMatrixEcParam()
	self.read_config()
	self.num_row=self.config['num_rows']
	self.num_col=self.config['num_cols']
	self.ra=0
	if self.config['invert_image']:
	    self.ra=self.num_row-1

	for i in range(self.num_row):
	    self.command.row.add()
	    for j in range(self.num_col):
		self.command.row[i].column.add()
	#Setup animations
	self.seq_id=0
	self.animations={}
	self.anim_name=None

    def set_slew_rate(self,val):
	self.param.slew_rate=val
    def enable_leds(self):
	self.command.enable=True	
	
    def disable_leds(self):
	self.command.enable=False
    
    def read_config(self):
	M3Component.read_config(self)
	
    def set_rgb(self,rr,cc,rgb):
	#Invert rows
	if rr < self.num_row and cc<self.num_col:
	    self.command.row[abs(self.ra-rr)].column[cc].r = rgb[0]
	    self.command.row[abs(self.ra-rr)].column[cc].g = rgb[1]
	    self.command.row[abs(self.ra-rr)].column[cc].b = rgb[2]
    
    def set_image(self,img):
	pix = img.load()
	sz= img.size
	if self.num_row!=sz[1] or self.num_col!=sz[0]:
	    print 'Invalid image size',sz,'. Require',self.num_row,self.num_col
	for rr in range(self.num_row):
	    for cc in range(self.num_col):
		rgb = pix[cc, rr]
		# overdrive protection
		sum = rgb[0] + rgb[1] + rgb[2]
		if sum > 255:
		    r = rgb[0] * (255.0/sum)
		    g = rgb[1] * (255.0/sum)
		    b = rgb[2] * (255.0/sum)
		    rgb = [int(r),int(g),int(b)]
		self.set_rgb(rr,cc,rgb)
		
    def set_all_off(self):
	for rr in range(self.num_row):
	    for cc in range(self.num_col):
		self.set_rgb(rr,cc,[0,0,0])
		
    def get_animations(self):
	return self.animations.keys()
    
    def start_animation(self,anim_name,cycle=False,persist=False):
	"""
	anim_name: in the animation name
	cycle: put animation on loop
	persist: if not in loop, to hold the last frame indefinitely
	"""
	self.seq_id=0
	self.anim_name=anim_name
	self.cycle=cycle
	self.persist=persist
	self.time_start=None
	self.active_seq=self.animations[self.anim_name]['sequence']
	
    def load_animation(self, anim_name):
	"""Call with each animation to load at startup"""
	ani_path = m3t.get_m3_animation_path()
	filename = ani_path + "mouth/" + anim_name + ".yml"
	try:
            f=file(filename,'r')
            self.animations[anim_name]= yaml.safe_load(f.read())
        except (IOError, EOFError):
            print 'Animation file not present:',filename
            return
	for k in self.animations[anim_name]['sequence']:
	    filename = ani_path + "/mouth/" + k['file']	
	    try:
		k['image']= Image.open(filename)
	    except (IOError, EOFError):
		print 'File',filename,'not found'
		
    def convert_color(self,im,rgb):
	im_YCbCr = im.convert("YCbCr")	
	im_split = im_YCbCr.split()
	
	im_color_rgb = Image.new("RGB", im.size, rgb)
	im_color_YCbCr = im_color_rgb.convert("YCbCr")
	im_color_split = im_color_YCbCr.split()
	
	im_new = Image.merge("YCbCr",(im_split[0], im_color_split[1], im_color_split[2]))
	im_new_rgb = im_new.convert("RGB")
	return im_new_rgb
    
    def get_available_images(self):
	return glob.glob(m3t.get_m3_animation_path() + "mouth/*.tiff")
    
    def get_image(self,image_name):
	filename = m3t.get_m3_animation_path() + "mouth/" + image_name + ".tiff"
	try:
	    img=Image.open(filename)
	    return img
	except (IOError, EOFError):
	    print 'File',filename,'not found'
	    return None
	
    def load_command(self):
        """Called before every command msg sent from proxy"""
        self.step_animation()
	
    def step_animation(self):
	if self.anim_name is None:
	    return 
	if self.time_start is None:
	    self.time_start=time.time()
	if time.time()-self.time_start>self.active_seq[self.seq_id]['duration']:
	    self.seq_id=self.seq_id+1
	    self.time_start=time.time()
	if self.seq_id >= len(self.active_seq):
	    if self.cycle:
		self.seq_id=0
	    else:
		self.anim_name=None
		if not self.persist:
		    self.set_all_off()
		return
	self.set_image(self.active_seq[self.seq_id]['image'])
	

    
    
    
