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
from m3.toolbox import *
import m3.loadx6_pb2 as mrt
#import Numeric as nu

from m3.component import M3Component
from m3.unit_conversion import *


class M3LoadX6(M3Component):
    """Calibrated interface for a 6 DOF force-torque sensor"""
    def __init__(self,name):
	M3Component.__init__(self,name,type='m3loadx6')
	self.status=mrt.M3LoadX6Status()
	self.command=mrt.M3LoadX6Command()
	self.param=mrt.M3LoadX6Param()
	self.wrench=nu.zeros(6,float)
	self.read_config()	
    #Utility API
    def get_dig_ext0(self):
	"""Digital input"""
	return self.status.dig_ext_0
    def get_adc_ext0(self):
	"""12bit ADC input"""
	return self.status.adc_ext_0
    def get_adc_ext1(self):
	"""12bit ADC input"""
	return self.status.adc_ext_1
    def get_adc_ext2(self):
	"""12bit ADC input"""
	return self.status.adc_ext_2
    def get_wrench(self):
	"""6 DOF wrench (mNm/mN)"""
	return self.wrench
    
    def get_Fx_mN(self):
	return self.wrench[0]
    def get_Fy_mN(self):
	return self.wrench[1]
    def get_Fz_mN(self):
	return self.wrench[2]
    
    def get_Fx_Lb(self):
	return mN2lb(self.wrench[0])
    def get_Fy_Lb(self):
	return mN2lb(self.wrench[1])
    def get_Fz_Lb(self):
	return mN2lb(self.wrench[2])
    
    def get_Fx_Kg(self):
	return mN2Kg(self.wrench[0])
    def get_Fy_Kg(self):
	return mN2Kg(self.wrench[1])
    def get_Fz_Kg(self):
	return mN2Kg(self.wrench[2])
    
    def get_Tx_mNm(self):
	return self.wrench[3]
    def get_Ty_mNm(self):
	return self.wrench[4]
    def get_Tz_mNm(self):
	return self.wrench[5]
    
    def get_Tx_inLb(self):
	return mNm2inLb(self.wrench[3])
    def get_Ty_inLb(self):
	return mNm2inLb(self.wrench[4])
    def get_Tz_inLb(self):
	return mNm2inLb(self.wrench[5])
    
    def update_status(self):
	self.wrench=nu.array(self.status.wrench,float)
  
