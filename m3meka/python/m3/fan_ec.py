#M3 -- Meka Robotics Robot Components
#Copyright (C) 2008 Meka Robotics
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

import m3.sea_ec_pb2 as mec
from m3.sea_ec import M3SeaEc
import yaml

class M3FanEc(M3SeaEc):
	"""EtherCAT interface for DC Fan driven by SeaEc controller"""
	def __init__(self,name):
		M3SeaEc.__init__(self,name,type='m3fan_ec')
	def set_speed(self,val):
		val=min(1.0,max(-1.0,float(self.param.k_p)*val))*self.param.pwm_max
		if (val==0.0):
			self.command.mode=mec.M3SEA_EC_MODE_OFF
		else:
			self.command.t_desire=int(val)
			self.command.mode=mec.M3SEA_EC_MODE_PWM
		
	
