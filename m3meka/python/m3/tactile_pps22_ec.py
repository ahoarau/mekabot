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
import m3.tactile_pps22_ec_pb2 as mec
from m3.component import M3Component
import Numeric as nu

class M3TactilePPS22Ec(M3Component):
    """EtherCAT interface for a 22 taxel Pressure Profile Systems sensor"""
    def __init__(self,name):
	M3Component.__init__(self,name,type='m3tactile_pps22_ec')
	self.status=mec.M3TactilePPS22EcStatus()
	self.command=mec.M3TactilePPS22EcCommand()
	self.param=mec.M3TactilePPS22EcParam()
	self.taxels=nu.zeros(22,nu.Float32)
	self.read_config()
    def get_taxels(self):
	return self.taxels
    def update_status(self):
	self.taxels=nu.array(self.status.taxels,nu.Float32)
    
