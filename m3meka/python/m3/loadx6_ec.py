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
import m3.loadx6_ec_pb2 as mec
from m3.component import M3Component


class M3LoadX6Ec(M3Component):
    """EtherCAT interface for a 6 DOF force-torque sensor"""
    def __init__(self,name):
	M3Component.__init__(self,name,type='m3loadx6_ec')
	self.status=mec.M3LoadX6EcStatus()
	self.command=mec.M3LoadX6EcCommand()
	self.param=mec.M3LoadX6EcParam()
	self.read_config()
	
	
