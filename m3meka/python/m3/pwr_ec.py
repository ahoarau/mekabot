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
import m3.pwr_ec_pb2 as mec
from m3.component import M3Component


class M3PwrEc(M3Component):
    """EtherCAT interface for the M3 Power Board"""
    def __init__(self,name):
        M3Component.__init__(self,name,type='m3pwr_ec')
        self.status=mec.M3PwrEcStatus()
        self.command=mec.M3PwrEcCommand()
        self.param=mec.M3PwrEcParam()
        self.read_config()




