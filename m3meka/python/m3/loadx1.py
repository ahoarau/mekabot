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
import m3.loadx1_pb2
from m3.component import M3Component

class M3LoadX1(M3Component):
    """Calibrated interface for a load cell"""
    def __init__(self,name,type='m3loadx1'):
        M3Component.__init__(self,name,type=type)
        self.status=m3.loadx1_pb2.M3LoadX1Status()
        self.command=m3.loadx1_pb2.M3LoadX1Command()
        self.param=m3.loadx1_pb2.M3LoadX1Param()
        self.read_config()

    #Utility API
    def get_torque_mNm(self): 
        return self.status.torque
    def get_torquedot_mNm(self):
        return self.status.torquedot
    


