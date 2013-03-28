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

import yaml
import os 
from m3.toolbox import *
import m3uta.example_pb2 as mrt
from m3.component import M3Component


class M3Example(M3Component):
    """Example component"""
    def __init__(self,name):
        M3Component.__init__(self,name,type='m3example')
        self.status=mrt.M3ExampleStatus()
        self.command=mrt.M3ExampleCommand()
        self.param=mrt.M3ExampleParam()
        self.read_config()
    def set_enable_off(self):
        self.command.enable=False
    def set_enable_on(self):
        self.command.enable=True
    def set_max_fx(self,v):
        self.param.max_fx=v
    def set_max_fy(self,v):
        self.param.max_fy=v
    def set_max_fz(self,v):
        self.param.max_fz=v
    def set_fx(self,v):
        self.command.fx=v
    def set_fy(self,v):
        self.command.fy=v
    def set_fz(self,v):
        self.command.fz=v
    def get_foo( self):
        return self.status.foo
    


