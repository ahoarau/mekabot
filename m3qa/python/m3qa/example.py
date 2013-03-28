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


import m3xyz.example_pb2 as mx
from m3.component import M3Component

class M3XYZExample(M3Component):
    """Example component"""
    def __init__(self,name):
        M3Component.__init__(self,name,type='m3xyz_example')
        self.status=mx.M3XYZExampleStatus()
        self.command=mx.M3XYZExampleCommand()
        self.param=mx.M3XYZExampleParam()
        self.read_config()
    def set_enable(self,x):
        self.command.enable=x
    def get_sensor(self):
        return self.status.sensor
    def get_sensor_array(self,i):
        return self.status.sensor_array(i)
    


