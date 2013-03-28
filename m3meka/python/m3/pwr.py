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
import m3.pwr_pb2 as mrt
from m3.component import M3Component
from m3.unit_conversion import *


class M3Pwr(M3Component):
    """Calibrated interface for the M3 Power Board"""
    def __init__(self,name):
        M3Component.__init__(self,name,type='m3pwr')
        self.status=mrt.M3PwrStatus()
        self.command=mrt.M3PwrCommand()
        self.param=mrt.M3PwrParam()
        self.read_config()

    def set_motor_power_on(self):
        self.command.enable_motor=True
    def set_motor_power_off(self):
        self.command.enable_motor=False
    def is_motor_power_on(self, ff):
        return self.status.motor_enabled

    def get_current_digital_mA( self):
        return self.status.current_digital
    def get_bus_voltage_V(self):
        return self.status.bus_voltage 
    def get_timestamp_uS(self):
        return self.status.timestamp

    def get_bus_current(self):
        return self.status.bus_current

