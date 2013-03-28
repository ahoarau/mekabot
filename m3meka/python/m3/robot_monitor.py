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

import m3.robot_monitor_pb2 as mrm
import yaml
import os
import subprocess as sub
from m3.toolbox import *
from m3.component import M3Component
import m3.component_factory as m3f
import m3.m3rt_toolbox as m3t
import m3.toolbox as m3tt
import time
import re
from time import strftime


class M3RobotMonitor(M3Component):
   
    def __init__(self,name):
        M3Component.__init__(self,name,type='m3robot_monitor')
        
        self.status=mrm.M3RobotMonitorStatus()
        self.command=mrm.M3RobotMonitorCommand()
        self.param=mrm.M3RobotMonitorParam()
        self.read_config()
        
    def print_status(self):
        for i in range(len(self.status.volt_comp)):
            print self.status.volt_comp[i].component_name, self.status.volt_comp[i].state, self.status.volt_comp[i].msg
            
        for i in range(len(self.status.temp_comp)):
            print self.status.temp_comp[i].component_name, self.status.temp_comp[i].state, self.status.temp_comp[i].msg
        
    def read_config(self):
        M3Component.read_config(self)
        
        try:
            f=file(self.config_name,'r')
            self.config= yaml.safe_load(f.read())
        except (IOError, EOFError):
            print 'Config file not present:',self.config_name
            return
    
        if self.config.has_key('volt_components'):
            for k in self.config['volt_components']:
                self.param.volt_comp.add()
                self.param.volt_comp[-1].component_name = k
                self.param.volt_comp[-1].max_val_warn = self.config['volt_components'][k]['max_warn']
                self.param.volt_comp[-1].min_val_warn = self.config['volt_components'][k]['min_warn']
                self.param.volt_comp[-1].max_val_err = self.config['volt_components'][k]['max_err']
                self.param.volt_comp[-1].min_val_err = self.config['volt_components'][k]['min_err']
                
        if self.config.has_key('temp_components'):
            for k in self.config['temp_components']:
                self.param.temp_comp.add()
                self.param.temp_comp[-1].component_name = k
                self.param.temp_comp[-1].max_val_warn = self.config['temp_components'][k]['max_warn']
                self.param.temp_comp[-1].min_val_warn = self.config['temp_components'][k]['min_warn']
                self.param.temp_comp[-1].max_val_err = self.config['temp_components'][k]['max_err']
                self.param.temp_comp[-1].min_val_err = self.config['temp_components'][k]['min_err']