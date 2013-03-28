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
import m3.actuator_ec_pb2 as mec
from m3.component import M3Component


class M3ActuatorEc(M3Component):
    """EtherCAT interface for the M3 Actuator"""
    def __init__(self,name,type='m3actuator_ec'):
        M3Component.__init__(self,name,type=type)
        self.status=mec.M3ActuatorEcStatus()
        self.command=mec.M3ActuatorEcCommand()
        self.param=mec.M3ActuatorEcParam()
        self.read_config()
    def get_timestamp(self):
        return self.status.timestamp
    def get_qei_on(self):
        return self.status.qei_on
    def get_adc_torque(self):
        return self.status.adc_torque
    def get_adc_ext_temp(self):
        return self.status.adc_ext_temp
    def get_adc_amp_temp(self):
        return self.status.adc_amp_temp
    def set_encoder_zero(self,x):
	"""Zero incremental encoder on supported boards"""
	if x:
	    self.param.config=self.param.config | mec.ACTUATOR_EC_CONFIG_CALIB_QEI_MANUAL
	else:
	    self.param.config=self.param.config &~ mec.ACTUATOR_EC_CONFIG_CALIB_QEI_MANUAL
