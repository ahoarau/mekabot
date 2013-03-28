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
import m3.actuator_pb2
import m3.actuator_ec_pb2  as aepb
from m3.component import M3Component
from m3.unit_conversion import *
from scipy import polyval

class M3Actuator(M3Component):
    """Calibrated interface for the M3 Actuator"""
    def __init__(self,name,type='m3actuator'):
        M3Component.__init__(self,name,type=type)
        self.status=m3.actuator_pb2.M3ActuatorStatus()
        self.command=m3.actuator_pb2.M3ActuatorCommand()
        self.param=m3.actuator_pb2.M3ActuatorParam()
        self.read_config()
        
    #Utility API
    def set_mode(self,p):
    	self.command.ctrl_mode = p;
    def set_pwm(self,p):
        self.command.pwm_desired=p
    def set_torque_mNm(self, t):
        self.command.tq_desired=t
    def set_i_desired(self, i):
        self.command.i_desired = i
    def set_control_mode(self,m):
        self.command.ctrl_mode=m
    def set_mode_off(self):
        self.command.ctrl_mode=m3.actuator_pb2.ACTUATOR_MODE_OFF
    def set_mode_pwm(self):
        self.command.ctrl_mode=m3.actuator_pb2.ACTUATOR_MODE_PWM
    def set_mode_torque(self):
        self.command.ctrl_mode=m3.actuator_pb2.ACTUATOR_MODE_TORQUE
    def set_brake_off(self):
        self.command.brake_off=True
    def set_brake_on(self):
        self.command.brake_off=False
    def get_motor_temp_C(self): 
        return self.status.motor_temp
    def get_motor_temp_F(self): 
        return C2F(self.status.motor_temp)
    def get_amp_temp_C(self): 
        return self.status.amp_temp
    def get_amp_temp_F(self): 
        return C2F(self.status.amp_temp)
    def get_torque_mNm(self): 
        return self.status.torque
    def get_current_mA(self): 
        return self.status.current
    def get_theta_deg(self): 
        return self.status.theta
    def get_theta_rad(self): 
        return deg2rad(self.status.theta)
    def get_thetadot_rad(self): 
        return deg2rad(self.status.thetadot)
    def get_thetadot_deg(self): 
        return self.status.thetadot
    def get_thetadotdot_rad(self): 
        return deg2rad(self.status.thetadotdot)
    def get_thetadotdot_deg(self): 
        return self.status.thetadotdot
    def get_timestamp_uS(self):
        return self.status.base.timestamp
    def get_flags(self):
        return self.status.flags
    def get_limitswitch_pos(self):
        return self.status.flags & aepb.ACTUATOR_EC_FLAG_POS_LIMITSWITCH
    def get_limitswitch_neg(self):
        return self.status.flags & aepb.ACTUATOR_EC_FLAG_NEG_LIMITSWITCH
    def get_encoder_calibrated(self):
        return self.status.flags & aepb.ACTUATOR_EC_FLAG_QEI_CALIBRATED
