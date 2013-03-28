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
import m3.joint_pb2 as jpb
import m3.actuator_pb2 as apb
import m3.actuator_ec_pb2 as aepb
from m3.component import M3Component
from m3.unit_conversion import *
from scipy import polyval

class M3Joint(M3Component):
    """Interface for joint-space controllers of an actutaor
    """
    def __init__(self,name,type='m3joint'):
        M3Component.__init__(self,name,type=type)
        self.status=jpb.M3JointStatus()
        self.command=jpb.M3JointCommand()
        self.param=jpb.M3JointParam()
        self.read_config()

    #Utility API
    def set_pwm(self,p):
        """Desired joint PWM"""
        self.command.pwm_desired=p
    def set_torque_mNm(self, t):
        """Desired joint torque"""
        self.command.tq_desired=t
    def set_theta_rad( self,q):
        """Desired joint angle"""
        self.command.q_desired=rad2deg(q)
    def set_theta_deg( self,q):
        """Desired joint angle"""
        self.command.q_desired=q
    def set_stiffness(self, s):
        """Desired joint stiffness for JOINT_MODE_THETA_GC. Value: 0-1.0"""
        self.command.q_stiffness=s
    def set_thetadot_deg(self,qd):
        self.command.qdot_desired=qd
    def set_slew_rate(self,qd):
        self.command.q_slew_rate=qd
    def set_slew_rate_proportion(self,qd):
	self.command.q_slew_rate=qd*self.param.max_q_slew_rate
    def set_thetadot_rad(self,qd):
        self.command.qdot_desired=rad2deg(qd)
    def set_control_mode(self,m):    
        self.command.ctrl_mode=m
    def set_mode_off(self):
        self.command.ctrl_mode=jpb.JOINT_MODE_OFF
    def set_mode_pwm(self):
        self.command.ctrl_mode=jpb.JOINT_MODE_PWM
    def set_mode_torque(self):
        self.command.ctrl_mode=jpb.JOINT_MODE_TORQUE
    def set_mode_torque_gc(self):
        self.command.ctrl_mode=jpb.JOINT_MODE_TORQUE_GC
    def set_mode_theta(self):
        self.command.ctrl_mode=jpb.JOINT_MODE_THETA
    def set_mode_theta_gc(self):
        self.command.ctrl_mode=jpb.JOINT_MODE_THETA_GC
    def set_mode_theta_mj(self):
        self.command.ctrl_mode=jpb.JOINT_MODE_THETA_MJ
    def set_mode_theta_gc_mj(self):
        self.command.ctrl_mode=jpb.JOINT_MODE_THETA_GC_MJ
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
    def get_torque_gravity_mNm(self): 
        return self.status.torque_gravity
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
    
	