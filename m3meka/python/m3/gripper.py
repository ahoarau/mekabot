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

from m3.joint_array import M3JointArray
#import Numeric as nu
import m3.unit_conversion as m3u

class M3Gripper(M3JointArray):
    """Wrapper for 2DOF SEA Gripper"""
    def __init__(self,name):
        M3JointArray.__init__(self,name,ndof=2,ctype='m3gripper')
    def get_force_mN(self): 
        return self.get_torque_mNm()*self.config['calib']['cb_drive_radius_m']
    def get_force_g(self):
        return m3u.mN2g(self.get_force_mN())
    def get_force_Lb(self):
        return m3u.mN2Lb(self.get_force_mN())
    def get_pos_m(self):
        return self.get_theta_rad()*self.config['calib']['cb_drive_radius_m']
    def get_pos_mm(self):
        return self.get_pos_m()*1000.0
    def get_pos_in(self):
        return m3u.m2in(self.get_pos_m())

    def set_force_mN(self,v):
	tq=nu.array(v)*self.config['calib']['cb_drive_radius_m']
	self.set_torque_mNm(v)
    def set_force_g(self,v):
	self.set_force_mN(m3u.g2mN(nu.array(v)))
    def set_force_Lb(self,v):
	self.set_force_mN(m3u.Lb2mN(nu.array(v)))
	
    def set_pos_m(self,v):
	self.set_theta_rad(nu.array(v)/self.config['calib']['cb_drive_radius_m'])
    def set_pos_mm(self,v):
	self.set_pos_m(nu.array(v)/1000.0)
    def set_pos_in(self,v):
	self.set_pos_m(m3u.in2m(nu.array(v)))