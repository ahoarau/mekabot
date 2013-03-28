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

from m3.joint import M3Joint
import numpy as nu
import m3.unit_conversion as m3u
import math
import time
import m3.toolbox as m3t

class M3JointZLift(M3Joint):
    """Wrapper for 1DOF Z-Lift"""
    def __init__(self,name):
        M3Joint.__init__(self,name,type='m3joint_zlift')
        self.cb_mm_per_deg = self.config['calib']['cb_screw_pitch']/(self.config['calib']['cb_gearing']*360.0);
        self.cb_mNm_per_mN = self.config['calib']['cb_screw_pitch']*self.config['calib']['cb_screw_efficiency']/(2.0*math.pi*1000);


    def motor_torque_mNm_to_output_force_mN(self,x):
        return x/self.cb_mNm_per_mN
    def output_force_mN_to_motor_torque_mNm(self,x):
        return x*self.cb_mNm_per_mN;

    def get_force_mN(self): 
        return self.get_torque_mNm()/self.cb_mNm_per_mN
    def get_force_g(self):
        return m3u.mN2g(self.get_force_mN())
    def get_force_Lb(self):
        return m3u.mN2Lb(self.get_force_mN())

    def get_pos_m(self):
        return self.get_pos_mm()/1000.0
    def get_pos_mm(self):
        return self.get_theta_deg()*self.cb_mm_per_deg
    def get_pos_in(self):
        return m3u.m2in(self.get_pos_m())

    def set_force_mN(self,v):
        tq=nu.array(v)*self.cb_mNm_per_mN
        self.set_torque_mNm(v)
    def set_force_g(self,v):
        self.set_force_mN(m3u.g2mN(nu.array(v)))
    def set_force_Lb(self,v):
        self.set_force_mN(m3u.Lb2mN(nu.array(v)))

    def set_pos_m(self,v):
        self.set_pos_mm(nu.array(v)*1000.0)
    def set_pos_mm(self,v):
        self.set_theta_deg(nu.array(v)/self.cb_mm_per_deg)
    def set_pos_in(self,v):
        self.set_pos_m(m3u.in2m(nu.array(v)))

    def calibrate(self,proxy):
        print '----------------------------------------------------'
        if not self.get_encoder_calibrated():
            print 'Z-Lift encoder not yet calibrated. Calibrate now[y]?'
            if not m3t.get_yes_no('y'):
                return False
            print 'Disengage E-Stop. Hit enter when ready'
            raw_input()
            if self.get_limitswitch_neg(): 
                print 'Already at limit, moving up first...'
                self.set_mode_pwm()
                self.set_pwm(self.config['pwm_calibration_up'])
                proxy.step()
                time.sleep(4.0)
            print 'Moving down to limit'
            print 'Manual assist may be required'
            self.set_mode_pwm()
            self.set_pwm(self.config['pwm_calibration_down'])
            proxy.step()
            for i in range(20):
                if self.get_encoder_calibrated():
                    print 'Z-Lift encoder calibrated. Currently at position: ',self.get_pos_mm(),'(mm)'
                    return True
                print i,' : ', self.get_theta_deg()
                time.sleep(1.0)
                proxy.step()
            self.set_mode_off()
            proxy.step()
            if not self.get_encoder_calibrated():
                print 'Calibration failed'
                return False
        else:
            print 'Z-Lift encoder calibrated. Currently at position: ',self.get_pos_mm(),'(mm)'
            return True
