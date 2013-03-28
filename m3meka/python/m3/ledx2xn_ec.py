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
import time
import math
import os 
import m3.toolbox as m3t
import m3.ledx2xn_ec_pb2 as mec
import copy
from m3.component import M3Component

mode_ledx2xn_off=0
mode_ledx2xn_direct=1
mode_ledx2xn_slew=2
mode_ledx2xn_circulate=3

class M3LedX2XNEc(M3Component):
    """EtherCAT interface for 2Branch NBoard A6281 LED controller"""
    def __init__(self,name):
        M3Component.__init__(self,name,type='m3ledx2xn_ec')
        self.status=mec.M3LedX2XNEcStatus()
        self.command=mec.M3LedX2XNEcCommand()
        self.param=mec.M3LedX2XNEcParam()
        self.read_config()
        self.slew={'branch_a':[],'branch_b':[]}
        self.nled={'branch_a':self.config['param']['n_branch_a'],'branch_b':self.config['param']['n_branch_b']}
        self.phase={'branch_a':[],'branch_b':[]}
        for i in range(self.nled['branch_a']):
            self.command.branch_a.r.append(0)
            self.command.branch_a.g.append(0)
            self.command.branch_a.b.append(0)
            self.slew['branch_a'].append([m3t.M3Slew(),m3t.M3Slew(),m3t.M3Slew()])
            self.phase['branch_a'].append(math.pi*i/(math.pi*2))
        for i in range(self.nled['branch_b']):
            self.command.branch_b.r.append(0)
            self.command.branch_b.g.append(0)
            self.command.branch_b.b.append(0)
            self.slew['branch_b'].append([m3t.M3Slew(),m3t.M3Slew(),m3t.M3Slew()])
            self.phase['branch_b'].append(math.pi*i/(math.pi*2))
            
        self.circ_slew={'branch_a':[],'branch_b':[]}
        for i in range(6):
            for b in ['branch_a','branch_b']:
                self.circ_slew[b].append(m3t.M3Slew())
                
        self.command_branch={'branch_a':self.command.branch_a,'branch_b':self.command.branch_b}
        self.mode={'branch_a':mode_ledx2xn_off,'branch_b':mode_ledx2xn_off}
        self.rgb={'branch_a':[[0,0,0]]*self.nled['branch_a'],'branch_b':[[0,0,0]]*self.nled['branch_b']}
        self.slew_rate={'branch_a':0,'branch_b':0}
        self.circ_rate={'branch_a':0,'branch_b':0}
        self.circ_slew_rate={'branch_a':0,'branch_b':0}
        self.circ_rgb_1={'branch_a':None,'branch_b':None}
        self.circ_rgb_2={'branch_a':None,'branch_b':None}
        self.step={'branch_a':0,'branch_b':0}
        self.disable_leds()

    def enable_leds(self):
        self.command.enable_a=True
        self.command.enable_b=True

    def disable_leds(self):
        self.command.enable_a=False
        self.command.enable_b=False
        
    def set_mode_direct(self,branch,rgb):
        """rgb values 0-1023"""
        self.mode[branch]=mode_ledx2xn_direct
        self.rgb[branch]=rgb

    def set_mode_slew(self,branch,slew_rate,rgb):
        """rgb values 0-1023"""
        self.mode[branch]=mode_ledx2xn_slew
        self.slew_rate[branch]=slew_rate
        self.rgb[branch]=rgb
        
        
    #Rate: Period (s)
    #Circulate rgb_1 to rgb_2 around loop of N leds
    def set_mode_circulate(self,branch,circ_rate,slew_rate,rgb_1,rgb_2):
        """rgb values 0-1023"""
        ##pass in two rgb tuples. interpolate beween them to fill array
        self.mode[branch]=mode_ledx2xn_circulate
        self.circ_rate[branch]=circ_rate
        self.circ_rgb_1[branch]=[self.circ_slew[branch][0].step(rgb_1[0],slew_rate),
                                 self.circ_slew[branch][1].step(rgb_1[1],slew_rate),
                                 self.circ_slew[branch][2].step(rgb_1[2],slew_rate)]
        self.circ_rgb_2[branch]=[self.circ_slew[branch][3].step(rgb_2[0],slew_rate),
                                 self.circ_slew[branch][4].step(rgb_2[1],slew_rate),
                                 self.circ_slew[branch][5].step(rgb_2[2],slew_rate)]

    def step_animation(self,branch):

        for i in range(self.nled[branch]):
            if self.mode[branch]==mode_ledx2xn_direct:
                self.command_branch[branch].r[i]=int(self.rgb[branch][i][0])
                self.command_branch[branch].g[i]=int(self.rgb[branch][i][1])
                self.command_branch[branch].b[i]=int(self.rgb[branch][i][2])
            if self.mode[branch]==mode_ledx2xn_slew:
                self.command_branch[branch].r[i]=int(self.slew[branch][i][0].step(self.rgb[branch][i][0],self.slew_rate[branch]))
                self.command_branch[branch].g[i]=int(self.slew[branch][i][1].step(self.rgb[branch][i][1],self.slew_rate[branch]))
                self.command_branch[branch].b[i]=int(self.slew[branch][i][2].step(self.rgb[branch][i][2],self.slew_rate[branch]))
            
            if self.mode[branch]==mode_ledx2xn_circulate:
                r1=self.circ_rgb_1[branch][0]
                r2=self.circ_rgb_2[branch][0]
                g1=self.circ_rgb_1[branch][1]
                g2=self.circ_rgb_2[branch][1]
                b1=self.circ_rgb_1[branch][2]
                b2=self.circ_rgb_2[branch][2]
                t=time.time()
                r=r1+(r2-r1)*math.cos(self.phase[branch][i]+2*math.pi*t/self.circ_rate[branch])
                g=g1+(g2-g1)*math.cos(self.phase[branch][i]+2*math.pi*t/self.circ_rate[branch])
                b=b1+(b2-b1)*math.cos(self.phase[branch][i]+2*math.pi*t/self.circ_rate[branch])
   
                self.command_branch[branch].r[i]=int(r)
                self.command_branch[branch].g[i]=int(g)
                self.command_branch[branch].b[i]=int(b)
                self.slew[branch][i][0].val=r
                self.slew[branch][i][0].val=g
                self.slew[branch][i][0].val=b
                    
        

    def load_command(self):
        """Called before every command msg sent from proxy"""
        self.step_animation('branch_a')
        self.step_animation('branch_b')



