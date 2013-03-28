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
import m3.ledx2_ec_pb2 as mec
from m3.component import M3Component


class M3LedX2Ec(M3Component):
    """EtherCAT interface for 2Branch 2Board OctoBrite LED controller"""
    def __init__(self,name):
	M3Component.__init__(self,name,type='m3ledx2_ec')
	self.status=mec.M3LedX2EcStatus()
	self.command=mec.M3LedX2EcCommand()
	self.param=mec.M3LedX2EcParam()
	self.read_config()
	self.slew_aa_r=m3t.M3Slew()
	self.slew_aa_g=m3t.M3Slew()
	self.slew_aa_b=m3t.M3Slew()
	self.slew_ab_r=m3t.M3Slew()
	self.slew_ab_g=m3t.M3Slew()
	self.slew_ab_b=m3t.M3Slew()
	self.slew_ba_r=m3t.M3Slew()
	self.slew_ba_g=m3t.M3Slew()
	self.slew_ba_b=m3t.M3Slew()
	self.slew_bb_r=m3t.M3Slew()
	self.slew_bb_g=m3t.M3Slew()
	self.slew_bb_b=m3t.M3Slew()
    def enable_leds(self):
	self.command.enable_a=True
	self.command.enable_b=True
	
    def disable_leds(self):
	self.command.enable_a=False
	self.command.enable_b=False

    def slew_rgb(self,rgb,rate): 
	#List of 4 rgb triples 
	#rate is max increment per cycle
	self.command.branch_a.board_a.r=int(self.slew_aa_r.step(rgb[0][0],rate))
	self.command.branch_a.board_a.g=int(self.slew_aa_g.step(rgb[0][1],rate))
	self.command.branch_a.board_a.b=int(self.slew_aa_b.step(rgb[0][2],rate))
	self.command.branch_a.board_b.r=int(self.slew_ab_r.step(rgb[1][0],rate))
	self.command.branch_a.board_b.g=int(self.slew_ab_g.step(rgb[1][1],rate))
	self.command.branch_a.board_b.b=int(self.slew_ab_b.step(rgb[1][2],rate))
	self.command.branch_b.board_a.r=int(self.slew_ba_r.step(rgb[2][0],rate))
	self.command.branch_b.board_a.g=int(self.slew_ba_g.step(rgb[2][1],rate))
	self.command.branch_b.board_a.b=int(self.slew_ba_b.step(rgb[2][2],rate))
	self.command.branch_b.board_b.r=int(self.slew_bb_r.step(rgb[3][0],rate))
	self.command.branch_b.board_b.g=int(self.slew_bb_g.step(rgb[3][1],rate))
	self.command.branch_b.board_b.b=int(self.slew_bb_b.step(rgb[3][2],rate))
		
    def set_rgb(self,rgb): 
	#List of 4 rgb triples 
	self.command.branch_a.board_a.r=int(rgb[0][0])
	self.command.branch_a.board_a.g=int(rgb[0][1])
	self.command.branch_a.board_a.b=int(rgb[0][2])
	self.command.branch_a.board_b.r=int(rgb[1][0])
	self.command.branch_a.board_b.g=int(rgb[1][1])
	self.command.branch_a.board_b.b=int(rgb[1][2])
	self.command.branch_b.board_a.r=int(rgb[2][0])
	self.command.branch_b.board_a.g=int(rgb[2][1])
	self.command.branch_b.board_a.b=int(rgb[2][2])
	self.command.branch_b.board_b.r=int(rgb[3][0])
	self.command.branch_b.board_b.g=int(rgb[3][1])
	self.command.branch_b.board_b.b=int(rgb[3][2])