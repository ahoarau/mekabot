#! /usr/bin/python

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
#along with m3.  If not, see <http://www.gnu.org/licenses/>.


import os 
from m3.eeprom import *
import m3.toolbox as m3t
import time
from struct import pack

print 'Use this tool to overwrite the EEPROM serial number of an EtherCAT slave'
config_path=os.environ['M3_ROBOT']+'/robot_config/eeprom/'
print '--------------------------- Slaves -------------------------------'
os.system('sudo ethercat slaves')
print '------------------------------------------------------------------'	
print 'Enter slave id'
sid=m3t.get_int()
print 'Enter serial number'
sn=m3t.get_int()
print 'Enter slave name (eg MA1J0 or EL4132)'
name=m3t.get_string()

#Read in eeprom
cmd='sudo ethercat sii_read -p'+str(sid)
stdout_handle = os.popen(cmd, 'r')
eep=stdout_handle.read()
if len(eep)==0:
	print 'Unable to read slave EEPROM.'
	exit()
#Write orig to file
fn=config_path+'eeprom_'+name+'_sn_'+str(sn)+'_orig.hex'
out_port = open_binary_output_file(fn)
for c in eep:
	write_char(out_port,c)
out_port.close()
#Update binary sn field
hsn=pack('H',sn)
eep2=eep[:28]+hsn+eep[30:]
#Write to file
fn2=config_path+'eeprom_'+name+'_sn_'+str(sn)+'.hex'
out_port = open_binary_output_file(fn2)
for c in eep2:
	write_char(out_port,c)
out_port.close()
#Write to slave
print 'Write to slave [y]?'
if m3t.get_yes_no('y'):
	cmd='sudo ethercat -p '+str(sid)+' sii_write '+fn2
	print 'Executing: ',cmd
	os.system(cmd)
	print 'Power cycle and hit return'
	raw_input()
	time.sleep(4.0)
	print '--------------------------- Slaves -------------------------------'
	os.system('sudo ethercat slaves')
	print '------------------------------------------------------------------'
