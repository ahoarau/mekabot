
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

import os
import time

# #######################################################################################
"""Configures the M3 EtherCAT hub to enable Port 3
The current bus configuration uses Port3 of the ET1200 on 
the EtherCAT Hub (ECHUB). Port3 is closed by default on powerup, 
unlike the other ports. This script opens the port by writing the correct
bits to the DL control register (0x101) on the ET1200. The ECHUB is always the first
slave (p0). 

Control bits \\0000 (octal) writes 0 to 0x101, placing on 4 ports in mode 00 (auto-on)
Control bits \\0303 (octal) writes 195 to 0x101, placing ports 0,3 in mode 11 (closed)
and ports 1,2 in mode (00) auto-on.

M3 V0.0:
It isn't clear why, but the last slave in in the bus-chain must have its port 0 manually
closed, hence the final call.  With the SEAX2 boards, the unconnected port is always port 0. 
However, it seems that this isn't  necessary if it is port 1 that is unconnected. 

M3 V1.0:


This code is very dependent on the particular M3 hardware. 
In future versions this functionality should go away/be integrated into the master patch.
"""

def is_slave_last_in_chain(id):
    cmd='sudo ethercat slaves -v -p'+str(id) #reg_read -p'+str(id)+' 0x0110 4'
    stdout_handle = os.popen(cmd, "r")
    s = stdout_handle.read()
    s=s.split('\n')
    for i in range(len(s)):
        if s[i].find('Port ')>=0:
            #Format: Port  Type  Link  Loop    Signal  NextSlave  RxTime [ns]  Diff [ns]   NextDc [ns]
            port0=s[i+1].split()
            port1=s[i+2].split()
            if port0[2]=='down' or port1[2]=='down': #link down
                return True
            if port0[3]=='closed' or port1[3]=='closed': #loop closed
                return True
            break
 
def get_echub_ids():
    cmd = 'sudo ethercat slaves | grep ECHUB'
    stdout_handle = os.popen(cmd, "r")
    s = stdout_handle.read()
    s=s.split('\n')
    ret=[]
    for x in s:
	if len(x):
	    ret.append(int(x[:3]))
    #print ret
    #ret = [int(x[:2]) for x in s]
    return ret

def get_num_slaves_init_e():
    cmd = 'sudo ethercat slaves | grep INIT\ \ \ E'
    stdout_handle = os.popen(cmd, "r")
    s = stdout_handle.read()
    s=s.split('\n')
    return len(s)-1

def get_num_slaves_init():
    cmd = 'sudo ethercat slaves | grep INIT'
    stdout_handle = os.popen(cmd, "r")
    s = stdout_handle.read()
    s=s.split('\n')
    return len(s)-1

def get_num_slaves_preop():
    cmd = 'sudo ethercat slaves | grep PREOP'
    stdout_handle = os.popen(cmd, "r")
    s = stdout_handle.read()
    s=s.split('\n')
    return len(s)-1

def restart_ethercat():
    cmd = 'sudo /etc/init.d/ethercat restart'
    os.system(cmd)

def get_num_slaves_op():
    cmd = 'sudo ethercat slaves | grep OP'
    stdout_handle = os.popen(cmd, "r")
    s = stdout_handle.read()
    s=s.split('\n')
    nop=len(s)-1
    npeop=get_num_slaves_preop() #preop show up as op in grep
    return nop-npeop

def get_num_slaves():
    cmd = 'sudo ethercat slaves'
    stdout_handle = os.popen(cmd, "r")
    s = stdout_handle.read()
    s=s.split('\n')
    return len(s)-1

def get_ids_last_in_chain():
    ids=[]
    for i in range(get_num_slaves()):
        if is_slave_last_in_chain(i):
            ids.append(i)
    return ids

# Note, this command depends on the shell used. Bash works.
def set_all_ports_always_open_v0_1(id):
    cmd='echo -ne \\\\0000 | sudo ethercat reg_write -v -p'+str(id)+' 0x0101 -'
    os.system(cmd)

def set_all_ports_always_open(id):
    cmd='echo -ne \\\\0240 | sudo ethercat reg_write -v -p'+str(id)+' 0x0101 -'
    os.system(cmd)
    
def close_ports_0_3_v0_1(id):
     cmd='echo -ne \\\\0303 | sudo ethercat reg_write -v -p'+str(id)+' 0x0101 -'
     os.system(cmd)

def close_ports_0_3(id):
     cmd='echo -ne \\\\0314 | sudo ethercat reg_write -v -p'+str(id)+' 0x0101 -'
     os.system(cmd)
     
def echub_init():
    #ids=get_ids_last_in_chain()
    #print 'IDS',ids
    #for i in ids:
    #    close_ports_0_3(i)
    ids=get_echub_ids()
    #for i in ids:
    #print 'Opening ports on slave: ',i
    set_all_ports_always_open(ids[-1]) #Hack, just open last hub
    #ids=get_ids_last_in_chain()
    #print 'Last in chain: ',ids
 

def ethercat_bus_init(verbose=True):
	print 'Initializing the M3 EtherCAT bus...'
	print '-----------------------------------'
	if verbose:
		print 'Opening Port3'
	echub_init()
	time.sleep(4.0)
	n=get_num_slaves()
	if verbose:
		print 'Slaves on bus: ',n
	i=get_num_slaves_init()
	ie=get_num_slaves_init_e()
	p=get_num_slaves_preop()
	o=get_num_slaves_op()
	if verbose:
		print 'Confirming slave states'
	for i in range(5):
		if o!=0:
			if verbose:
				print 'Slaves in operational state. Stop all M3 processes and try again'
			exit()
		if p==n:
			if verbose:
				print 'All slaves in state PREOP. Initialization successful...'
			exit()
		if ie>0:
			if verbose:
				print 'Num Slaves:',n,'INIT',i,'INIT_E',ie,'PREOP',p
			time.sleep(1.0)
			i=get_num_slaves_init()
			ie=get_num_slaves_init_e()
			p=get_num_slaves_preop()
	if ie!=0:
		if verbose:
			print ie,'slaves still in state INIT_E. Restarting EtherCAT master...'
		restart_ethercat()
		time.sleep(3.0)
		i=get_num_slaves_init()
		ie=get_num_slaves_init_e()
		p=get_num_slaves_preop()
		for i in range(10):
			p=get_num_slaves_preop()
			if verbose:
				print i,': Slaves in PREOP',p,', of ',n
			if p==n:
				print 'All slaves in state PREOP. Initialization successful...'
				return True
			time.sleep(1.0)

		print 'Unable to initialize the EtherCAT bus. Power cycle the robot and try again'
		return False
	    
# #######################################################################################

