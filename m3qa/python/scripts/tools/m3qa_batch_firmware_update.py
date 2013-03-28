#! /usr/bin/python
# -*- coding: utf-8 -*-

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
import glob
import sys
import  m3.m3_bootloader
import m3.m3rt_toolbox as m3rt
import m3.toolbox as m3t

def easy_batch_write(p3):
	boards=get_boards()
	if len(boards.keys())==0:
		print 'No boards found'
		return
	set_bootloader_eeprom(boards,p3)
	time.sleep(1.0)
	m3t.load_m3ec_kmod()
	bld=m3.m3_bootloader.M3Bootloader()
	bld.SetVerbose(1)
	boards=batch_startup(bld,boards)
	bords=batch_ping(bld,boards)
	boards=batch_write(bld,boards)
	for k in boards.keys():
		print 'Write success',k,' : ',boards[k]['write_success']
	bld.Shutdown()
	m3t.unload_m3ec_kmod()
	time.sleep(1.0)
	restore_board_eeprom(boards,p3)
	
def batch_write(bld,boards):
	print 'Ready to write firmware.'
	print 'Prompt to write each board?[y]'
	prompt=m3t.get_yes_no('y')
	for k in boards.keys():
		if boards[k]['connected']:
			print 'Writing', boards[k]['firmware'],'to ',k  
			if prompt:
				print 'Hit enter to begin'
				raw_input()
			if bld.WriteProgramMemory(boards[k]['slave_id'],boards[k]['firmware']):
				print 'Write success for',k
				boards[k]['write_success']=True
			else:
				print 'Write fail for',k,'. Disconnecting...'
				boards[k]['connected']=False
				boards[k]['write_success']=False
	return boards
			
	

def batch_ping(bld,boards):
	for k in boards.keys():
		if boards[k]['connected']:
			if bld.Ping(boards[k]['slave_id']):
				print 'Ping sucess for',k
			else:
				print 'Ping fail for',k,'. Disconnecting...'
				boards[k]['connected']=False
	return boards

def batch_startup(bld,boards):
	for k in boards.keys():
		if bld.Startup(boards[k]['slave_id']):
			print 'Connected to bootloader for slave',k
			boards[k]['connected']=True
		else:
			print 'Unable to connect to bootloader for slave',k
			boards[k]['connected']=False
	return boards

	
def get_boards():
	print 'Finding burnable board from m3_config.yml...'		
	name_ec=m3t.get_ec_component_names()
	boards={}
	for n in name_ec:
		config=m3t.get_component_config(n)
		try:
			fw=config['firmware']+'.hex'
		except KeyError:
			print 'Missing firmware key for',n,'...skipping'
			fw=None
		sn=config['ethercat']['serial_number']
		try:
			chid=config['chid']
		except KeyError:
			chid=0
		ssn=get_slave_sn()
		sid=None
		for s in ssn:
			if sn==s[0]:
				sid=s[1]
		ff=m3t.get_m3_config_path()+'eeprom/eeprom*_sn_'+str(sn)+'.hex'
		eepf = glob.glob(ff)
		if len(eepf)==0:
			print 'EEPROM file not found',ff,'skipping board...'
		if len(eepf)>1:
			print 'Multiple eeproms found for',ff
			print 'Select correct eeprom ID'
			for i in range(len(eepf)):
				print i,' : ',eepf[i]
			idx=m3t.get_int()
			eepf=[eepf[idx]]
		if sid!=None and len(eepf)==1 and fw is not None and chid==0:
			boards[n]={'firmware':fw,
				   'sn':sn,
				   'slave_id':sid,
				   'eeprom':eepf[0],
				   'connected':False,
				   'write_success':False}
		
	print 'Found the following valid configurations: '
	print boards.keys()
	print
	print 'Selection type'
	print '-----------------'
	print  'a: all boards'
	print   's: single board'
	print   'm: multiple boards'
	t=raw_input()
	if t!='a' and t!='s' and t!='m':
		print 'Invalid selection'
		return {}
	rem=[]
	if t=='m':
		for k in boards.keys():
			print '-----------------------'
			print 'Burn ',k,'board [y]?'
			if not m3t.get_yes_no('y'):
				rem.append(k)
		for r in rem:
			boards.pop(r)
	if t=='s':
		k=m3t.user_select_components_interactive(boards.keys(),single=True)
		if len(k)==0:
			return {}
		boards={k[0]:boards[k[0]]}
	
	print '-------------------------------'
	print 'Burning the following boards: '
	for k in boards.keys():
		print k
	return boards

def display_boards(boards):
	print '----------Boards---------------'
	for k in boards.keys():
		print k,'SN: ',boards[k]['sn'],'Firmware: ',boards[k]['firmware'],'SlaveID: ',boards[k]['slave_id'],'Connected',boards[k]['connected'],'WriteSuccess',boards[k]['write_success']
		
def set_bootloader_eeprom(boards,p3):
	#Check if eeprom files are available
	filename=m3t.get_m3_config_path()+'eeprom/eeprom_bld_sn_0.hex'
	eepf=glob.glob(filename)
	if len(eepf)!=1:
		print 'BLD EEPROM file not found',filename
		return
	else:
		bld_eeprom=eepf[0]
	for k in boards.keys():	
		cmd='sudo ethercat -f -p '+str(boards[k]['slave_id'])+' sii_write '+bld_eeprom
		print 'Executing: ',cmd
		os.system(cmd)
	print 'Power cycle now. Hit return when ready'
	raw_input()
	time.sleep(4.0)
	if p3:
		m3rt.ethercat_bus_init(verbose=True)
	list_slaves()


def restore_board_eeprom(boards,p3):
	for k in boards.keys():	
		cmd='sudo ethercat -f -p '+str(boards[k]['slave_id'])+' sii_write '+boards[k]['eeprom']
		print 'Executing: ',cmd
		os.system(cmd)
	print 'Power cycle and hit return'
	raw_input()
	time.sleep(4.0)
	if p3:
		m3rt.ethercat_bus_init(verbose=True)
	list_slaves()

def list_slaves():
		print '--------------------------- Slaves -------------------------------'
		os.system('sudo ethercat slaves')
		print '------------------------------------------------------------------'	
		
def get_slave_sn():
	stdout_handle = os.popen("sudo ethercat slaves", "r")
	s = stdout_handle.read()
	if len(s)==0:
		return []
	ss=s.split('\n')
	q=[]
	ids=[]
	id=0
	for ll in ss: 
		f=ll.find('SN: ')
		if f>=0:
			s=ll[f:]
			ssn=''
			i=ssn.join([x for x in s if x.isdigit()])
			sn=int(i)
			q.append([sn,id])
		id=id+1
	return q
	

def print_usage():
	#print 'Hit enter to continue...'
	#raw_input()
	print
	print '---------- M3 Firmware Update -----------'
	print 'q: quit'
	print 'l: list slaves'
	print 'd: display boards'
	print 'e: easy batch write firmware'
	print '---------- Advanced Users ---------------'
	print 'w: batch write firmware'
	print 'p: batch ping slaves'
	print 'b: set bootloader eeprom'
	print 'r: restore eeprom'
	print 's: batch bootloader startup'
	print '------------------------------------------'
	print

if __name__ == '__main__':
	if (len(glob.glob('m3_controller*.hex'))==0):
		print 'No m3_controller*.hex files available'
		print 'Run from directory with dsPIC hex files'
	else:
		try:
			#print 'Use Port3 [n]?'
			#p3=m3t.get_yes_no('n')
			print 'Slaves on Port3 not supported (yet).'
			print 'If any Port3 slaves show with lsec, abort now and power cycle'
			print 'Do not run m3rt_bus_init.py'
			print 'Continue [y]?'
			if not m3t.get_yes_no('y'):
			  exit()
			p3=False
			print 'Quick batch write [y]?'
			if m3t.get_yes_no('y'):
				easy_batch_write(p3)
				exit()
			else:
				boards=get_boards()
			if not len(boards):
				exit()
			bld=m3.m3_bootloader.M3Bootloader()
			unload=False
			while (True):
				print_usage()
				c=raw_input()
				if c=='q' or c=='w' or c=='p' or c=='l' or c=='r' or c=='b' or c=='e' or c=='s' or c=='d':
					if c=='d':
						display_boards(boards)
					if c=='s':
						m3t.load_m3ec_kmod()
						unload=True
						boards=batch_startup(bld,boards)
					if c=='l':
						list_slaves()
					if c=='e':
						easy_batch_write(bld,boards)
						unload=False
					if c=='w':
						boards=batch_write(bld,boards)
					if c=='p':
						boards=batch_ping(bld,boards)
					if c=='r':
						m3t.unload_m3ec_kmod()
						unload=False
						restore_board_eeprom(boards,p3)
					if c=='b':
						set_bootloader_eeprom(boards,p3)
					if c=='q':
						break
				else:
					print 'Invalid command'
		except (KeyboardInterrupt,EOFError):
			pass
		if unload:
			m3t.unload_m3ec_kmod()
	
		    