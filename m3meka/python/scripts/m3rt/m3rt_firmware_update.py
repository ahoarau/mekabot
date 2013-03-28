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
#along with M3.  If not, see <http://www.gnu.org/licenses/>.


import os
import time
import glob
import sys
import  m3.m3_bootloader
import m3.m3rt_toolbox as m3t
import m3.toolbox as m3b

def echub_init():
	print 'Opening ECHUB Port3 on Slave 0'
	m3t.echub_init()
	
def get_int():
	while True:
		try:
			x=int(input())
			break
		except (NameError, SyntaxError):
			print 'Invalid value, try again'
	return x

def get_m3_lib_path():
	cmd='echo /lib/modules/$(uname -r)/m3/'
	stdout_handle = os.popen(cmd, "r")
	s = stdout_handle.read()
	s=s[:-1]
	return s

def get_m3_config_path():
	try:
		return os.environ['M3_ROBOT']+'/robot_config/eeprom/'
	except KeyError:
		print 'SET YOUR M3_ROBOT ENVIRONMENT VARIABLE'
	return ''
	
	
class M3FirmwareUpdate:
	def __init__(self,p3=True):
		self.p3=p3
		self.restart()
		self.kmod_loaded=False
		
	def restart(self):
		self.slave_id=None
		self.sn=None
		self.eeprom_filename=None
		self.bld=None
		
	def shutdown(self):
		if self.bld is not None:
			self.bld.Shutdown()
		self.unload_kmod()
		self.restore_eeprom()
	
		
	def get_hex_file(self):
		files=glob.glob('m3_controller*.hex')
		if (len(files)==0):
			print 'No hex files available'
			return None
		print '-------------------- Available Hex Files --------------------------'
		for i in range(len(files)):
			print 'Id: ',i,'File: ', files[i]
		print 'Enter File Id'
		id=get_int()
		return files[id]
		
	def set_slave(self):
		self.restore_eeprom() #Will restore previous slave if is one
		while True:
			self.restart()
			print
			self.list_slaves()
			print 'Slave ID to update?'
			slave_id=get_int()
			
			stdout_handle = os.popen("sudo ethercat slaves", "r")
			s = stdout_handle.read()
			s=s.split('\n')[slave_id]
			s=s[s.find('SN: '):]
			ssn=''
			sn=int(ssn.join([x for x in s if x.isdigit()]))
			
			#Check if eeprom files are available
			filename=get_m3_config_path()+'eeprom_bld_sn_0.hex'
			eepf=glob.glob(filename)
			if len(eepf)!=1:
				print 'BLD EEPROM file not found',filename
				self.eep_bld_filename=None
			else:
				self.eep_bld_filename=eepf[0]
				
			filename=get_m3_config_path()+'eeprom*_sn_'+str(sn)+'.hex'
			eepf = glob.glob(filename)
			if len(eepf)!=1:
				print 'EEPROM file not found',filename
				self.restart()
			else:
				eeprom_filename=eepf[0]
				print '--------------------------------'
				print 'Serial Number: ',sn
				print 'Id', slave_id
				print 'EEPROM',eeprom_filename
				print 'Is this correct (y/n)?'
				r=raw_input()
				print 
				if r=='y':
					self.slave_id=slave_id
					self.sn=sn
					self.eeprom_filename=eeprom_filename
					self.overwrite_eeprom()
					self.load_kmod()
					self.bld=m3.m3_bootloader.M3Bootloader()
					if not self.bld.Startup(self.slave_id):
						print 'Unable to connect to slave bootloader...'
						self.unload_kmod()
						self.bld.Shutdown()
						self.restart()
						return
					return 
					
	def restore_eeprom(self):
		if self.eeprom_filename is not None:
			print 'Restoring EEPROM with',self.eeprom_filename
			cmd='sudo ethercat -f -p '+str(self.slave_id)+' sii_write '+self.eeprom_filename
			os.system(cmd)
			print 'Cycle the slave power now. '
			print 'Hit enter when done...'
			raw_input()
			if p3:
				echub_init()
			time.sleep(1.0)
			self.eeprom_filename=None
			self.list_slaves()
			
	def overwrite_eeprom(self):
		if self.eeprom_filename is not None:
			print 'Overwriting EEPROM with: ',self.eep_bld_filename
			if False: #Old method v0.3
				eep_bld=self.eeprom_filename+'.bld'
				os.system('cp '+self.eeprom_filename+' '+eep_bld)
				f=open(eep_bld,'rb+')
				f.seek(8) #Position of Station Id on Eeprom file
				f.write('\0\1') #Set to non-zero
				f.close()
			cmd='sudo ethercat -f -p '+str(self.slave_id)+' sii_write '+self.eep_bld_filename
			print 'Running cmd: ',cmd
			os.system(cmd)
			if False: #Old method
				os.system('rm '+eep_bld)
			print 'Cycle the slave power now. '
			print 'Hit enter when done...'
			raw_input()
			if p3:
				echub_init()
			time.sleep(1.0)
			self.list_slaves()
	
	def list_slaves(self):
		print '--------------------------- Slaves -------------------------------'
		os.system('sudo ethercat slaves')
		print '------------------------------------------------------------------'
    
	def load_kmod(self):
		cmd='sudo m3rt_insmods' #'sudo insmod '+get_m3_lib_path()+'m3ec.ko'
		os.system(cmd) 
		self.kmod_loaded=True

	def unload_kmod(self):
		if self.kmod_loaded:
			cmd='sudo m3rt_rmmods' #'sudo rmmod '+get_m3_lib_path()+'m3ec.ko'
			os.system(cmd)
			self.kmod_loaded=False
    
	def read_program_memory(self):
		print 'Enter address (0x000000)'
		address=raw_input()
		if self.bld is not None:
			self.bld.ReadProgramMemory(self.slave_id,address)
		else:
			print 'You must first select a slave'
	
	def write_program_memory(self):
		if self.bld is not None:
			f=self.get_hex_file()
			if f is not None:
				self.bld.WriteProgramMemory(self.slave_id,f)
		else:
			print 'You must first select a slave'
			
	def read_config(self):
		if self.bld is not None:
			print '------------------------------------------------------------------'
			self.bld.ReadConfigId(self.slave_id)
			print '------------------------------------------------------------------'
		else:
			print 'You must first select a slave'
			
		
	def display_slave(self):
		print '--------------------------------'
		print 'Slave',self.slave_id
		print 'Serial number',self.sn
		print 'Bootloader active',self.bld is not None
		print 'EEPROM file',self.eeprom_filename
	
	def ping(self):
		if self.bld is not None:
			print '------------------------------------------------------------------'
			self.bld.Ping(self.slave_id)
			print '------------------------------------------------------------------'
		else:
			print 'You must first select a slave'
			
def print_usage():
	
	print 'Hit enter to continue...'
	raw_input()
	print
	print '---------- M3 Firmware Update -----------'
	print 'q: quit'
	print 's: set slave'
	print 'l: list slaves'
	print 'r: read program memory'
	print 'w: write hex file'
	print 'c: read slave config'
	print 'p: ping bootloader'
	print 'd: display current slave'
	print '------------------------------------------'
	print

if __name__ == '__main__':
	if (len(glob.glob('m3_controller*.hex'))==0):
		print 'No m3_controller*.hex files available'
		print 'Run from directory with dsPIC hex files'
	else:
		print 'Require Port3 [y]?'
		p3=m3b.get_yes_no('y')
		f=M3FirmwareUpdate(p3)
		if p3:
			echub_init()
		try:
			while (True):
				print_usage()
				
				c=raw_input()
				if c=='q'or c=='q'or c=='r' or c=='w' or c=='c' or c=='p' or c=='d' or c=='s' or c=='l':
					if c=='s':
						f.set_slave()
					if c=='l':
						f.list_slaves()
					if c=='r':
						f.read_program_memory()
					if c=='c':
						f.read_config()
					if c=='w':
						f.write_program_memory()
					if c=='p':
						f.ping()
					if c=='d':
						f.display_slave()
					if c=='q':
						f.shutdown()
						break
				else:
					print 'Invalid command'
		except (KeyboardInterrupt,EOFError):
			pass
		f.shutdown() #just in case
	
		    