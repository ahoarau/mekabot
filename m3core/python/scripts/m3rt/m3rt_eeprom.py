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
import m3.component_factory as mcf

def get_int():
	while True:
		try:
			x=int(input())
			break
		except (NameError, SyntaxError):
			print 'Invalid value, try again'
	return x

			
m3_product_codes = mcf.m3_product_codes

try:
	config_path=os.environ['M3_ROBOT']+'/robot_config/eeprom/'
	try:
		print 'Product code'
		print '--------------------'
		for k in m3_product_codes:
			print k,' : ',m3_product_codes[k]
		code=int(raw_input())
		
		slave_cfg_dicts = mcf.m3_eeprom_cfgs[code]
		fpfx = mcf.m3_fpfx[code]
		
		
			
		#Get EEPROM Version
		if len(slave_cfg_dicts)>1:
			while True:
				print 'EEPROM version ID: '
				print '-----------------------'
				for i in range(len(slave_cfg_dicts)):
					print i,': ',slave_cfg_dicts[i]['Version Notes']
				version=get_int()
				if version>=0 and version<len(slave_cfg_dicts):
					slave_cfg_dict=slave_cfg_dicts[version]
					break
				else:
					print 'Invalid version ID'
		else:
			slave_cfg_dict=slave_cfg_dicts[0]
		#Get Serial Number	
		print 'Serial Number: '
		id=get_int()
		#Get Prefix
		print 'Component Prefix (eg MA0J0)'
		comp=raw_input()
		slave_cfg_dict['Serial Number']=id
		slave_cfg_dict['Categories'][0]['Data'][1]=slave_cfg_dict['Categories'][0]['Data'][1]+' [Name: '+comp+' ; SN: '+str(id)+']'
		slave_cfg_dict['Product Code']=code
		fn=config_path+'eeprom_'+fpfx+'_sn_'+str(id)+'.hex'
		dict_to_eeprom(slave_cfg_dict,fn)
		print 'Write EEPROM to active slave (y/n)?'
		if raw_input()=='y':
			print '---------- Available Slaves -------------'
			os.system('sudo ethercat slaves')
			print '-----------------------------------------'
			print 'Enter slave id'
			id=get_int()
			print 'Writing EEPROM...'
			os.system('sudo ethercat si_write --force -p '+str(id)+' '+fn)
			print 'Done...'
	except ValueError:
		print 'Bad value'
except ValueError:#KeyError:
	print 'SET YOUR M3_CONFIG ENVIRONMENT VARIABLE'




