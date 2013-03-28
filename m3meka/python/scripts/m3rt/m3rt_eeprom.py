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

def get_int():
	while True:
		try:
			x=int(input())
			break
		except (NameError, SyntaxError):
			print 'Invalid value, try again'
	return x

			
m3_product_codes={ 'm3_actx1':1010L,
		  'm3_actx2':1011L,
		  'm3_actx3':1012L,
		  'm3_actx4':1013L,
		  'm3_tactx2':1014L,
		  'm3_ledx2':1009L,
		  'm3_bld':1007L,
		  'm3_echub':1000L,
		  'm3_echub4':1001L,
		  'm3_pwr':1002L,
		  'm3_loadx6':1003L,
		  'm3_armh':1015L,
		  'm3_skinv0':1016L}

try:
	config_path=os.environ['M3_ROBOT']+'/robot_config/eeprom/'
	try:
		print 'Product code'
		print '--------------------'
		for k in m3_product_codes:
			print k,' : ',m3_product_codes[k]
		code=int(raw_input())
		if (code==m3_product_codes['m3_echub']):
			from m3.m3_echub_eep import slave_cfg_dicts
			fpfx='echub'
		if (code==m3_product_codes['m3_echub4']):
			from m3.m3_1100hub_eep import slave_cfg_dicts
			fpfx='echub'
		if (code==m3_product_codes['m3_pwr']):
			from  m3.m3_pwr_eep import slave_cfg_dicts
			fpfx='pwr'
		if (code==m3_product_codes['m3_loadx6']):
			from  m3.m3_loadx6_eep  import slave_cfg_dicts
			fpfx='loadx6'
		if (code==m3_product_codes['m3_ledx2']):
			from  m3.m3_ledx2_eep  import slave_cfg_dicts
			fpfx='ledx2'
		if (code==m3_product_codes['m3_bld']):
			from  m3.m3_bld_eep  import slave_cfg_dicts
			fpfx='bld'
		if (code==m3_product_codes['m3_actx1']):
			from  m3.m3_actx1_eep  import slave_cfg_dicts
			fpfx='actx1'
		if (code==m3_product_codes['m3_actx2']):
			from  m3.m3_actx2_eep  import slave_cfg_dicts
			fpfx='actx2'
		if (code==m3_product_codes['m3_actx3']):
			from  m3.m3_actx3_eep  import slave_cfg_dicts
			fpfx='actx3'
		if (code==m3_product_codes['m3_actx4']):
			from  m3.m3_actx4_eep  import slave_cfg_dicts
			fpfx='actx4'
		if (code==m3_product_codes['m3_tactx2']):
			from  m3.m3_tactx2_eep  import slave_cfg_dicts
			fpfx='tactx2'
		if (code==m3_product_codes['m3_skinv0']):
			from  m3.m3_skin0_eep  import slave_cfg_dicts
			fpfx='tactx2'
		'''if (code==m3_product_codes['m3_armh']):
			from  m3.m3_armh_eep  import slave_cfg_dicts
			fpfx='armh'''''
			
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




