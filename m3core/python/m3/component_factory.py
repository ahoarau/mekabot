#! /usr/bin/python
# -*- coding: utf-8 -*-

import yaml
import os 
import m3.toolbox_core as m3t
import m3.monitor

component_map={
    'm3monitor': m3.monitor.M3Monitor
    }
    
m3_product_codes={}

m3_fpfx={}

m3_eeprom_cfgs={}

success = True

try:
    #f=file('/home/meka/mekabot/m3sim-1.3/robot_config/m3_component_py.yml','r')
    f=file(m3t.get_m3_config_path() + 'm3_config.yml','r')    
    config = yaml.safe_load(f.read())
    
except (IOError, EOFError):
    #print 'Config file not present:'#,self.config_name
    success = False

if success:
    if config['factory_py_libs'] is not None:
        for k in config['factory_py_libs']:
            execfile(k)
    else:
	print 'WARNING: No Python factory libraries configured in m3_config.yml'

def create_component(name):
    """This is a useful utility for creating components based
    on the name only. The m3_config.yml file maps component names
    to types. This is used to figure out the type and instantiate
    a new component class"""
    ttype=m3t.get_component_config_type(name)
    if ttype=='':
        print 'Component Factory type not found for component',name
        return None
    if not component_map.has_key(ttype):
        print 'Component Factory type ',ttype, 'not found in component_map for',name
        return None
    return component_map[ttype](name)