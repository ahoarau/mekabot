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
#import matplotlib
#matplotlib.use('TkAgg')
import pylab

import os
import sys
import time
import exceptions
import yaml
import Gnuplot
import numpy as nu
#import Numeric
import glob
import datetime
from datetime import timedelta
from m3.unit_conversion import *
from threading import Thread
import m3.component_base_pb2 as mbs
#import roslib; roslib.load_manifest('m3_toolbox_ros')
#import rospy
#import rosbag
from m3.toolbox_core import *


def get_m3_animation_path():
        try:
                return os.environ['M3_ROBOT']+'/animations/'
        except KeyError:
                print 'SET YOUR M3_ROBOT ENVIRONMENT VARIABLE'
        return ''


def get_omnibase_pwr_component_name(name):
        config= None	
        try:
                f=file(get_component_config_filename(name),'r')
                config= yaml.safe_load(f.read())
        except (IOError, EOFError):
                print 'Config file not present:',get_component_config_filename(name),'for',name
                return
        jnt_array_name=config['joint_array_component']
        return get_chain_pwr_component_name(jnt_array_name)

def get_chain_pwr_component_name(name):		
        names=get_chain_joint_names(name)
        if len(names)>0:
                return get_joint_pwr_component_name(names[0])
        else:
                return ''

def get_joint_pwr_component_name(name):
        return get_actuator_pwr_component_name(get_joint_actuator_component_name(name))

def get_actuator_pwr_component_name(name):
        return get_actuator_ec_pwr_component_name(get_actuator_ec_component_name(name))

def get_actuator_ec_pwr_component_name(name):
        config= None	
        try:
                f=file(get_component_config_filename(name),'r')
                config= yaml.safe_load(f.read())
        except (IOError, EOFError):
                print 'Config file not present:',get_component_config_filename(name),'for',name
                return
        return config['pwr_component']

def get_joint_actuator_component_name(name):	
        if get_component_config_type(name) == 'm3joint_slave':
                return ""
        config= None	
        try:
                f=file(get_component_config_filename(name),'r')
                config= yaml.safe_load(f.read())
        except (IOError, EOFError):
                print 'Config file not present:',get_component_config_filename(name),'for',name
                return
        return config['actuator_component']
        
def get_joint_ctrl_component_name(name):	
        if get_component_config_type(name) == 'm3joint_slave':
                return ""
        config= None	
        try:
                f=file(get_component_config_filename(name),'r')
                config= yaml.safe_load(f.read())
        except:
                print 'Config file not present:',get_component_config_filename(name),'for',name
                return ""
        ctrl = ""
        try:
		ctrl = config['control_component']
	except:
		ctrl = ""
        return ctrl


def get_chain_dynamatics_component_name(name):		
        config= None	
        try:
                f=file(get_component_config_filename(name),'r')
                config= yaml.safe_load(f.read())
        except (IOError, EOFError):
                print 'Config file not present:',get_component_config_filename(name),'for',name
                return

        if config.has_key('dynamatics_component'):
                return config['dynamatics_component']
        else:
                return ''

def get_chain_loadx6_name(chain_name):
        if (chain_name.find('arm')==-1):
                print 'Error: get_chain_loadx6_name() valid for arms only'
                return None
        joint_names=get_chain_joint_names(chain_name)
        j0=joint_names[0]
        return 'm3loadx6_'+j0[j0.find('_')+1:j0.find('_',j0.find('_')+1)]+'_l0'

def get_chain_limb_name(name):		
        config= None	
        try:
                f=file(get_component_config_filename(name),'r')
                config= yaml.safe_load(f.read())
        except (IOError, EOFError):
                print 'Config file not present:',get_component_config_filename(name),'for',name
                return
        if config.has_key('limb_name'):
                return config['limb_name']
        else:
                return None

def get_chain_joint_names(name):		
        config= None	
        try:
                f=file(get_component_config_filename(name),'r')
                config= yaml.safe_load(f.read())
        except (IOError, EOFError):
                print 'Config file not present:',get_component_config_filename(name),'for',name
                return
        if config.has_key('joint_components'):
                #Have to sort keys by *_JID*'
                ret=[]
                for jid in range(len(config['joint_components'])):
                        ret.append(config['joint_components']['J'+str(jid)])
                return ret
        else:
                return []

def get_chain_joint_limits(name):
        l=[]
        names=get_chain_joint_names(name)
        for n in names:
                try:
                        f=file(get_component_config_filename(n),'r')
                        config= yaml.safe_load(f.read())
                        l.append([config['param']['min_q'],config['param']['max_q']])
                except (IOError, EOFError):
                        print 'Config file not present:',get_component_config_filename(name),'for',name
                        return []
        return l

def get_actuator_ec_component_name(name):
        config= get_component_config(name)	
        if config is not None:
                return config['ec_component']
        else:
                return None

def get_joint_chain_name(name):
        path=get_m3_config_path()
        filename= path+'m3_config.yml'
        f=file(filename,'r')
        config= yaml.safe_load(f.read())

        for cdir in config['rt_components'].keys():
                for c in config['rt_components'][cdir].keys():
                        if (c==name):
                                for d in config['rt_components'][cdir].keys():
                                        if config['rt_components'][cdir][d] == 'm3arm' or \
                                           config['rt_components'][cdir][d] == 'm3torso' or \
                                           config['rt_components'][cdir][d] == 'm3hand' or \
                                           config['rt_components'][cdir][d] == 'm3head':
                                                return d
        return ''

def get_robot_name():
        path=get_m3_config_path()
        filename= path+'m3_config.yml'
        f=file(filename,'r')
        config= yaml.safe_load(f.read())

        for cdir in config['rt_components'].keys():
                for c in config['rt_components'][cdir].keys():
                        if config['rt_components'][cdir][c] == 'm3humanoid' or config['rt_components'][cdir][c] == 'm3humanoid_sea':
                                return c
        return ''
        return ''

"""
* Animation file reader for files provided by Andrea Thomaz
* each line is a trajectory via point: ts q0 q1 ... qn qd0 qd1 ... qdn qdd0 qdd1 ... qddn
* n dof (7 each arm + torso: n=17) (define joint ordering in the comments)
* ts: timestamp (s)
* qx: joint angle of dof x (rad)
* qdx: joint velocity of dof x (rad/s)
* qddx: joint accel of dof x (rad/s2)

Note: the joint angles seem scale by a factor of 0.5, and the velocities are all >0
       we'll just use the joint angles anyhow
"""

def get_animation_files():
        path=get_m3_animation_path()
        return glob.glob(path+'*.txt')

def get_animation_names():
        path=get_m3_animation_path()
        full=glob.glob(path+'*.txt')
        names=[]
        for s in full:
                names.append(s[s.rfind('/')+1:s.rfind('.')])
        return names

def load_animation(filename):
        f=open(filename,'rb')
        data=f.read()
        nl=data.find('\r\n')
        samples=[]
        while nl>=0:
                sample=[]
                line=data[:nl]
                ts=line.find('\t')
                while ts>=0:
                        sample.append(float(line[:ts]))
                        line=line[ts+1:]
                        ts=line.find('\t')
                sample.append(float(line))
                samples.append(sample)
                data=data[nl+1:]
                nl=data.find('\r\n')
        x=nu.array(samples)

        #limb names should map to chain limb names
        animation={'timestamp':x[:,0],
                   'torso':{    'theta':x[:,1:4],'thetadot':x[:,18:21],'thetadotdot':x[:,35:38]},
                   'left_arm':{'theta':-2*x[:,4:11], 'thetadot':x[:,21:28],'thetadotdot':x[:,38:45]},
                   'right_arm':{'theta':-2*x[:,11:18], 'thetadot':x[:,28:35],'thetadotdot':x[:,45:52]}}

        ##patch animation to work with meka bodies
        animation['torso']['theta'][:,1:3]=animation['torso']['theta'][:,1:2]*-1
        animation['torso']['thetadot'][:,1:3]=animation['torso']['thetadot'][:,1:2]*-1
        animation['torso']['thetadotdot'][:,1:3]=animation['torso']['thetadotdot'][:,1:2]*-1

        animation['right_arm']['theta'][:,2]=animation['right_arm']['theta'][:,2]*-1
        animation['right_arm']['thetadot'][:,2]=animation['right_arm']['thetadot'][:,2]*-1
        animation['right_arm']['thetadotdot'][:,2]=animation['right_arm']['thetadotdot'][:,2]*-1

        animation['left_arm']['theta'][:,2]=animation['left_arm']['theta'][:,2]*-1
        animation['left_arm']['thetadot'][:,2]=animation['left_arm']['thetadot'][:,2]*-1
        animation['left_arm']['thetadotdot'][:,2]=animation['left_arm']['thetadotdot'][:,2]*-1

        animation['left_arm']['theta'][:,1]=animation['left_arm']['theta'][:,1]*-1
        animation['left_arm']['thetadot'][:,1]=animation['left_arm']['thetadot'][:,1]*-1
        animation['left_arm']['thetadotdot'][:,1]=animation['left_arm']['thetadotdot'][:,1]*-1

        #animation['left_arm']['theta'][:,0]=animation['left_arm']['theta'][:,0]*-1
        #animation['left_arm']['thetadot'][:,0]=animation['left_arm']['thetadot'][:,0]*-1
        #animation['left_arm']['thetadotdot'][:,0]=animation['left_arm']['thetadotdot'][:,0]*-1
        return animation

def get_via_files():
        path=get_m3_animation_path()
        return glob.glob(path+'*.via')

def get_via_names():
        path=get_m3_animation_path()
        full=glob.glob(path+'*.via')
        names=[]
        for s in full:
                names.append(s[s.rfind('/')+1:s.rfind('.')])
        return names
