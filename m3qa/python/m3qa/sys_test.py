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

import yaml
import os
import subprocess as sub
from m3.toolbox import *
from m3.component import M3Component
import m3.component_factory as m3f
import m3.m3rt_toolbox as m3t
import m3.toolbox as m3tt
import time
import re
from time import strftime


class M3SysTest():
   
    def __init__(self):
        try:
            self.name=m3tt.get_m3_config()['sys_test']
        except KeyError:
            print 'Key sys_test not defined in m3_config.yml. Exiting...'
            exit()
        path=m3tt.get_m3_config_path()
        filename= path+'sys/m3_sys_test_'+self.name+'.yml'
        try:
            f=file(filename,'r')
            self.config= yaml.safe_load(f.read())
            f.close()
        except IOError:
            print 'File ',filename,'not found. Exiting...'
            exit()
        self.log=[]
        
        #load included
        self.config_include={'not_val':[],'in_range':[]}
        if self.config.has_key('include'):
            for fi in self.config['include']:
                filename= path+'sys/m3_sys_test_'+fi+'.yml'
                try:
                    f=file(filename,'r')
                    print 'Including: ',filename
                    c= yaml.safe_load(f.read())
                    f.close()
                    if c.has_key('not_val'):
                        self.config_include['not_val'].append(c['not_val'])
                    if c.has_key('in_range'):
                        self.config_include['in_range'].append(c['in_range'])
                except IOError:
                    print 'File ',filename,'not found. Exiting...'
                    exit()
            
    def write_log(self):
        fn=get_m3_log_path()+'m3_sys_test_'+self.name+'_'+strftime("%Y-%m-%d_%H:%M:%S")+'.yaml'
        f=file(fn,'w')
        print 'Writing log...',fn
        print 'Check logfile for detailed results'
        f.write(yaml.safe_dump(self.log, default_flow_style=False,width=200))
        f.close()
        

    def test_dropped_packets(self):
        err_cnt=0
        err_msg=[]
        print 'Please wait 10 seconds, testing for dropped packets:'       
        cmd = 'sudo dmesg -c'
        stdout_handle = os.popen(cmd, "r")
        output = stdout_handle.read()        
        time.sleep(10)
        stdout_handle = os.popen(cmd, "r")
        output = stdout_handle.read()        
        match = re.search('EtherCAT',output)
        if match != None:
            err = 'Error, EtherCAT messages found in dmesg:'
            res={'Test':'PacketDrop','Error':True, 'Msg':err + output}
            err_msg.append('Test: PackeDrop EtherCAT: FAIL')
            print err_msg[-1]
            self.log.append(err)
            err_cnt=err_cnt+1
        else:
            self.log.append({'Test':'PacketDrop','Error':False, 'Msg':'No Ethercat messages in dmesg' + output})
            print 'Test: PacketDrop EtherCAT: Pass'
        match = re.search('M3',output)
        if match != None:
            err = 'Error, M3 INFO messages found in dmesg:'
            res={'Test':'PacketDrop','Error':True, 'Msg':err + output}
            self.log.append(res)
            err_msg.append('Test: PacketDrop M3_INO: Fail')
            print err_msg[-1]
            err_cnt=err_cnt+1
        else:
            self.log.append({'Test':'PacketDrop','Error':False, 'Msg':'No Ethercat messages in dmesg' + output})
            print 'Test: PacketDrop M3_INFO: Pass'
        return err_cnt,err_msg
    
    def test_slaves_present(self, num_slaves):
        #print 'Checking slaves on Ecat bus..'
        err_cnt=0
        err_msg=[]
        if m3t.get_num_slaves() != num_slaves:
            err = 'Error, slaves should be :' + str(num_slaves) + ' but ' + str(m3t.get_num_slaves()) + ' found.'
            res={'Test':'NumSlaves','Error':True, 'Msg':err}
            err_msg.append('Test: NumSlaves: FAIL: \t'+err)
            print err_msg[-1]
            err_cnt=err_cnt+1
            self.log.append(res)
        else:
            err = 'Slaves should be :' + str(num_slaves) + ' and ' + str(m3t.get_num_slaves()) + ' found.'
            res={'Test':'NumSlaves','Error':False, 'Msg':err}
            print 'Test: NumSlaves: Pass'
            self.log.append(res)
        if m3t.get_num_slaves_op()+m3t.get_num_slaves_preop() != m3t.get_num_slaves():
            err = str(m3t.get_num_slaves()) + ' not equal to slaves in (PRE)OP: ' + str(m3t.get_num_slaves_op())
            res={'Test':'Slaves (PRE)OP','Error':True, 'Msg':err}
            err_msg.append('Test: Slaves(PRE)OP: FAIL: \t'+err)
            print err_msg[-1]
            err_cnt=err_cnt+1
            self.log.append(res)
        else:
            err = 'Slaves found: ' + str(m3t.get_num_slaves()) + ' is equal to slaves in (PRE)OP: ' + str(m3t.get_num_slaves_op()+m3t.get_num_slaves_preop())
            res={'Test':'Slaves (PRE)OP','Error':False, 'Msg':err}
            print 'Test: Slaves (PRE)OP: Pass'
            self.log.append(res)
        return err_cnt,err_msg
    
    def test_sensor_in_range(self, config, proxy):
        err_cnt=0
        err_msg=[]
        for k in config:
            #print 'Starting', k, '..'
            for j in range(len(config[k])):
                method = config[k][j]['method']
                mmin = config[k][j]['min']
                mmax = config[k][j]['max']
                nsteps = config[k][j]['nsteps']
                comp = m3f.create_component(k)
                proxy.subscribe_status(comp,verbose=False)
                proxy.publish_param(comp)
                proxy.step()
                fp=True
                for i in range(nsteps):
                    error = False
                    proxy.step()
                    fcn = getattr(comp,method)
                    value = fcn()
                    if value < mmin or value > mmax:
                        error = True
                        msg=comp.name + ' ' + method + ' val: ' + str(value) + ' min: ' + str(mmin) + ' max: ' + str(mmax) + ' ' + str(i)
                        res={'Test':'SensorInRange','Error':True,'Msg':msg}
                        if fp:
                            m= 'Test: SensorInRange: FAIL: \t'+comp.name+' : \t'+method+' : \t'+"%3.2f" % value
                            err_msg.append(m)
                            print err_msg[-1]
                            err_cnt=err_cnt+1
                            fp=False
                    else:
                        msg=comp.name + ' ' + method + ' val: ' + str(value) + ' min: ' + str(mmin) + ' max: ' + str(mmax) + ' ' + str(i)
                        res={'Test':'SensorInRange','Error':False,'Msg':msg}
                        if fp:
                            print 'Test: SensorInRange: Pass: \t',comp.name,' : \t',method,' : \t',"%3.2f" % value
                            fp=False
                    self.log.append(res)
        return err_cnt,err_msg
    def test_sensor_not_val(self, config, proxy):
        err_cnt=0
        err_msg=[]
        for k in config:
            #print 'Starting', k, '..'
            for j in range(len(config[k])):
                method = config[k][j]['method']
                v = config[k][j]['val']
                nsteps = config[k][j]['nsteps']
                comp = m3f.create_component(k)
                proxy.subscribe_status(comp,verbose=False)
                proxy.publish_param(comp)	
                proxy.step()
                fp=True
                for i in range(nsteps):
                    error = False
                    proxy.step()
                    fcn = getattr(comp,method)
                    value = fcn()
                    if value ==v:
                        error = True
                        msg=comp.name + ' ' + method + ' val: ' + str(value) 
                        res={'Test':'SensorNotVal','Error':True,'Msg':msg}
                        if fp:
                            m= 'Test: SensorNotVal: FAIL: \t'+comp.name+' : \t'+method+' : \t'+"%3.2f" % value
                            err_msg.append(m)
                            print err_msg[-1]
                            err_cnt=err_cnt+1
                            fp=False
                    else:
                        msg=comp.name + ' ' + method + ' val: ' + str(value)
                        res={'Test':'SensorNotVal','Error':False,'Msg':msg}
                        if fp:
                            print 'Test: SensorNotVal: Pass: \t',comp.name+' : \t'+method+' : \t'+"%3.2f" % value
                            fp=False
                    self.log.append(res)
        return err_cnt,err_msg
