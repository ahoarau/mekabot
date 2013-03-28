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

# ###########################################

class M3Exception(exceptions.Exception):
        def __init__(self, value):
                self.value = value
        def __str__(self):
                return repr(self.value)


# ###########################################
class M3KeyStrokeThread(Thread):
        def __init__ (self,menu,req_enter=False):
                Thread.__init__(self)
                self.done=False
                self.menu=menu
                self.value=''
                self.req_enter=req_enter
        def start(self):
                Thread.start(self)
        def stop(self):
                self.done=True
        def run(self):
                while not self.done:
                        for m in self.menu:
                                print m
                        if self.req_enter:
                                self.value=raw_input()
                                if len(self.value)>1:
                                        self.x[:1]
                        else:
                                self.value=sys.stdin.read(1)#get_keystroke()
                        if self.value=='q':
                                self.done=True

# ###########################################	
def load_m3ec_kmod():
        os.system('sudo m3rt_insmods') 


def unload_m3ec_kmod():
        os.system('sudo m3rt_rmmods' )


def get_m3_lib_path():
        cmd='echo /lib/modules/$(uname -r)/m3/'
        stdout_handle = os.popen(cmd, "r")
        s = stdout_handle.read()
        s=s[:-1]
        return s

def load_m3ec_module():
        print 'Loading M3EC Kernel Module'
        cmd='sudo /sbin/insmod '+get_m3_lib_path()+'m3ec.ko'
        os.system(cmd) 

def unload_m3ec_module():
        print 'Unloading M3EC Kernel Module'
        cmd='sudo /sbin/rmmod m3ec.ko'
        os.system(cmd)

def open_echub_port3():
        print 'Opening ECHUB Port3 on Slave 0'
        cmd='echo -ne \\00 | ethercat phy_write -v -p0 0x101 -'
        os.system(cmd)

def get_keystroke():
        os.system("stty raw")
        r = sys.stdin.read(1)
        os.system("stty sane")
        return r

def get_string(default=''):
        x=raw_input()
        if x=='':
                return default
        return x

def get_yes_no(default=None):
        while True:
                x=raw_input()
                if x=='':
                        x=default
                if x=='y' or x=='Y':
                        return True
                if x=='n' or x=='N':
                        return False
                print 'Invalid value:(y/n) required'

def get_float(default=None):
        while True:
                try:
                        x=float(input())
                        break
                except (NameError, SyntaxError,TypeError):
                        if default is not None:
                                return default
                        print 'Invalid value, try again'
        return x

def get_int(default=None):
        while True:
                try:
                        x=int(input())
                        break
                except (NameError, SyntaxError,TypeError):
                        if default is not None:
                                return default
                        print 'Invalid value, try again'
        return x

def get_m3_ros_config_path():
        try:
                return os.environ['M3_ROBOT']+'/ros_config/'
        except KeyError:
                print 'SET YOUR M3_ROBOT ENVIRONMENT VARIABLE'
        return ''

def get_m3_config_path():
        try:
                return os.environ['M3_ROBOT']+'/robot_config/'
        except KeyError:
                print 'SET YOUR M3_ROBOT ENVIRONMENT VARIABLE'
        return ''

def get_m3_log_path():
        try:
                return os.environ['M3_ROBOT']+'/robot_log/'
        except KeyError:
                print 'SET YOUR M3_ROBOT ENVIRONMENT VARIABLE'
        return ''

def get_config_hostname():
        path=get_m3_config_path()
        filename= path+'m3_config.yml'
        f=file(filename,'r')
        config= yaml.safe_load(f.read())
        f.close()
        try:
                h = config['hostname']
        except KeyError:
                h = None
        return h


def get_local_hostname():
	cmd='hostname'
	stdout_handle = os.popen(cmd, "r")
	s = stdout_handle.read()
	return s[:-1]


def get_component_config(name):
        config= None	
        try:
                f=file(get_component_config_filename(name),'r')
                config= yaml.safe_load(f.read())
        except (IOError, EOFError):
                print 'Config file not present:',get_component_config_filename(name),'for',name
        return config

def get_m3_config():
        path=get_m3_config_path()
        filename= path+'m3_config.yml'
        f=file(filename,'r')
        config= yaml.safe_load(f.read())
        return config


def get_ec_component_names():
        c=get_m3_config()
        x=[]
        if c.has_key('ec_components'):
                for k in c['ec_components'].keys():
                        x.extend(c['ec_components'][k].keys())
        return x

def get_component_config_type(name):
        path=get_m3_config_path()
        filename= path+'m3_config.yml'
        f=file(filename,'r')
        config= yaml.safe_load(f.read())
        try:
                for cdir in config['ec_components'].keys():
                        for c in config['ec_components'][cdir].keys():
                                if (c==name):
                                        return config['ec_components'][cdir][c]
        except KeyError:
                pass
        try:
                for cdir in config['rt_components'].keys():
                        for c in config['rt_components'][cdir].keys():
                                if (c==name):
                                        return config['rt_components'][cdir][c]
        except KeyError:
                pass
        return ''


def get_component_config_filename(name):

        path=get_m3_config_path()
        filename= path+'m3_config.yml'
        f=file(filename,'r')
        config= yaml.safe_load(f.read())
        try:
                for cdir in config['ec_components'].keys():
                        for c in config['ec_components'][cdir].keys():
                                if (c==name):
                                        return path+cdir+'/'+name+'.yml'
        except KeyError:
                pass
        try:
                for cdir in config['rt_components'].keys():
                        for c in config['rt_components'][cdir].keys():
                                if (c==name):
                                        return path+cdir+'/'+name+'.yml'
        except KeyError:
                pass
        return ''

def get_component_config_path(name):
        path=get_m3_config_path()
        filename= path+'m3_config.yml'
        f=file(filename,'r')
        config= yaml.safe_load(f.read())
        try:
                for cdir in config['ec_components'].keys():
                        for c in config['ec_components'][cdir].keys():
                                if (c==name):
                                        return path+cdir+'/'
        except KeyError:
                pass
        try:
                for cdir in config['rt_components'].keys():
                        for c in config['rt_components'][cdir].keys():
                                if (c==name):
                                        return path+cdir+'/'
        except KeyError:
                pass
        return ''

def time_string():
        time_stamp = time.localtime()
        output="_".join([('0'*(2-len(str(i)))+str(i)) for i in time_stamp[:6]])
        return output

def timestamp_string(ts):
        """ Print human readable date from timestamp in ns"""
        year=timedelta(days=365.25)
        thirty_years=30*year
        print  datetime.datetime.fromtimestamp(ts*1e-9)+thirty_years

# Build a map like:
# [ (0,'status.pdo.adc_torque')
#   (1,'status.base.timestamp'),...]
# Allows an index selection of msg fields
def GetIdMapDictFromMsg(msg=None,d=None,idx=0,root=''):
        if msg is not None:
                d=GetDictFromMsg(msg)
        map=[]
        for k  in d.keys():
                v=d[k]
                if type(v)==dict:
                        id.extend(GetIdMapDictFromMsg(msg=None,d=v,idx=idx,root=root+'.'+k))
                else:
                        id.append((idx,root+'.'+k))
                        idx=idx+1
        return map

#return a list: ['a','b','c.d',...]
#exclude ['fielda','fieldb',...] : exclude certain fields
def get_msg_fields(msg,prefix='',exclude=None):
        ret=[]
        fields=msg.DESCRIPTOR.fields_by_name.keys()
        for f in fields:
		if exclude==None or len([x for x in exclude if f.find(x)>=0])==0:
			v=getattr(msg,f)
			if type(v)==long or type(v)==float or type(v)==int or type(v)==bool or type(v)==str or hasattr(v,'__len__'):
				ret.append(prefix+f)
			elif hasattr(v,'__class__'):
				ret=ret+get_msg_fields(v,prefix+f+'.',exclude)
	return ret

	
def user_select_msg_field(msg):
        name=''
        fields=msg.DESCRIPTOR.fields_by_name.keys()
        print '---------------'
        for i in range(len(fields)):
                print i,' ; ',fields[i]
        print '---------------'
        print 'Enter Field ID'
        id=get_int()
        name=fields[id]
        v=getattr(msg,fields[id])

        if type(v)==float or type(v)==int or type(v)==bool or type(v)==str:
                return name
        if hasattr(v,'__len__'):
                return name
        if hasattr(v,'__class__'):
                name=name+'.'+user_select_msg_field(v)
        return name

def get_msg_field_value(msg,field):
        dot=field.find('.')
        if dot==-1:
                return getattr(msg,field)
        return get_msg_field_value(getattr(msg,field[:dot]),field[dot+1:])

def GetDictFromMsg(msg):
        d={}
        fields=msg.DESCRIPTOR.fields_by_name.keys() 
        for attr in fields:
                v=getattr(msg,attr)
                if type(v)==float or type(v)==int or type(v)==bool or type(v)==str or type(v)==long or type(v)==unicode:
                        d[attr]=v
                elif hasattr(v,'__setitem__'): #list type
                        d[attr]=list(v)
                elif hasattr(v,'__getitem__'): # type google.protobuf.reflection.RepeatedCompositeFieldContainer
                        d[attr]={}
                        for x in range(len(v)):
                                d[attr][x]=GetDictFromMsg(v[x])
                elif hasattr(v,'__class__'):
                        d[attr]=GetDictFromMsg(v)
        return d

def SetMsgFromDict(msg,d):
        for k in d.keys():
                if type(d[k])==float or type(d[k])==int or type(d[k])==bool or type(d[k])==str or type(d[k])==long:
                        
                        t=type(getattr(msg,k))
                        setattr(msg,k,t(d[k]))
                elif type(d[k])==type([]):
                        attr=getattr(msg,k)
                        for i in range(len(d[k])):
                                t=type(attr[i])
                                attr[i]=t(d[k][i])
                else:
                        if hasattr(msg,'__getitem__'): #array
                                v=msg[k]
                        else:
                                v=getattr(msg,k)
                        SetMsgFromDict(v,d[k])

def float_list(v):
        return [float(x) for x in v]

def int_list(v):
        return [int(x) for x in v]

def gplot(x,y=None,g=None,yrange=None,persist_in=1):
        if y is None: 
                y=range(len(x))
        if g is None:
                g = Gnuplot.Gnuplot(persist = persist_in)
                g.title('M3 Plot')
                g('set data style lines')
                g('set term x11 noraise')
                if yrange is not None:
                        g('set yrange ['+str(yrange[0])+':'+str(yrange[1])+']')
        g.plot(zip(y,x))
        return g

def gplot2(x1,x2,y=None,g=None,yrange=None,persist_in=1):
        if y is None: 
                y=range(len(x1))
        if g is None:
                g = Gnuplot.Gnuplot(persist = persist_in)
                g.title('M3 Plot')
                g('set data style lines')
                g('set term x11 noraise')
                if yrange is not None:
                        g('set yrange ['+str(yrange[0])+':'+str(yrange[1])+']')
        d = Gnuplot.Data(y,x1)
        d2 = Gnuplot.Data(y,x2)
        g.plot(d,d2)
        return g

def mplot(x,y,xlabel='X',ylabel='Y',save_file=None):
        pylab.plot(y,x)
        pylab.xlabel(xlabel)
        pylab.ylabel(ylabel)
        pylab.title('M3 Plot')
        pylab.grid(True)
        if save_file is not None:
                pylab.savefig(save_file)
        pylab.show()


def mplot2(x,y1,y2,xlabel='X',ylabel='Y',y1name='y1',y2name='y2'):
        pylab.plot(x,y1,'k-')
        pylab.plot(x,y2,'g-')
        pylab.legend([y1name,y2name])
        pylab.xlabel(xlabel)
        pylab.ylabel(ylabel)
        pylab.title('M3 Plot')
        pylab.grid(True)
        pylab.show()

def mplot3(x,y1,y2,y3,xlabel='X',ylabel='Y',y1name='y1',y2name='y2',y3name='y3'):
        pylab.plot(x,y1,'k-')
        pylab.plot(x,y2,'g-')
        pylab.plot(x,y3,'r-')
        pylab.legend([y1name,y2name,y3name])
        pylab.xlabel(xlabel)
        pylab.ylabel(ylabel)
        pylab.title('M3 Plot')
        pylab.grid(True)
        pylab.show()

def mplot4(x,y1,y2,y3,y4,xlabel='X',ylabel='Y',y1name='y1',y2name='y2',y3name='y3',y4name='y4'):
        pylab.plot(x,y1)
        pylab.plot(x,y2)
        pylab.plot(x,y3)
        pylab.plot(x,y4)
        pylab.legend([y1name,y2name,y3name,y4name])
        pylab.xlabel(xlabel)
        pylab.ylabel(ylabel)
        pylab.title('M3 Plot')
        pylab.grid(True)
        pylab.show()

def mplotN(x,y,xlabel='X',ylabel='Y',yname=None,title='M3 Plot'):
        #pylab.ion()
        #pylab.clf()
        for yy in y:
                pylab.plot(x,yy)
        if yname is not None and len(yname)==len(y):
                pylab.legend(yname)
        pylab.xlabel(xlabel)
        pylab.ylabel(ylabel)
        pylab.title(title)
        pylab.grid(True)
        pylab.show()
        #pylab.draw()
        #pylab.ioff()

def user_select_components(names):
        if len(names)==0:
                return []
        print '--------------------------------------'
        print 'Number available components: ',len(names)
        for i in range(len(names)):
                print 'Component ',i,' : ',names[i]
        print '--------------------------------------'
        select=[]
        print 'Selecting components. Enter q to finish'
        while True:
                print 'Enter component id [',names[0],']'
                id=raw_input()
                if id=='':
                        return [names[0]]
                if id=='q':
                        return select
                if int(id)>=0 and int(id)<len(names):
                        select.append(names[int(id)])

def user_select_components_interactive(names,single=False,item='Components'):
        if len(names)==0:
                return []
        if single and len(names)==1:
                return names
        idx=0
        print '--------------------------------------'
        print 'Number available ',item,': ',len(names)
        for i in range(len(names)):
                print item,i,' : ',names[i]
        print '---------------------------------------------------------'
        print 'Select a component.'
        print 'f: forward , b: back , enter: select, i: index, q: quit'
        print '---------------------------------------------------------'
        select=[]
        while True:
                print idx,': [ ',names[idx],' ]'
                os.system("stty raw")
                r = sys.stdin.read(1)
                os.system("stty sane")
                print
                if r=='f':
                        idx=(idx+1)%len(names)
                elif r=='b':
                        idx=(idx-1)%len(names)
                elif r=='\r':
                        select.append(names[idx])
                        if single and len(select)==1:
                                return select
                        idx=(idx+1)%len(names)
                elif r=='q':
                        return select
                elif r=='i':
                        print 'Enter ',item, 'index: '
                        ii=get_int()
                        if ii>=0 and ii<len(names):
                                select.append(names[ii])
                                return select
                        else:
                                print 'Invalid index'
                else:
                        idx=(idx+1)%len(names)

# #############################################################################################	
#RC filter, where:
# y_k = (T/(T+h)) y_k-1 + (h/(T+h)) x_k
#Will reach 63% of final value in time_constant_s seconds
class M3ExponentialAvg:
        def __init__(self):
                self.y_k=0
                self.y_k_last=0
                self.h=0
                self.T=0
        def resize(self,sample_period_s, time_constant_s):
                """Provide the step period (seconds) and the filter time constant (seconds)"""
                self.T=time_constant_s
                self.h=sample_period_s
        def reset(self,val):
                self.y_k=val
                self.y_k_last=val
        def step(self,x_k):
                self.y_k = (self.T/(self.T+self.h))*self.y_k_last + (self.h/(self.T+self.h))*x_k
                self.y_k_last=self.y_k
                return self.y_k
# #############################################################################################	
class M3Average():
        def __init__(self,n):
                self.idx=0
                self.buf=nu.zeros(n,nu.float)
                self.n=n
        def reset(self,x):
                self.idx=0
                self.buf=nu.ones(self.n,nu.float)*x
        def step(self,x):
                self.buf[self.idx]=x
                self.idx=int((self.idx+1)%self.n)
                return nu.average(self.buf)
# #############################################################################################	
class M3Slew():
        def __init__(self):
                self.val=0.0
        def step(self,des,rate):
                if self.val<des:
                        self.val=min(des,self.val+rate)
                else:
                        self.val=max(des,self.val-rate)
                return self.val
# #############################################################################################	

class M3ScopeN():
        def __init__(self,n=12,xwidth=200,yrange=None,title='M3ScopeN'):
                self.n=n
                self.y=[]
                for i in range(n):
                        self.y.append([0.0]*xwidth)
                self.x=range(xwidth)
                self.g = Gnuplot.Gnuplot(persist = 1)                
                self.g('set data style lines')
                self.g('set term x11 noraise')
                if yrange is not None:
                        self.g('set yrange ['+str(yrange[0])+':'+str(yrange[1])+']')
        def plot(self,y):
                d=[]
                for i in range(self.n):
                        self.y[i].pop(0)
                        self.y[i].append(y[i])
                        d.append(Gnuplot.Data(self.x,self.y[i]))
                self.g.plot(*d)
# #############################################################################################	

class M3Scope():
        def __init__(self,xwidth=200,yrange=None):
                self.y=[0.0]*xwidth
                self.x=range(xwidth)
                self.g=None
                self.yrange=yrange
        def plot(self,val):
                self.y.pop(0)
                self.y.append(val)
                self.g=gplot(self.y,self.x,self.g,self.yrange)
        def print_to_file(self,filename):
                self.g.hardcopy(filename,color=1)

class M3Scope2():
        def __init__(self,xwidth=200,yrange=None):
                self.y1=[0.0]*xwidth
                self.y2=[0.0]*xwidth
                self.x=range(xwidth)
                self.g=None
                self.yrange=yrange
        def plot(self,yy1,yy2):
                self.y1.pop(0)
                self.y1.append(yy1)
                self.y2.pop(0)
                self.y2.append(yy2)
                self.g=gplot2(self.y1,self.y2,self.x,self.g,self.yrange)
        def print_to_file(self,filename):
                self.g.hardcopy(filename,color=1)
# ####################### Inplace  Operations ##########################

def DictPrint(d,space=''):
        key_names = d.keys()
        for key in key_names:
                if type(d[key])==type({}):
                        print space,key
                        DictPrint(d[key],space+'    ') #recurse on dictionary
                else:	
                        print space,key,':',d[key]

def DictSet(dest,src):
        """Src and dest should have identical structure. The structure can be nested dictionaries with values
        of either other dictionaries or of a mutable sequence. This just copies in the individual
        sequence values of each dictionary. Recursive function"""
        key_names = src.keys()
        for key in key_names:
                if type(src[key])==type({}):
                        DictSet(dest[key],src[key]) #recurse on dictionary
                else:	
                        idx=0
                        try:
                                try:
                                        for s in src[key]:
                                                dest[key][idx]=s
                                                idx=idx+1
                                except KeyError: #if  structure changes, then may get this error
                                        pass
                        except TypeError: #caught on numerical value fields
                                dest[key]=src[key]



def SeqSet(dest,src,indices=None):
        """Inplace copy. Src and dest should have identical structure. Does an itemwise copy of numarrays, lists,...
        If indices is supplied, only copies those indices"""
        if indices==None:
                indices = range(len(src))
        for idx in indices:
                dest[idx]=src[idx]

def DictListCopy(src):
        return src
#"""Make a copy of a dictionary, converting any non-list arrays to lists"""
#ret = src.copy()
#key_names = ret.keys()
#for key in key_names:
        #if type(ret[key])==type({}):
                #ret[key]=DictListCopy(ret[key]) #recurse on dictionary
        #else:
                #v=type(ret[key])
                #if type(v)==float or type(v)==int or type(v)==bool or type(v)==str or type(v)==long:
                        #ret[key]=
                #if type(ret[key]) is not [] and type(ret[key]) is not  str: #std::vector
                        #ret[key]=list(ret[key])
                #else:
                #ret[key]=ret[key][:]
#return ret


def wrap_rad(rad):        
        if type(rad) == nu.array or type(rad) == list or type(rad) == nu.ndarray:
                for i in range(len(rad)):
                        while rad[i] > nu.pi:
                                rad[i] -= 2*nu.pi
                        while rad[i] < -nu.pi:
                                rad[i] += 2*nu.pi
        else:
                while rad > nu.pi:
                        rad -= 2*nu.pi
                while rad < -nu.pi:
                        rad += 2*nu.pi
        return rad

def wrap_deg(deg):
        return wrap_rad(deg2rad(deg))

def unwrap_rad(rad, max, min):
        if type(rad) == nu.array or type(rad) == list or type(rad) == nu.ndarray:
                for i in range(len(rad)):
                        #if i == 4:
                        #        print rad[i], min[i], max[i]
                        if (rad[i] > min[i]) and (rad[i] < max[i]):
                                pass
                                #if (i == 4):
                                #    print "1"
                        elif (rad[i] + 2*nu.pi < max[i]):
                                #if (i == 4):
                                #    print "2"
                                rad[i] += 2*nu.pi
                        elif (rad[i] - 2*nu.pi > min[i]):
                                #if (i == 4):
                                #    print "3"
                                rad[i] -= 2*nu.pi
                        else: # TODO: throw error here?
                                pass
                                #if (i == 4):
                                #    print "4"
                return rad
        else:   
                if (rad > min) and (rad < max):
                        return rad
                elif (rad + 2*nu.pi < max):
                        return rad + 2*nu.pi
                elif (rad - 2*nu.pi > min):
                        return rad - 2*nu.pi
                else: #TODO: throw error here?
                        return rad


def unwrap_deg(deg, max, min):
        return unwrap_rad(deg2rad(deg), deg2rad(max), deg2rad(min))


def find_distance(array_1, array_2):
        array_diff = nu.absolute(array_1 - array_2)
        for i in range(len(array_diff)):
                while array_diff[i] >= nu.pi:
                        array_diff[i] -= nu.pi
        return nu.sqrt(nu.sum(array_diff*array_diff))

def get_count(count, max_count, min_count):
        if count > 0:
                if -count >= min_count:
                        return -count
                elif count+1 <= max_count:
                        return count + 1
                else:
                        raise M3Exception('Counter reached end of limits.')
        else:
                if 1-count <= max_count:
                        return 1 - count
                elif count - 1 >=  min_count:
                        return count - 1
                else:
                        raise M3Exception('Counter reached end of limits.')

def in_range_rad(solution_wrapped, max, min):
        valid = True
        solution = unwrap_rad(solution_wrapped, max, min)

        for i in range(len(solution)):
                #if i == 4:
                #        if solution[i] < 0:
                #                print rad2deg(solution[i]), rad2deg(solution_wrapped[i])
                #                if (solution[i] + 2*3.14 > max[i]) or (solution[i] + 3.14*2 < min[i]):
                #                        valid = False
                #        else:
                #                if (solution[i] > max[i]) or (solution[i] < min[i]):
                #                        valid = False
                #else:    
                #        if (solution[i] > max[i]) or (solution[i] < min[i]):
                #                valid = False
                if (solution[i] > max[i]) or (solution[i] < min[i]):
                        valid = False
        return valid

# ############################### LOGGING #################################################

def get_time_sorted_valid_logdirs():
        path=get_m3_log_path()
        logdirs=[ name for name in os.listdir(path) if os.path.isdir(os.path.join(path, name)) ]
        valid_logdirs=[path+xdir for xdir in logdirs if len(glob.glob(path+'/'+xdir+'/*.pb.log'))>0]
        valid_logdirs.sort(key=lambda x: os.path.getmtime(x))
        valid_logdirs.reverse()
        valid_logdirs=[x[x.rfind('/')+1:] for x in valid_logdirs]
        return valid_logdirs

def convert_loglist_to_RosBag(ll, bag_file_name, topic_name, ros_msg_type):
        bag = rosbag.Bag(bag_file_name, 'w')
        for m3_msg in ll:
                ros_msg = ros_msg_type()
                convert_m3_dict_to_ros_msg(m3_msg,ros_msg)
                bag.write(topic_name,ros_msg)
        bag.close();

def convert_m3_dict_to_ros_msg(m3_msg,ros_msg):        
        for k in m3_msg.keys():
                if type(m3_msg[k])==dict:
                        convert_m3_dict_to_ros_msg(m3_msg[k],getattr(ros_msg,k))
                else:
                        if type(m3_msg[k])==unicode:
                                setattr(ros_msg,k,str(m3_msg[k]))
                        else:
                                setattr(ros_msg,k,m3_msg[k])


# Useage
# dump_log_dir_to_dict('/home/meka/mekabot/m3qa/robot_log/foo/','m3actuator_ec_ma0j0',mec.M3ActuatorEcStatus())
# Pass in the component name of the logged component as well as a Status container to load into
# Returns a list of dictionaries. Each dictionary is one time sample from the component.
def dump_log_dir_to_loglist(logname,comp_name,status):
        print 'Loading log: ',logname
        info = get_log_info(logname)
        ret=[]
        for ip in info: #loop over all logfile pages
                log_page=mbs.M3StatusLogPage()
                f = open(ip['filename'], "rb")
                s=f.read()
                f.close()
                log_page.ParseFromString(s) #load page into protobuf class
                for status_all in log_page.entry: #loop over all entries in a page
                        for i in range(len(status_all.name)): #loop over all components in an entry
                                if status_all.name[i]==comp_name:
                                        status.ParseFromString(status_all.datum[i])
                                        d=GetDictFromMsg(status)
                                        ret.append(d)
        return ret

def get_timeseries_from_loglist(field,ll):
        ret=[]
        for s in ll:
                dot=field.find('.') #hiearchy: eg, joint.torque
                ks=s
                f=field
                while dot!=-1:
                        ks=s[f[:dot]] #sub dictionary
                        f=f[dot+1:]
                        dot=f.find('.')
                ret.append(ks[f])
        return ret

def get_log_dir(logname,logpath=None):
        if logpath is not None:
                return logpath+'/'+logname
        try:
                logpath=os.environ['M3_ROBOT']+'/robot_log'
                if not os.path.isdir(logpath):
                        print 'Directory does not exist: '+logpath
                        return None
                return logpath+'/'+logname
        except KeyError:
                print 'You must set your M3_ROBOT Environment variable to do logging.'
                return None

def get_log_info(logname,logpath=None,logdir=None):
        #logname format: /dir/dir2/.../logname/logname_xxxxxx_yyyyyy.pb.log
        if logdir is None:
                logdir=get_log_dir(logname,logpath)
        log_files=glob.glob(logdir+'/*.pb.log')
        log_files.sort()
        info=[]
        for lf in log_files:
                istart=lf[:lf.rfind('_')]	
                istart=istart[istart.rfind('_')+1:]
                if istart.isdigit():
                        istart=int(istart)
                else:
                        istart=-1
                iend=lf[lf.rfind('_')+1:lf.find('.pb.log')]
                if iend.isdigit():
                        iend=int(iend)
                else:
                        iend=-1
                info.append({'filename':lf,'start_idx':istart,'end_idx':iend})
        return info

def make_log_dir(logdir):
        if os.path.isdir(logdir):
                if len(glob.glob(logdir+'/*'))>0:
                        os.system('rm '+logdir+'/*')
                return True
        r=os.system('mkdir '+logdir)
        if r==0:
                return True
        else:
                print 'Unable to make log directory: ',logdir
                return False