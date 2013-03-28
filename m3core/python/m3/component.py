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
import m3.toolbox_core as m3t
import m3.component_base_pb2 as mbs


class M3Component:
    """Base class for all Python M3 components.
    The component's name is always prefixed by its type.
    For example: m3actutaor_ma0_j0: type_limbId_jointId
    This convention is used by the M3RtProxy to negotiate data transfer 
    and to locate the component config file.
    """
    def __init__(self,name,type):
        self.name=name
        self.config_name=m3t.get_component_config_filename(name)
        self.config=None
        self.type=type
        self.status=None
        self.command=None
        self.param=None


    def read_config(self):
        """Assumes that the config file is shared between the 
	server and proxy (NFS, etc)"""
        try:
            f=file(self.config_name,'r')
            self.config= yaml.safe_load(f.read())
        except (IOError, EOFError):
            print 'Config file not present:',self.config_name
            return
        if self.config.has_key('param'):
            self.load_attr_from_config(self.config['param'],self.param)

    def write_config(self):
        """Assumes that the config file is shared between the 
	server and proxy (NFS, etc)"""
        if self.config.has_key('param'):
            self.load_config_from_attr(self.config['param'],self.param)
        try:
            f=file(self.config_name,'w')
            print 'Saving...',self.config_name
            f.write(yaml.safe_dump(self.config, default_flow_style=False,width=200))
            f.close()
        except (IOError, EOFError):
            print 'Config file not present:',self.config_file
            return

    def load_attr_from_config(self,config, obj):
        """Load message fields from config dictionary"""
        for k in config:
            if hasattr(obj,k):
		if type(config[k])==list: #repeated field
                    attr=getattr(obj,k)
                    for i in range(len(config[k])):
                        if i<len(attr):
                            attr[i]=config[k][i]
                        else:
                            attr.append(config[k][i])
		elif type(config[k])==dict: #Nested structure
		    self.load_attr_from_config(config[k],getattr(obj,k))
                else:
                    setattr(obj,k,config[k])

		    
    def load_config_from_attr(self,config,obj):
	"""Load config dictionary from message fields"""
	for k in config:
	    if hasattr(obj,k):
		    a=getattr(obj,k)
		    if  hasattr(a,'__setitem__'): #repeated buf:
			    config[k]=list(a)
		    elif hasattr(a,'__getitem__'): # type google.protobuf.reflection.RepeatedCompositeFieldContainer
			    for i in range(len(a)):
				    self.load_config_from_attr(config[k][i],a[i])        
		    elif hasattr(a,'HasField'): #A protobuf struct
			    self.load_config_from_attr(config[k],a)
		    else:
			    config[k]=a	
					

    def update_status(self):
        """Called after every status msg recvd from proxy. Child may override if special processing required."""
        pass 

    def load_command(self):
        """Called before every command msg sent from proxy. Child may override if special processing required."""
        pass 

    def load_param(self):
        """Called before every param msg sent from proxy. Child may override if special processing required."""
        pass 

    def pretty_print_command(self):
        print self.command
    def pretty_print_status(self):
        print self.status
    def pretty_print_param(self):
        print self.param

        # ############################## Private methods ########################################

    def set_int_array(self,attr,val,ind):
        if ind is None:
            ind=range(len(attr))
        if hasattr(ind,'__getitem__'): #indice list given
            if type(val)==float or type(val)==int:
                for i in ind:
                    attr[i]=int(val)
            else:
                j = 0
                for i in ind:
                    attr[i]=int(val[j])
                    j+=1
        else: #single index given
            attr[int(ind)]=int(val)

    def set_float_array(self,attr,val,ind):
        if ind is None:
            ind=range(len(attr))
        if hasattr(ind,'__getitem__'): #indice list given
            if type(val)==float or type(val)==int:
                for i in ind:
                    attr[i]=float(val)
            else:
                j = 0
                for i in ind:
                    attr[i]=float(val[j])
                    j+=1
        else: #single index given
            attr[int(ind)]=float(val)
            
    