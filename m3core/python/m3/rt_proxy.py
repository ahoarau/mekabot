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

import socket
import sys
import xmlrpclib
import array
import m3.component_base_pb2 as mbs
import m3.toolbox_core as m3t
import time
import os



class M3RtProxy:
	def __init__(self,host=None,rpc_port=8000):
		"""M3RtProxy is the client interface to the M3RtServer.
	It manages the state of the server using XML_RPC methods. 
	It can query the server state,
	start/stop the run-time system, create a DataService connection,
	and publish/subscribe desired components to the DataService.
	The DataService uses a faster TCP/IP socket on port 10000"""
		self.host=host
		if host is None:
			self.host=m3t.get_config_hostname()
		if self.host is None:
			self.host = m3t.get_local_hostname()
		self.rpc_port=rpc_port
		self.data_port=10000 #Currently hardcoded in M3
		self.proxy=None
		self.data_socket=None
		self.subscribed={}
		self.published_param={}
		self.published_command={}
		self.available_components=[]
		self.available_component_types=[]
		self.log_comps={}
		self.log_names=[]
		self.logname=None
		self.status_raw=mbs.M3StatusAll()
		self.command_raw=mbs.M3CommandAll()
		self.ns=0
		self.nsl=0
		self.data_svc=None
		self.ros_svc =None
		try:
			self.proxy = xmlrpclib.ServerProxy('http://'+self.host+':'+str(self.rpc_port))
			print 'Starting M3 RPC Client at ',self.host, 'on Port ',self.rpc_port,'...'
			#Check that connection made
			try:
				self.proxy.system.listMethods()
			except xmlrpclib.Error, v:
				self.proxy=None
				raise m3t.M3Exception('Error: '+v+'Make sure that the M3 RPC Server is running')
		except socket.error, msg:
			self.proxy=None
			raise m3t.M3Exception('Check that server is started. Socket Error: '+str(msg))

	# ###########################################################################################	
	def step(self):	
		"""Update the server with commands and parameters. Fetch new status data.
	This should be called periodically within the main loop"""		
		self.__send_command()
		self.__recv_status()		

	def start(self,start_data_svc=True,start_ros_svc=False):
		"""Startup the RtSystem on the server. This will load all available components
	and begin execution in state SAFEOP. It can also start a DataService"""
		self.__start_rt_system()
		self.data_svc = start_data_svc
		self.ros_svc = start_ros_svc
		
		if start_data_svc:
			self.__start_data_service()

		if start_ros_svc:
			self.__start_ros_service()

	def stop(self, force_safeop=True):
		"""Stop the RtSystem and any running data service on the server. 
	This should be called at client process shutdown
	Will automatically move all components to SAFEOP by default (for safety)"""
		if force_safeop:
			self.make_safe_operational_all()
		try:
			if self.data_svc is not None:
				self.__stop_data_service()
			if self.ros_svc is not None:
				self.__stop_ros_service()
			if self.proxy is not None:
				#if not self.proxy.IsDataServiceRunning():
				self.proxy.RemoveRtSystem()
			self.proxy=None
			self.subscribed={}
			self.published_param={}
			self.published_command={}
			self.available_components=[]
			self.available_component_types=[]
			self.status_raw=mbs.M3StatusAll()
			self.command_raw=mbs.M3CommandAll()
		except socket.error, msg:
			pass #Ok because shutting down

	# On startup of the M3RtSystem on the server, all components start in state SAFEOP
	# SAFEOP components can only update status data
	# Each component must be manually placed in state OP in order to execute commands

	# ####################################### Component services ###################################
	def make_safe_operational(self,name):
		"""Place a component in state SAFEOP"""
		return self.proxy.SetComponentStateSafeOp(name)

	def make_operational(self,name):
		"""Place a component in state OP"""
		return self.proxy.SetComponentStateOp(name)

	def make_operational_all(self):
		"""Place all components in state OP"""
		names=self.get_available_components()
		for n in names:		
			self.proxy.SetComponentStateOp(n)

	def make_safe_operational_all(self):
		"""Place all components in state SAFEOP"""
		names=self.get_available_components()
		for n in names:
			self.proxy.SetComponentStateSafeOp(n)
			
	# TODO: Fix this on m3rt side
	'''def add_ros_component(self, name):
		"""Start the ros nodes for this component"""
		#return self.proxy.AddRosComponent(name)
		return false'''

	def subscribe_status(self,component,verbose=True):
		"""Subscribe to the servers status message for this component"""
		self.__check_component(component)
		if verbose:
			print 'Subscribing to status for: ',component.name
		self.subscribed[component.name]={'status':component.status,'component':component}
		self.status_raw.name.append(component.name)
		self.status_raw.datum.append('')
		self.proxy.ClientSubscribeStatus(component.name,self.data_port)

	def publish_command(self,component):
		"""Publish this components' command message to the server"""
		self.__check_component(component)
		if self.published_command.has_key(component.name):
			return
		print 'Publishing command for: ',component.name
		self.published_command[component.name]={'component':component,'command':component.command}
		self.command_raw.name_cmd.append(component.name)
		self.command_raw.datum_cmd.append(component.command.SerializeToString())

	def publish_param(self,component):
		"""Publish this component's param message to the server"""
		self.__check_component(component)
		if self.published_param.has_key(component.name):
			return
		print 'Publishing param for: ',component.name
		self.published_param[component.name]={'component':component,'param':component.param}
		self.command_raw.name_param.append(component.name)
		self.command_raw.datum_param.append(component.param.SerializeToString())

	# ########################################## Message Conversion  ########################################
	# All component messages (status/command/param) can be automaticaly converted into dictionaries and back
	# This is a convience functionality for use with the M3Gui, cPickle, etc...
	# TODO: Currently only supports some of protobuf data type (float, string, array of messages)

	def get_status_dict(self):
		"""Build dictionary from all subscribed status messasges"""
		d={}
		for k,v in self.subscribed.items():
			d[k]=m3t.GetDictFromMsg(v['status'])
		return d

	def get_command_dict(self):
		"""Build dictionary from all published command messasges"""
		d={}
		for k,v in self.published_command.items():
			d[k]=m3t.GetDictFromMsg(v['command'])
		return d

	def get_param_dict(self):
		"""Build dictionary from all published param messasges"""
		d={}
		for k,v in self.published_param.items():
			d[k]=m3t.GetDictFromMsg(v['param'])
		return d

	def set_command_from_dict(self,d):
		"""Load all published command messages with data from dictionary"""
		for k,v in self.published_command.items():
			m3t.SetMsgFromDict(v['command'],d[k])

	def set_param_from_dict(self,d):
		"""Load all published param messages with data from dictionary"""
		for k,v in self.published_param.items():
			m3t.SetMsgFromDict(v['param'],d[k])

	# ###################################### Utility Functions #################################################################   

	def get_joint_components(self):
		"""Get a list of available components on the server of common joint types"""
		c=[]
		c+=self.get_available_components('m3joint')
		c+=self.get_available_components('m3joint_slave')
		c+=self.get_available_components('m3joint_zlift')
		return c
	
	def get_chain_components(self):
		"""Get a list of available components on the server of common joint types"""
		c=[]
		c+=self.get_available_components('m3arm')
		c+=self.get_available_components('m3torso')
		c+=self.get_available_components('m3head')		
		return c
	
	def get_available_components(self,type=None):
		"""Get a list of available components on the server of a particular type"""
		if type==None:
			return self.available_components
		c=[]
		for i in range(len(self.available_components)):
			if self.available_component_types[i]==type:
				c.append(self.available_components[i])
		return c

	def get_num_components(self):
		return self.proxy.GetNumComponents()
	
	def is_component_available(self,name):
		"""Is component loaded on the server"""
		if self.proxy is None:
			return False
		return self.proxy.GetComponentIdx(name)>=0

	def pretty_print_component(self,name):
		"""Display component data on server"""
		if self.proxy is None:
			raise m3t.M3Exception('M3RtProxy not started')
		self.proxy.PrettyPrintComponent(name)

	def pretty_print_rt_system(self):
		"""Display system state on server"""
		if self.proxy is not None:
			self.proxy.PrettyPrintRtSystem()

	def pretty_print_component_states(self):
		"""Display all component states locally"""
		names=self.get_available_components()
		states=['STATE_INIT','STATE_ERROR','STATE_SAFEOP','STATE_OP']
		print '------------ Component States ------------'
		for n in names:
			print 'State: ',n,' : ',states[self.proxy.GetComponentState(n)]

	# ################################### Logging Service #################################################
	# The STATUS message of particular components can be logged to disk by the server. 
	# Theses log files are in the Google protocol buffer serialization format
	# The client can then interatively load the files from the server disk into a locally 
	# registered component. 
	# Each log requires a logname. The server will place all logfiles into a directory by that name, and delete
	# any pre-existing files. Thus log directories should be backed up before reusing a name.
	# Each log session runs stores messages at a fixed frequency up to the M3RtSystem cycle frequency.
	# N samples (messages for one cycle period) are stored in a single file on disk.
	# NOTE: Be careful, it is possible to hang the system with too much disk activity. The default settings generally work.

	def register_log_component(self,comp):
		"""Register the component for logging"""
		self.log_comps[comp.name]=comp
		self.log_names.append(comp.name)

	def start_log_service(self,logname, sample_freq_hz=100,samples_per_file=100,logpath=None,verbose=True):
		"""Start logging registered components to directory logname"""
		if logpath is None:
			logpath=os.environ['M3_ROBOT']+'/robot_log'
		if not self.proxy.IsRtSystemRunning():
			raise m3t.M3Exception('Cannot start log. M3RtSystem is not yet running on the server')
		return self.proxy.start_log_service(logname,float(sample_freq_hz),self.log_names,int(samples_per_file),logpath,verbose)

	def stop_log_service(self):
		"""Stop the active logging session"""
		return self.proxy.stop_log_service()

	def get_log_component_names(self,logname):
		"""Get the available components contained in a completed log session"""
		if self.logname!=logname:
			if not self._load_log(logname):
				return []
			filename=self.log_info[0]['filename']
			self.log_page=mbs.M3StatusLogPage()
			s=self.proxy.get_log_file(filename).data
			self.log_page.ParseFromString(s)
		status_all=self.log_page.entry[0]
		return [str(x) for x in status_all.name]

	def get_log_num_samples(self,logname):
		"""Get the number of samples in a completed log session"""
		log_info=self.proxy.get_log_info(logname)
		if len(log_info)==0:
			return 0
		return log_info[-1]['end_idx']

	def load_log_sample(self,logname,idx):
		"""Load sample idx into the subscribed components"""
		if self.logname!=logname:
			if not self._load_log(logname):
				return
			#Load new page if out of bounds
		if idx<self.log_start_idx or idx>self.log_end_idx:
			search_idx=(self.log_file_idx+1)%len(self.log_info)
			while True:
				if self.log_info[search_idx]['start_idx']<=idx and self.log_info[search_idx]['end_idx']>=idx:
					self.log_file_idx=search_idx
					self.log_start_idx=self.log_info[search_idx]['start_idx']
					self.log_end_idx=self.log_info[search_idx]['end_idx']
					break
				search_idx=(search_idx+1)%len(self.log_info)
				if search_idx==self.log_file_idx:
					raise m3t.M3Exception('M3RtProxy invalid log sample idx: '+str(idx))
			filename=self.log_info[self.log_file_idx]['filename']
			self.log_page=mbs.M3StatusLogPage()
			s=self.proxy.get_log_file(filename).data
			self.log_page.ParseFromString(s)
		entry_idx=idx-self.log_info[self.log_file_idx]['start_idx']
		status_all=self.log_page.entry[entry_idx]
		for i in range(len(status_all.name)):
			for name  in self.log_names:
				if name==status_all.name[i]:
					self.log_comps[name].status.ParseFromString(status_all.datum[i])

# #################################### Private methods ################################################################   

	def __send_command(self):
		if self.data_socket is None:
			m3t.M3Exception('M3RtProxy data socket not created')
		idx=0
		for name,v in self.published_command.items():
			self.command_raw.name_cmd[idx]=name
			v['component'].load_command()
			self.command_raw.datum_cmd[idx]=v['command'].SerializeToString()
			idx=idx+1
		idx=0
		for name,v in self.published_param.items():
			self.command_raw.name_param[idx]=name
			v['component'].load_param()
			self.command_raw.datum_param[idx]=v['param'].SerializeToString()
			idx=idx+1   
		nh=array.array('L',[9999]).tostring()
		nc=array.array('L',[self.command_raw.ByteSize()]).tostring()		
		sc=self.command_raw.SerializeToString()
		self.data_socket.sendall(nh+nc+sc)
		
	def __do_receive(self,nr):
		msg = ''
		while len(msg) < nr:
			chunk = self.data_socket.recv(nr-len(msg))
			if chunk == '':
				raise m3t.M3Exception('Proxy socket connection broken')
			msg = msg + chunk
		return msg

	def __recv_status(self):
		if self.data_socket is None:
			m3t.M3Exception('M3RtProxy data socket not created')
		nr=array.array('I')
		rcv=self.__do_receive(4)
		if (len(rcv)!=4):
			raise m3t.M3Exception('Incorrect packet recv size from proxy')
		nr.fromstring(rcv)
		nr=nr[0]
		data=self.__do_receive(nr)
		self.status_raw.ParseFromString(data)
		for name,v in self.subscribed.items():
			for j in range(len(self.status_raw.name)):
				if name==self.status_raw.name[j]:
					if len(self.status_raw.datum[j]): #Allow 0 len on serialize errors
						v['status'].ParseFromString(self.status_raw.datum[j])
						v['component'].update_status()

	def __check_component(self,component):
		"""Verify that the component type matches the server type"""
		if self.proxy is None:
			raise m3t.M3Exception('M3RtProxy not started')
		idx=self.proxy.GetComponentIdx(component.name)
		if idx==-1:
			raise m3t.M3Exception('Component '+component.name+' not available')
		type=self.proxy.GetComponentType(idx)
		if type!=component.type:
			raise m3t.M3Exception('Component type mismatch '+type+' , '+component.type)


	def __start_rt_system(self):
		try:
			try:
				self.rtsys_id=self.proxy.AttachRtSystem()
				if self.rtsys_id==0: #failed to start
					self.stop()
					raise m3t.M3Exception('Unable to start M3RtSystem. Try restarting server')
			except xmlrpclib.ProtocolError,v:
				raise m3t.M3Exception(v)
			#Query available components
			self.available_components=[]
			n=self.proxy.GetNumComponents()
			for i in range(n):
				name=self.proxy.GetComponentName(i)
				self.available_components.append(name)
			#Query component types
			self.available_component_types=[]
			for i in range(n):
				ttype=self.proxy.GetComponentType(i)
				self.available_component_types.append(ttype)
		except socket.error, msg:
			raise m3t.M3Exception('Check that server is started. Socket Error: '+str(msg))

	def __stop_data_service(self):
		try:
			if self.proxy is not None:
				self.proxy.RemoveDataService(self.data_port)
			if self.data_socket is not None:
				self.data_socket.close()
			self.data_socket=None
		except socket.error, msg:
			pass #Ok because shutting down

	def __start_data_service(self):
		#Create data service
		if self.proxy is None:
			raise m3t.M3Exception('M3RtProxy not started')
		#if self.proxy.IsDataServiceRunning():
		#	print 'M3RtDataService already running on port',self.data_port
		#	print 'Stopping existing connection...'
		#	self.proxy.RemoveDataService()
		port = self.proxy.AttachDataService()
		if port == -1 :
			raise m3t.M3Exception('Unable to attach M3RtDataService')
		#print '----------------'
		#print port
		#print '----------------'
		self.data_port = port
		#Create data stream socket
		try:
			self.data_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		except socket.error, msg:
			self.__stop_data_service()
			raise m3t.M3Exception('Error: '+msg[1])
		#Connect to data stream
		try:
			self.data_socket.connect((self.host,self.data_port))
		except socket.error, msg:
			self.__stop_data_service()
			raise m3t.M3Exception('Error: '+msg[1])



	def _load_log(self,logname):
		self.logname=logname
		self.log_info=self.proxy.get_log_info(logname)
		if len(self.log_info)==0:
			return False
		self.log_start_idx=self.log_info[0]['start_idx']
		self.log_end_idx=self.log_info[0]['end_idx']
		self.log_file_idx=0
		filename=self.log_info[0]['filename']
		self.log_page=mbs.M3StatusLogPage()
		s=self.proxy.get_log_file(filename).data
		self.log_page.ParseFromString(s)
		return True

	def __start_ros_service(self):
		#Create ros service
		if self.proxy is None:
			raise m3t.M3Exception('M3RtProxy not started')
		self.proxy.AttachRosService()

	def __stop_ros_service(self):
		if self.proxy is not None:
			self.proxy.RemoveRosService()