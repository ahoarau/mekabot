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

import time
import xmlrpclib
from SimpleXMLRPCServer import SimpleXMLRPCServer
import os
import glob
import sys
import ctypes
flags = sys.getdlopenflags()
sys.setdlopenflags(flags | ctypes.RTLD_GLOBAL) #allow exceptions to be passed between dll's
import m3.m3rt_system
import m3.component_base_pb2 as m3b
import socket
import m3.rt_proxy as m3p
import m3.toolbox as m3t
from threading import Thread

	
def start_log_service(logname, freq, components,page_size,logpath=None,verbose=True):
	logdir=m3t.get_log_dir(logname,logpath)
	if logdir is None:
		return False
	if not m3t.make_log_dir(logdir):
		return False
	if len(components)==1 and components[0]=='all':
		components=[]
		n=svc.GetNumComponents()
		for i in range(n):
			components.append(svc.GetComponentName(i))
	if len(components)==0:
		print 'No componentes registered for logging'
		return False
	for c in components:
		svc.AddLogComponent(c)
	return svc.AttachLogService(logname,logdir, freq,page_size,int(verbose)) 

def stop_log_service():
	return svc.RemoveLogService()

def get_log_file(logfilename):
	try:
		f = open(logfilename, "rb")
		s=f.read()
		f.close()
		return xmlrpclib.Binary(s)
	except IOError:
		return ''
def get_log_info(logname,logpath=None):
	return m3t.get_log_info(logname,logpath)

def get_hostname():
	cmd='hostname'
	stdout_handle = os.popen(cmd, "r")
	s = stdout_handle.read()
	return s[:-1]

class client_thread(Thread):
	def __init__ (self):
		Thread.__init__(self)
		self.stop = False
		
	def run(self):		
		self.proxy = m3p.M3RtProxy()		
		self.proxy.start(start_data_svc, start_ros_svc)	
		self.proxy.make_operational_all()		
		try:
			while not self.stop:				
				time.sleep(0.2)
		except:
			pass

# ################################################################################
host=get_hostname()
port=8000
make_op_all = False
start_ros_svc = False
start_data_svc = False

for idx in range(1,len(sys.argv)):
	if sys.argv[idx]=='-host' or sys.argv[idx]=='-h' and idx<len(sys.argv)-1:
		host=sys.argv[idx+1]
	elif sys.argv[idx]=='-port' or sys.argv[idx]=='-p' and idx<len(sys.argv)-1:
		port=int(sys.argv[idx+1])
	elif sys.argv[idx]=='-make_op_all' or sys.argv[idx]=='-m':
		make_op_all = True
	elif sys.argv[idx]=='-start_ros_svc' or sys.argv[idx]=='-r':
		start_ros_svc = True
	elif sys.argv[idx]=='-start_data_svc' or sys.argv[idx]=='-d':
		start_data_svc = True
	elif idx == 1 or sys.argv[idx]=='-help' or sys.argv[idx]=='--help':
		print ''
		print 'M3RT valid arguments:'
		print '   -h, -host <hostname>   specify hostname for server'
		print '   -p, -port <port>       specify port number for server'
		print '   -m, -make_op_all       launch server and place all components in mode Operational'
		print '   -r, -start_ros_svc     start ros service'
		print '   -d, -start_data_svc    start data service'
		print '   -help                  this help screen'
		print ''
		sys.exit()
		

svc=m3.m3rt_system.M3RtService()
svc.Startup() # Let client start rt_system
t = None
try:
	print 'Starting M3 RPC Server on Host: ',host,' at Port: ',port,'...'
	server = SimpleXMLRPCServer((host,port),logRequests=0)
	server.register_introspection_functions()
	server.register_instance(svc)
	server.register_function(start_log_service)
	server.register_function(stop_log_service)
	server.register_function(get_log_file)
	server.register_function(get_log_info)
	
	if make_op_all:
		t = client_thread()
		t.start()
		
	server.serve_forever()
	
	
except socket.error as (errno,strerror):
	print "Error({0}): {1}".format(errno, strerror),'. Check that ',host,'has a valid IP address.'
except:
	pass
# TODO: Find out why server_forever bombs out on CTRL-C if ROS service has been used.
#except KeyboardInterrupt:
#	pass
if make_op_all:
	t.stop = True
#print "Shutting down"
svc.Shutdown()

# ################################################################################
	
	
	
