# M3 -- Meka Robotics Real-Time Control System
# Copyright (c) 2010 Meka Robotics
# Author: edsinger@mekabot.com (Aaron Edsinger)
# 
# M3 is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# M3 is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
# 
# You should have received a copy of the GNU Lesser General Public License
# along with M3.  If not, see <http://www.gnu.org/licenses/>.

## Process this file with automake to produce Makefile.in

#//////////////////////////////////////////////////////////////////

m3_dir = $(top_srcdir)/src
ros_dir = /opt/ros/cturtle/ros
EXTRA_DIST = \
	m3_bootloader.h\
	mem.h\
	m3_bootloader.i
	
etherlab_inc = /opt/etherlab/include 
rtai_inc = /usr/realtime/include 

clean-local:
	rm -f m3_bootloader_wrap.cpp
	
pkgpython_PYTHON = m3_bootloader.py
pkgpyexec_LTLIBRARIES = _m3_bootloader.la

_m3_bootloader_la_SOURCES = \
	m3_bootloader.cpp \
	mem.cpp \
	m3_bootloader_wrap.cpp \
	m3_bootloader.i 
	
_m3_bootloader_la_CPPFLAGS = \
	$(SWIG_PYTHON_CPPFLAGS) \
	-I$(m3_dir)\
	-I$(etherlab_inc) -I$(rtai_inc) \
	-D__LINUX__ -D__IN_RTAI__ -D__RTAI__ -D_REENTRANT \
	-I$(ros_dir)/core/roscpp/include \
	-I$(top_srcdir)/ros/m3_msgs/srv_gen/cpp/include \
	-I$(top_srcdir)/ros/m3_msgs/msg_gen/cpp/include \
	-I$(ros_dir)/core/roslib/include \
	-I$(ros_dir)/core/rosconsole/include \
	-I$(ros_dir)/3rdparty/xmlrpcpp/src
	
_m3_bootloader_la_LDFLAGS = -module -version-info 1:1:0 
_m3_bootloader_la_LIBADD =  
m3_bootloader_wrap.cpp : m3_bootloader.i
	$(SWIG) $(SWIG_PYTHON_OPT) -I$(m3_dir)  -o $@ $<

