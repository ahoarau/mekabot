
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

m3_dir = $(top_srcdir)/src
rtai_inc = $(RTAI_DIR)/include  
ros_dir = $(ROS_DIR)
clean-local:
	rm -f component_base.pb.cpp
	rm -f component_base.pb.h

proto: component_base.pb.cpp

proto_install: 
	if test -d $(includedir)/m3rt; then echo '' ; else mkdir $(includedir)/m3rt;fi
	if test -d $(includedir)/m3rt/base; then echo '' ; else mkdir $(includedir)/m3rt/base;fi
	cp component_base.proto $(includedir)/m3rt/base 

BUILT_SOURCES = component_base.pb.cpp
component_base.pb.cpp: component_base.proto
	protoc -I./ --cpp_out=./ component_base.proto
	mv component_base.pb.cc component_base.pb.cpp
	protoc -I./ --python_out=$(top_srcdir)/python/m3rt component_base.proto

libm3baseincludedir = $(includedir)/m3rt/base

libm3baseinclude_HEADERS = \
	toolbox.h \
	simple_server.h \
	component_factory.h \
	component_ec.h \
	component.h \
	component_base.pb.h \
	component_async.h \
	component_shm.h \
	component_base.proto \
	m3rt_def.h \
	m3ec_def.h


if RTAI
libm3base_la_SOURCES = \
	component_base.pb.cpp\
	toolbox.cpp \
	simple_server.cpp \
	component_factory.cpp \
	component_ec.cpp \
	component.cpp 	\
	component_async.cpp\
	component_shm.cpp
else
libm3base_la_SOURCES = \
	component_base.pb.cpp\
	toolbox.cpp \
	simple_server.cpp \
	component_factory.cpp \
	component_ec.cpp \
	component.cpp
endif
	
lib_LTLIBRARIES = libm3base.la 

libm3base_la_LIBADD = \
	$(PROTOBUF_LIBS) \
	$(YAMLCPP_LIBS) \
	$(LRT) 

libm3base_la_LDFLAGS = -version-info 1:0:0 -module \
	-L$(ros_dir)/ros/core/roscpp/lib \
	-L$(ros_dir)/ros/core/roslib/lib \
	-L$(ros_dir)/ros/core/rosconsole/lib  \
	-L$(ros_dir)/ros/3rdparty/xmlrpcpp/lib \
	-L$(RTAI_DIR)/lib\
	-L$(ros_dir)/stacks/ros_comm/utilities/cpp_common/lib \
	-L$(ros_dir)/stacks/ros_comm/clients/cpp/roscpp_traits/lib \
	-L$(ros_dir)/stacks/ros_comm/clients/cpp/roscpp_serialization/lib \
	-L$(ros_dir)/stacks/ros_comm/clients/cpp/roscpp/lib \
	-L$(ros_dir)/stacks/ros_comm/tools/rosconsole/lib \
	-L$(ros_dir)/stacks/ros_comm/utilities/xmlrpcpp/lib
	
libm3base_la_CPPFLAGS =  -I$(m3_dir) -Wl,-E -ffast-math -pipe -mhard-float\
	-I$(rtai_inc) \
	-I$(ros_dir)/ros/core/roscpp/include \
	-I$(ros_dir)/ros/core/roslib/include \
	-I$(ros_dir)/ros/core/rosconsole/include \
	-I$(ros_dir)/ros/3rdparty/xmlrpcpp/src \
	-I$(top_srcdir)/ros/m3_msgs/srv_gen/cpp/include \
	-I$(top_srcdir)/ros/m3_msgs/msg_gen/cpp/include \
	-I$(ros_dir)/stacks/ros_comm/utilities/cpp_common/include \
	-I$(ros_dir)/stacks/ros_comm/clients/cpp/roscpp_traits/include \
	-I$(ros_dir)/stacks/ros_comm/clients/cpp/roscpp_serialization/include \
	-I$(ros_dir)/stacks/ros_comm/clients/cpp/roscpp/include \
	-I$(ros_dir)/stacks/ros_comm/tools/rosconsole/include \
	-I$(ros_dir)/stacks/ros_comm/utilities/xmlrpcpp/src\
	-I$(ros_dir)/stacks/ros_comm/utilities/rostime/include
 
