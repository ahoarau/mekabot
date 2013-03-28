# M3 -- Meka Robotics Robot Components
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

m3_dir = $(top_srcdir)/src/
m3rt_dir = $(prefix)/include/m3rt
m3rtb_dir = $(prefix)/include/m3rt/base
ros_dir = $(ROS_DIR)/..

clean-local:
	rm -f *.pb.*
	
trajectory.pb.h: trajectory.pb.cpp

log.pb.h: log.pb.cpp

trajectory.pb.cpp: trajectory.proto
	protoc -I$(prefix)/include/m3rt/base -I./ --cpp_out=./ trajectory.proto
	mv trajectory.pb.cc trajectory.pb.cpp
	protoc -I$(prefix)/include/m3rt/base -I./ --python_out=$(top_srcdir)/python/m3 trajectory.proto

log.pb.cpp: log.proto
	protoc -I$(prefix)/include/m3rt/base -I./ --cpp_out=./ log.proto
	mv log.pb.cc log.pb.cpp
	protoc -I$(prefix)/include/m3rt/base -I./ --python_out=$(top_srcdir)/python/m3 log.proto


proto: trajectory.pb.cpp

proto_install: 
	if test -d $(includedir)/m3; then echo '' ; else mkdir $(includedir)/m3;fi
	if test -d $(includedir)/m3/toolbox; then echo '' ; else mkdir $(includedir)/m3/toolbox;fi
	cp *.proto $(includedir)/m3/toolbox
	
lib_LTLIBRARIES = libm3toolbox.la 

libm3toolbox_la_LIBADD = $(PROTOBUF_LIBS) $(OROCOSKDL_LIBS) $(YAMLCPP_LIBS) -lm3base -lpthread

libm3toolbox_la_LDFLAGS = -version-info 1:1:0 -module \
	-L$(ros_dir)/ros/core/roscpp/lib \
	-L$(ros_dir)/ros/core/roslib/lib \
	-L$(ros_dir)/ros/core/rosconsole/lib \
	-L$(ros_dir)/ros/3rdparty/xmlrpcpp/lib\
	-L$(ros_dir)/stacks/ros_comm/utilities/cpp_common/lib \
	-L$(ros_dir)/stacks/ros_comm/clients/cpp/roscpp_traits/lib \
	-L$(ros_dir)/stacks/ros_comm/clients/cpp/roscpp_serialization/lib \
	-L$(ros_dir)/stacks/ros_comm/clients/cpp/roscpp/lib \
	-L$(ros_dir)/stacks/ros_comm/tools/rosconsole/lib \
	-L$(ros_dir)/stacks/ros_comm/utilities/xmlrpcpp/lib

libm3toolbox_la_CPPFLAGS =  -I$(m3_dir) -I$(m3rt_dir) -I$(m3rtb_dir) $(EIGEN2_CFLAGS) \
	-pipe -msse2 -ffast-math -lpthread -mhard-float \
	-I$(ros_dir)/ros/core/roscpp/include \
	-I$(ros_dir)/ros/core/roslib/include \
	-I$(ros_dir)/ros/core/rosconsole/include \
	-I$(ros_dir)/ros/3rdparty/xmlrpcpp/src \
	-I$(ros_dir)/stacks/m3_msgs/srv_gen/cpp/include \
	-I$(ros_dir)/stacks/m3_msgs/msg_gen/cpp/include \
	-I$(ros_dir)/stacks/ros_comm/utilities/cpp_common/include \
	-I$(ros_dir)/stacks/ros_comm/clients/cpp/roscpp_traits/include \
	-I$(ros_dir)/stacks/ros_comm/clients/cpp/roscpp_serialization/include \
	-I$(ros_dir)/stacks/ros_comm/clients/cpp/roscpp/include \
	-I$(ros_dir)/stacks/ros_comm/tools/rosconsole/include \
	-I$(ros_dir)/stacks/ros_comm/utilities/xmlrpcpp/src\
	-I$(ros_dir)/stacks/ros_comm/utilities/rostime/include

libm3toolboxincludedir = $(includedir)/m3/toolbox
libm3toolboxinclude_HEADERS = \
	dfilter.h\
	toolbox.h\
	log.h\
	log.pb.h\
	log.proto\
	trajectory.h\
	trajectory.pb.h\
	trajectory.proto\
	monitor.h\
        serial_port.h\
        serial_pdo.h

libm3toolbox_la_SOURCES = \
	trajectory.pb.cpp\
	log.pb.cpp\
	dfilter.cpp\
	trajectory.cpp\
	log.cpp\
	toolbox.cpp\
	monitor.cpp\
	factory_proxy.cpp\
        serial_port.cpp\
        serial_pdo.cpp

