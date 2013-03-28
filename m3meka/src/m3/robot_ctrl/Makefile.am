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

## Process this file with automake to produce Makefile.in

#//////////////////////////////////////////////////////////////////

m3_dir = $(top_srcdir)/src
m3rt_dir = $(prefix)/include/m3rt
m3rtb_dir = $(prefix)/include/m3rt/base
ros_dir = $(ROS_DIR)
m3tb_dir = $(prefix)/include/m3/toolbox
libm3robot_ctrlincludedir = $(includedir)/m3/robot_ctrl
libm3robot_ctrlinclude_HEADERS = \
	head_s2csp_ctrl.h\
	head_s2csp_ctrl.pb.h
	

clean-local:
	rm -f *.pb.*
	
head_s2csp_ctrl.pb.h: head_s2csp_ctrl.pb.cpp

head_s2csp_ctrl.pb.cpp: head_s2csp_ctrl.proto
	protoc -I$(prefix)/include/m3rt/base  -I$(prefix)/include/m3/toolbox -I../chains -I../hardware -I./ --cpp_out=./ head_s2csp_ctrl.proto
	mv head_s2csp_ctrl.pb.cc head_s2csp_ctrl.pb.cpp
	protoc -I$(prefix)/include/m3rt/base  -I$(prefix)/include/m3/toolbox -I../chains -I../hardware -I./ --python_out=$(top_srcdir)/python/m3 head_s2csp_ctrl.proto	
	
proto: head_s2csp_ctrl.pb.cpp

proto_install: 
	if test -d $(includedir)/m3; then echo '' ; else mkdir $(includedir)/m3;fi
	if test -d $(includedir)/m3/robot_ctrl; then echo '' ; else mkdir $(includedir)/m3/robot_ctrl;fi
	cp *.proto $(includedir)/m3/robot_ctrl
	
lib_LTLIBRARIES = libm3robot_ctrl.la 
libm3robot_ctrl_la_LIBADD = -lprotobuf -lm3base -lorocos-kdl -lm3hardware -lm3chains -lm3robots -lm3toolbox -lyaml-cpp 
libm3robot_ctrl_la_LDFLAGS = -version-info 1:1:0 -module \
	-L$(ros_dir)/core/roscpp/lib \
	-L$(ros_dir)/core/roslib/lib \
	-L$(ros_dir)/core/rosconsole/lib \
	-L$(ros_dir)/3rdparty/xmlrpcpp/lib \
	-L$(top_srcdir)/src/m3/toolbox \
	-L$(top_srcdir)/src/m3/hardware \
	-L$(top_srcdir)/src/m3/chains \
	-L$(top_srcdir)/src/m3/robots \
	-L$(ros_dir)/../stacks/ros_comm/utilities/cpp_common/lib \
	-L$(ros_dir)/../stacks/ros_comm/clients/cpp/roscpp_traits/lib \
	-L$(ros_dir)/../stacks/ros_comm/clients/cpp/roscpp_serialization/lib \
	-L$(ros_dir)/../stacks/ros_comm/clients/cpp/roscpp/lib \
	-L$(ros_dir)/../stacks/ros_comm/tools/rosconsole/lib \
	-L$(ros_dir)/../stacks/ros_comm/utilities/xmlrpcpp/lib

libm3robot_ctrl_la_CPPFLAGS =   -I../chains -I../robots -I../toolbox -I../hardware -I$(m3tb_dir) \
	-I$(m3_dir) -I$(m3rt_dir) -I$(m3rtb_dir) -ffast-math -pipe  -mhard-float\
	-I$(ros_dir)/core/roscpp/include \
	-I$(top_srcdir)/ros/m3meka_msgs/srv_gen/cpp/include \
	-I$(top_srcdir)/ros/m3meka_msgs/msg_gen/cpp/include \
	-I$(ros_dir)/core/roslib/include \
	$(EIGEN2_CFLAGS) \
	-I$(ros_dir)/core/rosconsole/include \
	-I$(ros_dir)/3rdparty/xmlrpcpp/src\
	-I$(ros_dir)/../stacks/ros_comm/utilities/cpp_common/include \
	-I$(ros_dir)/../stacks/ros_comm/clients/cpp/roscpp_traits/include \
	-I$(ros_dir)/../stacks/ros_comm/clients/cpp/roscpp_serialization/include \
	-I$(ros_dir)/../stacks/ros_comm/clients/cpp/roscpp/include \
	-I$(ros_dir)/../stacks/ros_comm/tools/rosconsole/include \
	-I$(ros_dir)/../stacks/ros_comm/utilities/xmlrpcpp/src\
	-I$(ros_dir)/../stacks/ros_comm/utilities/rostime/include
libm3robot_ctrl_la_SOURCES = \
	head_s2csp_ctrl.pb.cpp\
	head_s2csp_ctrl.cpp\
	factory_proxy.cpp
