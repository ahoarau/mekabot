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
m3tb_dir = $(prefix)/include/m3/toolbox

ros_dir = $(ROS_DIR)


libm3chainsincludedir = $(includedir)/m3/chains
libm3chainsinclude_HEADERS = \
	joint_array.h\
	joint_array_mode.pb.h\
	joint_array.pb.h\
	joint_chain.h\
	arm.h\
	hand.h\
	hand_ua.h\
	hand_ua.pb.h\
	torso.h\
	gripper.h\
	head.h\
	dynamatics.h\
	dynamatics.pb.h

clean-local:
	rm -f *.pb.*
	
#////////////////////////// M3 SYS ////////////////////////////
joint_array_mode.pb.h: joint_array_mode.pb.cpp

joint_array_mode.pb.cpp: joint_array_mode.proto
	protoc -I$(prefix)/include/m3rt/base -I../m3/toolbox -I../hardware -I./ --cpp_out=./ joint_array_mode.proto
	mv joint_array_mode.pb.cc joint_array_mode.pb.cpp
	protoc -I$(prefix)/include/m3rt/base -I../m3/toolbox -I../hardware -I./ --python_out=$(top_srcdir)/python/m3 joint_array_mode.proto
	
joint_array.pb.h: joint_array.pb.cpp

joint_array.pb.cpp: joint_array.proto
	protoc -I$(prefix)/include/m3rt/base -I../toolbox -I../hardware -I./ --cpp_out=./ joint_array.proto
	mv joint_array.pb.cc joint_array.pb.cpp
	protoc -I$(prefix)/include/m3rt/base -I../toolbox -I../hardware -I./ --python_out=$(top_srcdir)/python/m3 joint_array.proto

hand_ua.pb.h: hand_ua.pb.cpp

hand_ua.pb.cpp: hand_ua.proto
	protoc -I$(prefix)/include/m3rt/base -I../toolbox -I../hardware -I./ --cpp_out=./ hand_ua.proto
	mv hand_ua.pb.cc hand_ua.pb.cpp
	protoc -I$(prefix)/include/m3rt/base -I../toolbox -I../hardware -I./ --python_out=$(top_srcdir)/python/m3 hand_ua.proto

dynamatics.pb.h: dynamatics.pb.cpp

dynamatics.pb.cpp: dynamatics.proto
	protoc -I$(prefix)/include/m3rt/base -I../m3/toolbox -I../hardware -I./ --cpp_out=./ dynamatics.proto
	mv dynamatics.pb.cc dynamatics.pb.cpp
	protoc -I$(prefix)/include/m3rt/base -I../m3/toolbox -I../hardware -I./ --python_out=$(top_srcdir)/python/m3 dynamatics.proto



proto: joint_array_mode.pb.cpp joint_array.pb.cpp dynamatics.pb.cpp hand_ua.pb.cpp

proto_install: 
	if test -d $(includedir)/m3; then echo '' ; else mkdir $(includedir)/m3;fi
	if test -d $(includedir)/m3/chains; then echo '' ; else mkdir $(includedir)/m3/chains;fi
	cp *.proto $(includedir)/m3/chains
	
lib_LTLIBRARIES = libm3chains.la 
libm3chains_la_LIBADD = -lprotobuf -lm3base -lorocos-kdl -lm3hardware -lm3toolbox -lyaml-cpp 
libm3chains_la_LDFLAGS = -version-info 1:1:0 -module \
	-L$(ros_dir)/core/roscpp/lib \
	-L$(ros_dir)/core/roslib/lib \
	-L$(ros_dir)/core/rosconsole/lib \
	-L$(ros_dir)/3rdparty/xmlrpcpp/lib \
	-L$(top_srcdir)/src/m3/toolbox \
	-L$(top_srcdir)/src/m3/hardware \
	-L$(ros_dir)/../stacks/ros_comm/utilities/cpp_common/lib \
	-L$(ros_dir)/../stacks/ros_comm/clients/cpp/roscpp_traits/lib \
	-L$(ros_dir)/../stacks/ros_comm/clients/cpp/roscpp_serialization/lib \
	-L$(ros_dir)/../stacks/ros_comm/clients/cpp/roscpp/lib \
	-L$(ros_dir)/../stacks/ros_comm/tools/rosconsole/lib \
	-L$(ros_dir)/../stacks/ros_comm/utilities/xmlrpcpp/lib
	



libm3chains_la_CPPFLAGS =   -I../hardware/ -I../toolbox/ -I$(m3_dir) \
	-I$(m3rt_dir) -I$(m3rtb_dir) -ffast-math -pipe -mhard-float \
	-I$(m3tb_dir) \
	$(EIGEN2_CFLAGS) \
	-I$/usr/local/include

	

libm3chains_la_SOURCES = \
	joint_array_mode.pb.cpp\
	joint_array.pb.cpp\
	dynamatics.pb.cpp\
	hand_ua.pb.cpp\
	joint_array.cpp\
	joint_chain.cpp\
	arm.cpp\
	hand.cpp\
	hand_ua.cpp\
	torso.cpp\
	gripper.cpp\
	head.cpp\
	dynamatics.cpp\
	factory_proxy.cpp

