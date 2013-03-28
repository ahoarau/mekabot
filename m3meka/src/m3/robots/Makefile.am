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
rtai_inc = $(RTAI_DIR)/include
rtai_lib = $(RTAI_DIR)/lib

libm3robotsincludedir = $(includedir)/m3/robots
libm3robotsinclude_HEADERS = \
	chain_name.h\
	humanoid.h\
	robot.h\
	humanoid.pb.h\
	haptic_demo.pb.h\
	haptic_demo.h\
	humanoid_shm.pb.h \
	humanoid_shm_sds.h \
	humanoid_shm.h

clean-local:
	rm -f *.pb.*
	
humanoid.pb.h: humanoid.pb.cpp

humanoid.pb.cpp: humanoid.proto
	protoc -I$(prefix)/include/m3rt/base  -I../toolbox -I../chains -I../hardware -I./ --cpp_out=./ humanoid.proto
	mv humanoid.pb.cc humanoid.pb.cpp
	protoc -I$(prefix)/include/m3rt/base  -I../toolbox -I../chains -I../hardware -I./ --python_out=$(top_srcdir)/python/m3 humanoid.proto	

haptic_demo.pb.h: haptic_demo.pb.cpp

haptic_demo.pb.cpp: haptic_demo.proto
	protoc -I$(prefix)/include/m3rt/base  -I../toolbox -I../chains -I../hardware -I./ --cpp_out=./ haptic_demo.proto
	mv haptic_demo.pb.cc haptic_demo.pb.cpp
	protoc -I$(prefix)/include/m3rt/base  -I../toolbox -I../chains -I../hardware -I./ --python_out=$(top_srcdir)/python/m3 haptic_demo.proto	

	
humanoid_shm.pb.h: humanoid_shm.pb.cpp

humanoid_shm.pb.cpp: humanoid_shm.proto
	protoc -I$(prefix)/include/m3rt/base -I../toolbox -I../hardware -I../chains -I./ --cpp_out=./ humanoid_shm.proto
	mv humanoid_shm.pb.cc humanoid_shm.pb.cpp
	protoc -I$(prefix)/include/m3rt/base -I../toolbox -I../hardware -I../chains -I./ --python_out=$(top_srcdir)/python/m3 humanoid_shm.proto

proto: humanoid.pb.cpp humanoid_shm.pb.cpp  haptic_demo.pb.cpp

proto_install: 
	if test -d $(includedir)/m3; then echo '' ; else mkdir $(includedir)/m3;fi
	if test -d $(includedir)/m3/robots; then echo '' ; else mkdir $(includedir)/m3/robots;fi
	cp *.proto $(includedir)/m3/robots
	
lib_LTLIBRARIES = libm3robots.la 
libm3robots_la_LIBADD = -lprotobuf -lm3base -lorocos-kdl -lm3hardware -lm3chains -lm3toolbox -lyaml-cpp  -llxrt
libm3robots_la_LDFLAGS = -version-info 1:1:0 -module \
	-L$(ros_dir)/core/roscpp/lib \
	-L$(ros_dir)/core/roslib/lib \
	-L$(ros_dir)/core/rosconsole/lib \
	-L$(ros_dir)/3rdparty/xmlrpcpp/lib \
	-L$(top_srcdir)/src/m3/toolbox \
	-L$(top_srcdir)/src/m3/hardware \
	-L$(top_srcdir)/src/m3/chains \
	-L$(ros_dir)/../stacks/ros_comm/utilities/cpp_common/lib \
	-L$(ros_dir)/../stacks/ros_comm/clients/cpp/roscpp_traits/lib \
	-L$(ros_dir)/../stacks/ros_comm/clients/cpp/roscpp_serialization/lib \
	-L$(ros_dir)/../stacks/ros_comm/clients/cpp/roscpp/lib \
	-L$(ros_dir)/../stacks/ros_comm/tools/rosconsole/lib \
	-L$(ros_dir)/../stacks/ros_comm/utilities/xmlrpcpp/lib \
	-L$(rtai_lib)


libm3robots_la_CPPFLAGS =   -I../chains -I../toolbox -I../hardware -I$(m3_dir) -I$(m3tb_dir) \
	-I$(m3rt_dir) -I$(m3rtb_dir) -I$(eigen_dir) -ffast-math -pipe -mhard-float \
	$(EIGEN2_CFLAGS)\
	-I$(ros_dir)/core/roscpp/include \
	-I$(top_srcdir)/ros/m3meka_msgs/srv_gen/cpp/include \
	-I$(top_srcdir)/ros/m3meka_msgs/msg_gen/cpp/include \
	-I$(ros_dir)/core/roslib/include \
	-I$(ros_dir)/core/rosconsole/include \
	-I$(ros_dir)/3rdparty/xmlrpcpp/src \
	-I$(ros_dir)/../stacks/ros_comm/utilities/cpp_common/include \
	-I$(ros_dir)/../stacks/ros_comm/clients/cpp/roscpp_traits/include \
	-I$(ros_dir)/../stacks/ros_comm/clients/cpp/roscpp_serialization/include \
	-I$(ros_dir)/../stacks/ros_comm/clients/cpp/roscpp/include \
	-I$(ros_dir)/../stacks/ros_comm/tools/rosconsole/include \
	-I$(ros_dir)/../stacks/ros_comm/utilities/xmlrpcpp/src \
	-I$(ros_dir)/../stacks/ros_comm/utilities/rostime/include \
	-I/opt/ros/cturtle/stacks/common_msgs/sensor_msgs/msg_gen/cpp/include \
	-I/opt/ros/cturtle/ros/core/roslib/msg_gen/cpp/include \
	-I$(rtai_inc) \
	-D__RTAI__

libm3robots_la_SOURCES = \
	humanoid.pb.cpp\
	humanoid_shm.pb.cpp\
	haptic_demo.pb.cpp\
	humanoid.cpp\
	humanoid_shm.cpp\
	robot.cpp\
	haptic_demo.cpp\
	factory_proxy.cpp
