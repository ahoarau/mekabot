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

#rtai_inc = /usr/realtime/include 
#ros_dir = /opt/ros/cturtle/ros
rtai_inc = $(RTAI_DIR)/include
rtai_lib = $(RTAI_DIR)/lib
ros_dir = $(ROS_DIR)

libm3hardwareincludedir = $(includedir)/m3/hardware
libm3hardwareinclude_HEADERS = \
	m3ec_pdo_v0_def.h\
	m3ec_pdo_v1_def.h\
	m3ec_pdo_v2_def.h\
	m3ec_pdo_v3_def.h\
	m3ec_pdo_v4_def.h\
	m3meka_ec_def.h\
	motor.h\
	actuator.h\
	actuator.pb.h\
	pwr.pb.h\
	pwr.h\
	pwr_virtual.h\
	pwr_ec.h\
	pwr_ec.pb.h\
	actuator_ec.h\
	actuator_ec.pb.h\
	actuator_ec.proto\
	ctrl_simple.pb.h\
	robot_monitor.pb.h\
	ledx2xn_ec.pb.h\
	led_matrix_ec.pb.h\
	ledx2_ec.pb.h\
	tactile_pps22_ec.pb.h\
	loadx6_ec.pb.h\
	loadx6.pb.h\
	loadx1_ec.pb.h\
	loadx1.pb.h\
	joint.pb.h\
	joint.h\
	actuator_virtual.h\
	joint_slave.h\
	sensor.h\
	transmission.h\
	loadx1.h\
	loadx1_ec.h\
	loadx6.h\
	loadx6_virtual.h\
	loadx6_ec.h\
	tactile_pps22_ec.h\
	ledx2_ec.h\
	led_matrix_ec.h\
	joint_zlift.h\
	ledx2xn_ec.h\
	robot_monitor.h\
	ctrl_simple.h\
	smoothing_mode.pb.h\
	joint_mode_ros.pb.h\
	joint_zlift_shm.pb.h\
	joint_zlift_shm_sds.h\
	led_matrix_ec_shm.pb.h\
	led_matrix_ec_shm_sds.h\
	ledx2xn_ec_shm.pb.h\
	ledx2xn_ec_shm_sds.h


#	async_io.h
#	async_io.pb.h

	
clean-local:
	rm -f *.pb.*
	
#////////////////////////// M3 SYS ////////////////////////////
log_test.pb.h: log_test.pb.cpp

log_test.pb.cpp: log_test.proto
	protoc -I$(prefix)/include/m3rt/base -I./ --cpp_out=./ log_test.proto
	mv log_test.pb.cc log_test.pb.cpp
	protoc -I$(prefix)/include/m3rt/base -I./ --python_out=$(top_srcdir)/python/m3 log_test.proto

ledx2xn_ec.pb.h: ledx2xn_ec.pb.cpp

ledx2xn_ec.pb.cpp: ledx2xn_ec.proto
	protoc -I$(prefix)/include/m3rt/base -I./ --cpp_out=./ ledx2xn_ec.proto
	mv ledx2xn_ec.pb.cc ledx2xn_ec.pb.cpp
	protoc -I$(prefix)/include/m3rt/base -I./ --python_out=$(top_srcdir)/python/m3 ledx2xn_ec.proto
	
ledx2_ec.pb.h: ledx2_ec.pb.cpp

ledx2_ec.pb.cpp: ledx2_ec.proto
	protoc -I$(prefix)/include/m3rt/base -I./ --cpp_out=./ ledx2_ec.proto
	mv ledx2_ec.pb.cc ledx2_ec.pb.cpp
	protoc -I$(prefix)/include/m3rt/base -I./ --python_out=$(top_srcdir)/python/m3 ledx2_ec.proto
	
tactile_pps22_ec.pb.h: tactile_pps22_ec.pb.cpp

tactile_pps22_ec.pb.cpp: tactile_pps22_ec.proto
	protoc -I$(prefix)/include/m3rt/base -I./ --cpp_out=./ tactile_pps22_ec.proto
	mv tactile_pps22_ec.pb.cc tactile_pps22_ec.pb.cpp
	protoc -I$(prefix)/include/m3rt/base -I./ --python_out=$(top_srcdir)/python/m3 tactile_pps22_ec.proto
	
loadx6.pb.h: loadx6.pb.cpp

loadx6.pb.cpp: loadx6.proto
	protoc -I$(prefix)/include/m3rt/base -I./ --cpp_out=./ loadx6.proto
	mv loadx6.pb.cc loadx6.pb.cpp
	protoc -I$(prefix)/include/m3rt/base -I./ --python_out=$(top_srcdir)/python/m3 loadx6.proto
	
loadx6_ec.pb.h: loadx6_ec.pb.cpp

loadx6_ec.pb.cpp: loadx6_ec.proto
	protoc -I$(prefix)/include/m3rt/base -I./ --cpp_out=./ loadx6_ec.proto
	mv loadx6_ec.pb.cc loadx6_ec.pb.cpp
	protoc -I$(prefix)/include/m3rt/base -I./ --python_out=$(top_srcdir)/python/m3 loadx6_ec.proto

loadx1.pb.h: loadx1.pb.cpp

loadx1.pb.cpp: loadx1.proto
	protoc -I$(prefix)/include/m3rt/base -I./ --cpp_out=./ loadx1.proto
	mv loadx1.pb.cc loadx1.pb.cpp
	protoc -I$(prefix)/include/m3rt/base -I./ --python_out=$(top_srcdir)/python/m3 loadx1.proto
	
loadx1_ec.pb.h: loadx1_ec.pb.cpp

loadx1_ec.pb.cpp: loadx1_ec.proto
	protoc -I$(prefix)/include/m3rt/base -I./ --cpp_out=./ loadx1_ec.proto
	mv loadx1_ec.pb.cc loadx1_ec.pb.cpp
	protoc -I$(prefix)/include/m3rt/base -I./ --python_out=$(top_srcdir)/python/m3 loadx1_ec.proto
	
actuator.pb.h: actuator.pb.cpp

actuator.pb.cpp: actuator.proto
	protoc -I$(prefix)/include/m3rt/base -I./ --cpp_out=./ actuator.proto
	mv actuator.pb.cc actuator.pb.cpp
	protoc -I$(prefix)/include/m3rt/base -I./ --python_out=$(top_srcdir)/python/m3 actuator.proto

pwr.pb.h: pwr.pb.cpp

pwr.pb.cpp: pwr.proto
	protoc -I$(prefix)/include/m3rt/base -I./ --cpp_out=./ pwr.proto
	mv pwr.pb.cc pwr.pb.cpp
	protoc -I$(prefix)/include/m3rt/base -I./ --python_out=$(top_srcdir)/python/m3 pwr.proto
	
pwr_ec.pb.h: pwr_ec.pb.cpp

pwr_ec.pb.cpp: pwr_ec.proto
	protoc -I$(prefix)/include/m3rt/base -I./ --cpp_out=./ pwr_ec.proto
	mv pwr_ec.pb.cc pwr_ec.pb.cpp
	protoc -I$(prefix)/include/m3rt/base -I./ --python_out=$(top_srcdir)/python/m3 pwr_ec.proto

actuator_ec.pb.h: actuator_ec.pb.cpp

actuator_ec.pb.cpp: actuator_ec.proto
	protoc -I$(prefix)/include/m3rt/base -I./ --cpp_out=./ actuator_ec.proto
	mv actuator_ec.pb.cc actuator_ec.pb.cpp
	protoc -I$(prefix)/include/m3rt/base -I./ --python_out=$(top_srcdir)/python/m3 actuator_ec.proto
	
joint.pb.h: joint.pb.cpp

joint.pb.cpp: joint.proto
	protoc -I$(prefix)/include/m3rt/base   -I./ --cpp_out=./ joint.proto
	mv joint.pb.cc joint.pb.cpp
	protoc -I$(prefix)/include/m3rt/base   -I./ --python_out=$(top_srcdir)/python/m3 joint.proto

led_matrix_ec.pb.h: led_matrix_ec.pb.cpp

led_matrix_ec.pb.cpp: led_matrix_ec.proto
	protoc -I$(prefix)/include/m3rt/base   -I./ --cpp_out=./ led_matrix_ec.proto
	mv led_matrix_ec.pb.cc led_matrix_ec.pb.cpp
	protoc -I$(prefix)/include/m3rt/base   -I./ --python_out=$(top_srcdir)/python/m3 led_matrix_ec.proto

robot_monitor.pb.h: robot_monitor.pb.cpp

robot_monitor.pb.cpp: robot_monitor.proto
	protoc -I$(prefix)/include/m3rt/base   -I./ --cpp_out=./ robot_monitor.proto
	mv robot_monitor.pb.cc robot_monitor.pb.cpp
	protoc -I$(prefix)/include/m3rt/base   -I./ --python_out=$(top_srcdir)/python/m3 robot_monitor.proto

#async_io.pb.h: async_io.pb.cpp

#async_io.pb.cpp: async_io.proto
#	protoc -I$(prefix)/include/m3rt/base   -I./ --cpp_out=./ async_io.proto
#	mv async_io.pb.cc async_io.pb.cpp
#	protoc -I$(prefix)/include/m3rt/base   -I./ --python_out=$(top_srcdir)/python/m3 async_io.proto

ctrl_simple.pb.h: ctrl_simple.pb.cpp

ctrl_simple.pb.cpp: ctrl_simple.proto
	protoc -I$(prefix)/include/m3rt/base   -I./ --cpp_out=./ ctrl_simple.proto
	mv ctrl_simple.pb.cc ctrl_simple.pb.cpp
	protoc -I$(prefix)/include/m3rt/base   -I./ --python_out=$(top_srcdir)/python/m3 ctrl_simple.proto

joint_zlift_shm.pb.h: joint_zlift_shm.pb.cpp

joint_zlift_shm.pb.cpp: joint_zlift_shm.proto
	protoc -I$(prefix)/include/m3rt/base   -I./ --cpp_out=./ joint_zlift_shm.proto
	mv joint_zlift_shm.pb.cc joint_zlift_shm.pb.cpp
	protoc -I$(prefix)/include/m3rt/base   -I./ --python_out=$(top_srcdir)/python/m3 joint_zlift_shm.proto

joint_mode_ros.pb.h: joint_mode_ros.pb.cpp

joint_mode_ros.pb.cpp: joint_mode_ros.proto
	protoc -I$(prefix)/include/m3rt/base   -I./ --cpp_out=./ joint_mode_ros.proto
	mv joint_mode_ros.pb.cc joint_mode_ros.pb.cpp
	protoc -I$(prefix)/include/m3rt/base   -I./ --python_out=$(top_srcdir)/python/m3 joint_mode_ros.proto

smoothing_mode.pb.h: smoothing_mode.pb.cpp

smoothing_mode.pb.cpp: smoothing_mode.proto
	protoc -I$(prefix)/include/m3rt/base   -I./ --cpp_out=./ smoothing_mode.proto
	mv smoothing_mode.pb.cc smoothing_mode.pb.cpp
	protoc -I$(prefix)/include/m3rt/base   -I./ --python_out=$(top_srcdir)/python/m3 smoothing_mode.proto

joint_zlift_shm.pb.h: joint_zlift_shm.pb.cpp

led_matrix_ec_shm.pb.cpp: led_matrix_ec_shm.proto
	protoc -I$(prefix)/include/m3rt/base   -I./ --cpp_out=./ led_matrix_ec_shm.proto
	mv led_matrix_ec_shm.pb.cc led_matrix_ec_shm.pb.cpp
	protoc -I$(prefix)/include/m3rt/base   -I./ --python_out=$(top_srcdir)/python/m3 led_matrix_ec_shm.proto

ledx2xn_ec_shm.pb.cpp: ledx2xn_ec_shm.proto
	protoc -I$(prefix)/include/m3rt/base   -I./ --cpp_out=./ ledx2xn_ec_shm.proto
	mv ledx2xn_ec_shm.pb.cc ledx2xn_ec_shm.pb.cpp
	protoc -I$(prefix)/include/m3rt/base   -I./ --python_out=$(top_srcdir)/python/m3 ledx2xn_ec_shm.proto


proto: actuator.pb.cpp pwr.pb.cpp pwr_ec.pb.cpp  actuator_ec.pb.cpp joint.pb.cpp loadx1.pb.cpp loadx1_ec.pb.cpp \
	loadx6.pb.cpp loadx6_ec.pb.cpp tactile_pps22_ec.pb.cpp ledx2_ec.pb.cpp ledx2xn_ec.pb.cpp led_matrix_ec.pb.cpp \
	robot_monitor.pb.cpp ctrl_simple.pb.cpp joint_zlift_shm.pb.cpp smoothing_mode.pb.cpp joint_mode_ros.pb.cpp \
	led_matrix_ec_shm.pb.cpp ledx2xn_ec_shm.pb.cpp

proto_install: 
	if test -d $(includedir)/m3; then echo '' ; else mkdir $(includedir)/m3;fi
	if test -d $(includedir)/m3/hardware; then echo '' ; else mkdir $(includedir)/m3/hardware;fi
	cp *.proto $(includedir)/m3/hardware 
	
#///////////////////////////////////////////////////////////////////////////////////////////////////
lib_LTLIBRARIES = libm3hardware.la 
libm3hardware_la_LIBADD = -lprotobuf -lm3toolbox -lorocos-kdl -lyaml-cpp -lm3base  $(EIGEN2_LIBS) -llxrt

libm3hardware_la_LDFLAGS = -version-info 1:1:0 -module -L$(rtai_lib)  \
	-L$(ros_dir)/core/roscpp/lib \
	-L$(ros_dir)/core/roslib/lib \
	-L$(ros_dir)/core/rosconsole/lib \
	-L$(ros_dir)/3rdparty/xmlrpcpp/lib\
	-L$(top_srcdir)/src/m3/toolbox \
	-L$(ros_dir)/../stacks/ros_comm/utilities/cpp_common/lib \
	-L$(ros_dir)/../stacks/ros_comm/clients/cpp/roscpp_traits/lib \
	-L$(ros_dir)/../stacks/ros_comm/clients/cpp/roscpp_serialization/lib \
	-L$(ros_dir)/../stacks/ros_comm/clients/cpp/roscpp/lib \
	-L$(ros_dir)/../stacks/ros_comm/tools/rosconsole/lib \
	-L$(ros_dir)/../stacks/ros_comm/utilities/xmlrpcpp/lib


libm3hardware_la_CPPFLAGS =  -I$(rtai_inc) -I$(m3_dir) -I$(m3rt_dir) -I$(m3rtb_dir) -ffast-math -pipe \
	$(EIGEN2_CFLAGS) \
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
	-I$(ros_dir)/../stacks/ros_comm/utilities/xmlrpcpp/src\
	-I$(ros_dir)/../stacks/ros_comm/utilities/rostime/include\
	-D__RTAI__



libm3hardware_la_SOURCES = \
	joint_mode_ros.pb.cpp\
	smoothing_mode.pb.cpp\
	actuator_ec.pb.cpp\
	actuator.pb.cpp\
	loadx1_ec.pb.cpp\
	loadx1.pb.cpp\
	loadx6_ec.pb.cpp\
	robot_monitor.pb.cpp\
	loadx6.pb.cpp\
	ledx2_ec.pb.cpp\
	pwr_ec.pb.cpp\
	pwr.pb.cpp\
	tactile_pps22_ec.pb.cpp\
	joint.pb.cpp\
	ledx2xn_ec.pb.cpp\
	led_matrix_ec.pb.cpp\
	ctrl_simple.pb.cpp\
	log_test.pb.cpp\
 	pwr_ec.cpp\
	pwr.cpp\
	pwr_virtual.cpp\
	actuator_ec.cpp\
	actuator.cpp\
	sensor.cpp\
 	joint.cpp\
	actuator_virtual.cpp\
	joint_slave.cpp\
	joint_zlift.cpp\
  	transmission.cpp\
	loadx1_ec.cpp\
	loadx1.cpp\
	loadx6_ec.cpp\
	loadx6.cpp\
	loadx6_virtual.cpp\
	tactile_pps22_ec.cpp\
	ledx2_ec.cpp\
	motor.cpp\
	led_matrix_ec.cpp\
	ledx2xn_ec.cpp\
	robot_monitor.cpp\
	log_test.cpp\
	ctrl_simple.cpp\
	joint_zlift_shm.pb.cpp\
	joint_zlift_shm.cpp\
	led_matrix_ec_shm.pb.cpp\
	led_matrix_ec_shm.cpp\
	ledx2xn_ec_shm.pb.cpp\
	ledx2xn_ec_shm.cpp\
	factory_proxy.cpp

#	async_io.cpp
#	async_io.pb.cpp	

#///////////////////////////////////////////////////////////////////////////////////////////////////



