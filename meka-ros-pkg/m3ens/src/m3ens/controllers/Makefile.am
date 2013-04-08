
m3inc_dir = $(prefix)/include/m3
m3ens_dir = $(top_srcdir)/src
m3rtb_dir = $(prefix)/include/m3rt/base
m3rt_dir = $(prefix)/include/m3rt


rtai_inc = $(RTAI_DIR)/include  
ros_dir = $(ROS_DIR)/ros
rtai_lib = $(RTAI_DIR)/lib
libm3enscontrollersincludedir = $(includedir)/m3ens/controllers

libm3enscontrollersinclude_HEADERS = \
	example.h \
	example.pb.h 
    
clean-local:
	rm -f *.pb.*
	

example.pb.cpp: example.proto
	protoc -I$(m3ens_dir) -I$(m3rt_dir)/base -I$(m3inc_dir)/toolbox -I$(m3inc_dir)/hardware -I$(m3inc_dir)/chains -I./  --cpp_out=./ example.proto
	mv example.pb.cc example.pb.cpp
	protoc -I$(m3ens_dir) -I$(m3rt_dir)/base -I$(m3inc_dir)/toolbox -I$(m3inc_dir)/hardware -I$(m3inc_dir)/chains -I./  --python_out=$(top_srcdir)/python/m3ens example.proto		
	
proto: example.pb.cpp

proto_install: 
	if test -d $(includedir)/m3ens; then echo '' ; else mkdir $(includedir)/m3ens;fi
	if test -d $(includedir)/m3ens/controllers; then echo '' ; else mkdir $(includedir)/m3ens/controllers;fi
	cp *.proto $(includedir)/m3ens/controllers
	
lib_LTLIBRARIES = libm3enscontrollers.la 

libm3enscontrollers_la_LIBADD = -lprotobuf -lm3base -lorocos-kdl -lm3hardware -lm3chains -lm3toolbox -lyaml-cpp -lXmlRpc -lros -lroslib $(PROTOBUF_LIBS) $(YAMLCPP_LIBS) $(EIGEN2_LIBS) 

libm3enscontrollers_la_LDFLAGS = -version-info 0:1:0 -module \
	-L$(ros_dir)/core/roscpp/lib \
	-L$(ros_dir)/core/roslib/lib \
	-L$(ros_dir)/core/rosconsole/lib \
	-L$(ros_dir)/3rdparty/xmlrpcpp/lib \
	-L$(rtai_lib)
    
libm3enscontrollers_la_CPPFLAGS =  -I$(rtai_inc) -I$(m3ens_dir) -I$(m3inc_dir)/shared_mem -I$(m3inc_dir)/chains -I$(m3inc_dir)/toolbox \
	-I$(m3inc_dir)/hardware -I$(m3inc_dir)/robots -I$(m3inc_dir) -I$(m3rt_dir) -I$(m3rtb_dir) -I$(eigen_dir) $(PROTOBUF_CFLAGS) $(YAMLCPP_CFLAGS) $(EIGEN2_CFLAGS) \
	-ffast-math -pipe \
	-I$(top_srcdir)/ros/m3ens_ros/msg_gen/cpp/include \
	-I$(top_srcdir)/ros/m3ens_ros/srv_gen/cpp/include \
	-I/opt/ros/cturtle/stacks/m3_msgs/srv_gen/cpp/include \
	-I/opt/ros/cturtle/stacks/m3_msgs/msg_gen/cpp/include \
	-I$(ros_dir)/core/roscpp/include \
	-I$(ros_dir)/core/roslib/include \
	-I$(ros_dir)/core/rosconsole/include \
	-I$(ros_dir)/core/roslib/msg_gen/cpp/include \
	-I$(ros_dir)/3rdparty/xmlrpcpp/src \
	-I/usr/local/include/m3rt/base/ \
	-I/opt/ros/cturtle/stacks/geometry/tf/include \
	-I/opt/ros/cturtle/stacks/common_msgs/geometry_msgs/msg_gen/cpp/include/ \
	-I/opt/ros/cturtle/stacks/geometry/bullet/include/ \
	-I/opt/ros/cturtle/stacks/geometry/tf/msg_gen/cpp/include/ \
	-I/opt/ros/cturtle/stacks/geometry/tf_conversions/include/ \
	-I/opt/ros/cturtle/stacks/common_msgs/geometry_msgs/msg_gen/cpp/include/geometry_msgs/
	    	
libm3enscontrollers_la_SOURCES = \
	factory_proxy.cpp \
	example.pb.cpp\
	example.cpp 
