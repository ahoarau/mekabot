
m3qa_dir = $(top_srcdir)/src
m3rt_dir = $(prefix)/include/m3rt
m3rtb_dir = $(prefix)/include/m3rt/base
m3inc_dir = $(prefix)/include/m3
eigen_dir = $(top_srcdir)/../eigen
ros_dir = /opt/ros/cturtle/ros
libm3qaincludedir = $(includedir)/m3qa/components
libm3qainclude_HEADERS = \
	example.h\
	example.pb.h
	

clean-local:
	rm -f *.pb.*
	
example.pb.cpp: example.proto
	protoc -I$(m3rtb_dir)  -I$(m3inc_dir)/toolbox -I$(m3inc_dir)/chains -I$(m3inc_dir)/hardware -I$(m3inc_dir)/robots -I./ --cpp_out=./ example.proto
	mv example.pb.cc example.pb.cpp
	
	
proto: example.pb.cpp

proto_install: 
	if test -d $(includedir)/m3qa; then echo '' ; else mkdir $(includedir)/m3qa;fi
	if test -d $(includedir)/m3qa/components; then echo '' ; else mkdir $(includedir)/m3qa/components;fi
	cp *.proto $(includedir)/m3qa/components
	
lib_LTLIBRARIES = libm3qa.la 
libm3qa_la_LIBADD = -lprotobuf -lm3base -lorocos-kdl -lm3hardware -lm3chains -lm3toolbox -lm3robots -lyaml-cpp -lros -lroslib
libm3qa_la_LDFLAGS = -version-info 0:1:0 -module\
	-L$(ros_dir)/core/roscpp/lib \
	-L$(ros_dir)/core/roslib/lib \
	-L$(ros_dir)/core/rosconsole/lib \
	-L$(ros_dir)/3rdparty/xmlrpcpp/lib
libm3qa_la_CPPFLAGS =   -I$(m3qa_dir) -I$(m3inc_dir)/chains -I$(m3inc_dir)/toolbox -I$(m3inc_dir)/hardware -I$(m3inc_dir)/robots -I$(m3inc_dir) -I$(m3rt_dir) -I$(m3rtb_dir) -I$(eigen_dir) -ffast-math -pipe\
	-I$(ros_dir)/core/roscpp/include \
	-I$(ros_dir)/../stacks/m3_msgs/srv_gen/cpp/include \
	-I$(ros_dir)/../stacks/m3_msgs/msg_gen/cpp/include \
	-I$(ros_dir)/core/roslib/include \
	-I$(ros_dir)/core/rosconsole/include \
	-I$(ros_dir)/3rdparty/xmlrpcpp/src
libm3qa_la_SOURCES = \
	example.pb.cpp\
	example.cpp\
	factory_proxy.cpp

