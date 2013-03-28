
m3uta_dir = $(top_srcdir)/src
m3rt_dir = $(prefix)/include/m3rt
m3rtb_dir = $(prefix)/include/m3rt/base
m3inc_dir = $(prefix)/include/m3
eigen_dir = $(top_srcdir)/../../eigen
libm3utaincludedir = $(includedir)/m3uta/controllers
libm3utainclude_HEADERS = \
	example.h\
	example.pb.h
	

clean-local:
	rm -f *.pb.*
	

example.pb.cpp: example.proto
	protoc -I$(prefix)/include/m3rt/base -I./ --cpp_out=./ example.proto
	mv example.pb.cc example.pb.cpp
	protoc -I$(prefix)/include/m3rt/base -I./ --python_out=$(top_srcdir)/python/m3uta example.proto			
proto: example.pb.cpp

proto_install: 
	if test -d $(includedir)/m3uta; then echo '' ; else mkdir $(includedir)/m3uta;fi
	if test -d $(includedir)/m3uta/controllers; then echo '' ; else mkdir $(includedir)/m3uta/controllers;fi
	cp *.proto $(includedir)/m3uta/controllers
	
lib_LTLIBRARIES = libm3uta.la 
libm3uta_la_LIBADD = -lprotobuf -lm3base -lorocos-kdl -lm3hardware -lm3chains -lm3toolbox -lm3robots -lyaml-cpp
libm3uta_la_LDFLAGS = -version-info 0:1:0 -module
libm3uta_la_CPPFLAGS =   -I$(m3uta_dir) -I$(m3inc_dir)/chains -I$(m3inc_dir)/toolbox -I$(m3inc_dir)/hardware -I$(m3inc_dir)/robots -I$(m3inc_dir) -I$(m3rt_dir) -I$(m3rtb_dir) -I$(eigen_dir) -ffast-math -pipe
libm3uta_la_SOURCES = \
	example.pb.cpp\
	example.cpp\
	factory_proxy.cpp

