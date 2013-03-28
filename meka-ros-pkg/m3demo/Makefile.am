ACLOCAL_AMFLAGS = -I m4

SUBDIRS =  src/m3uta/controllers python

dist_bin_SCRIPTS = 
	
EXTRA_DIST = \
        autogen.sh \
        configure
        
python_install:
	$(MAKE) -C python install
	
proto:
	$(MAKE) -C src/m3uta/controllers proto

proto_install:
	$(MAKE) -C src/m3uta/controllers proto_install
