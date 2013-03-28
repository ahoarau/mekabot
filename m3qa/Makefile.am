ACLOCAL_AMFLAGS = -I m4

SUBDIRS = python

dist_bin_SCRIPTS = 
	
EXTRA_DIST = \
        autogen.sh \
        configure
        
python_install:
	$(MAKE) -C python install
	
#proto:
#	$(MAKE) -C src/m3qa/components proto

#proto_install:
#	$(MAKE) -C src/m3qa/components proto_install
