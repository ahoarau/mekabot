# M3 -- Meka Robotics Real-Time Control System
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
openrave_dir = openrave


EXTRA_DIST = \
	ik_a1r1_right_for_swig.h\
	ik_a1r1_left_for_swig.h\
	ik_a2r1_right_for_swig.h\
	ik_a2r1_left_for_swig.h\
	ik_a2r2_right_for_swig.h\
	ik_a2r2_left_for_swig.h
	
#clean-local:
#	rm -f ik_a1r1_right_wrap.cpp ik_a1r1_right.py ik_a1r1_left_wrap.cpp ik_a1r1_left.py \
#		ik_a1r1_right_from_ikfast.h ik_a1r1_left_from_ikfast.h \
#		ik_a2r1_right_wrap.cpp ik_a2r1_right.py ik_a2r1_left_wrap.cpp ik_a2r1_left.py \
#		ik_a2r1_right_from_ikfast.h ik_a2r1_left_from_ikfast.h \
#		ik_a2r2_right_wrap.cpp ik_a2r2_right.py ik_a2r2_left_wrap.cpp ik_a2r2_left.py \
#		ik_a2r2_right_from_ikfast.h ik_a2r2_left_from_ikfast.h

ik_a1r1_right_from_ikfast.h:
	openrave -d 0 --generateik robot $(m3_dir)/../ikfast_config/m3_a1r1_right.robot.xml\
		ikfast /usr/local/share/openrave/openravepy/ikfast.py\
		freeparam J2 manipulator 0\
		output ik_a1r1_right_from_ikfast.h

ik_a1r1_left_from_ikfast.h:
	openrave -d 0 --generateik robot $(m3_dir)/../ikfast_config/m3_a1r1_left.robot.xml\
		ikfast /usr/local/share/openrave/openravepy/ikfast.py\
		freeparam J2 manipulator 0\
		output ik_a1r1_left_from_ikfast.h

ik_a2r1_right_from_ikfast.h:
	openrave -d 0 --generateik robot $(m3_dir)/../ikfast_config/m3_a2r1_right.robot.xml\
		ikfast /usr/local/share/openrave/openravepy/ikfast.py\
		freeparam J2 manipulator 0\
		output ik_a2r1_right_from_ikfast.h

ik_a2r1_left_from_ikfast.h:
	openrave -d 0 --generateik robot $(m3_dir)/../ikfast_config/m3_a2r1_left.robot.xml\
		ikfast /usr/local/share/openrave/openravepy/ikfast.py\
		freeparam J2 manipulator 0\
		output ik_a2r1_left_from_ikfast.h
		
ik_a2r2_right_from_ikfast.h:
	openrave -d 0 --generateik robot $(m3_dir)/../ikfast_config/m3_a2r2_right.robot.xml\
		ikfast /usr/local/share/openrave/openravepy/ikfast.py\
		freeparam J2 manipulator 0\
		output ik_a2r2_right_from_ikfast.h

ik_a2r2_left_from_ikfast.h:
	openrave -d 0 --generateik robot $(m3_dir)/../ikfast_config/m3_a2r2_left.robot.xml\
		ikfast /usr/local/share/openrave/openravepy/ikfast.py\
		freeparam J2 manipulator 0\
		output ik_a2r2_left_from_ikfast.h
		
ikfast: ik_a1r1_right_from_ikfast.h ik_a1r1_left_from_ikfast.h ik_a2r1_right_from_ikfast.h ik_a2r1_left_from_ikfast.h ik_a2r2_right_from_ikfast.h ik_a2r2_left_from_ikfast.h 

pkgpython_PYTHON = ik_a1r1_right.py ik_a1r1_left.py ik_a2r1_right.py ik_a2r1_left.py ik_a2r2_right.py ik_a2r2_left.py
pkgpyexec_LTLIBRARIES = _ik_a1r1_right.la _ik_a1r1_left.la _ik_a2r1_right.la _ik_a2r1_left.la _ik_a2r2_right.la _ik_a2r2_left.la


_ik_a1r1_right_la_SOURCES = \
	ik_a1r1_right_wrap.cpp \
	ik_a1r1_right.i

_ik_a1r1_right_la_CPPFLAGS = \
	$(SWIG_PYTHON_CPPFLAGS) \
	-I$(m3_dir)
	
_ik_a1r1_right_la_LDFLAGS = -module -version-info 1:1:0

ik_a1r1_right_wrap.cpp : ik_a1r1_right.i ik_a1r1_right_from_ikfast.h
	$(SWIG) $(SWIG_PYTHON_OPT) -I$(m3_dir) -o $@ $<
	

_ik_a1r1_left_la_SOURCES = \
	ik_a1r1_left_wrap.cpp \
	ik_a1r1_left.i
	
_ik_a1r1_left_la_CPPFLAGS = \
	$(SWIG_PYTHON_CPPFLAGS) \
	-I$(m3_dir)
	
_ik_a1r1_left_la_LDFLAGS = -module -version-info 1:1:0

ik_a1r1_left_wrap.cpp : ik_a1r1_left.i ik_a1r1_left_from_ikfast.h
	$(SWIG) $(SWIG_PYTHON_OPT) -I$(m3_dir)  -o $@ $<
	

_ik_a2r1_right_la_SOURCES = \
	ik_a2r1_right_wrap.cpp \
	ik_a2r1_right.i

_ik_a2r1_right_la_CPPFLAGS = \
	$(SWIG_PYTHON_CPPFLAGS) \
	-I$(m3_dir)
	
_ik_a2r1_right_la_LDFLAGS = -module -version-info 1:1:0

ik_a2r1_right_wrap.cpp : ik_a2r1_right.i ik_a2r1_right_from_ikfast.h
	$(SWIG) $(SWIG_PYTHON_OPT) -I$(m3_dir) -o $@ $<


_ik_a2r1_left_la_SOURCES = \
	ik_a2r1_left_wrap.cpp \
	ik_a2r1_left.i
	
_ik_a2r1_left_la_CPPFLAGS = \
	$(SWIG_PYTHON_CPPFLAGS) \
	-I$(m3_dir)
	
_ik_a2r1_left_la_LDFLAGS = -module -version-info 1:1:0

ik_a2r1_left_wrap.cpp : ik_a2r1_left.i ik_a2r1_left_from_ikfast.h
	$(SWIG) $(SWIG_PYTHON_OPT) -I$(m3_dir)  -o $@ $<


_ik_a2r2_right_la_SOURCES = \
	ik_a2r2_right_wrap.cpp \
	ik_a2r2_right.i

_ik_a2r2_right_la_CPPFLAGS = \
	$(SWIG_PYTHON_CPPFLAGS) \
	-I$(m3_dir)
	
_ik_a2r2_right_la_LDFLAGS = -module -version-info 1:1:0

ik_a2r2_right_wrap.cpp : ik_a2r2_right.i ik_a2r2_right_from_ikfast.h
	$(SWIG) $(SWIG_PYTHON_OPT) -I$(m3_dir) -o $@ $<


_ik_a2r2_left_la_SOURCES = \
	ik_a2r2_left_wrap.cpp \
	ik_a2r2_left.i
	
_ik_a2r2_left_la_CPPFLAGS = \
	$(SWIG_PYTHON_CPPFLAGS) \
	-I$(m3_dir)
	
_ik_a2r2_left_la_LDFLAGS = -module -version-info 1:1:0

ik_a2r2_left_wrap.cpp : ik_a2r2_left.i ik_a2r2_left_from_ikfast.h
	$(SWIG) $(SWIG_PYTHON_OPT) -I$(m3_dir)  -o $@ $<
	
