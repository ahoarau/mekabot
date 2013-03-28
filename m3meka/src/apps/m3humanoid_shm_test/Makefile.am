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

rtai_inc = $(RTAI_DIR)/include
#rtai_lib = $(RTAI_DIR)/lib

m3_dir = $(top_srcdir)/src
bin_PROGRAMS = m3humanoid_shm_test
m3humanoid_shm_test_SOURCES = m3humanoid_shm_test.cpp
m3humanoid_shm_test_LDADD = -lpthread



AM_CPPFLAGS = 			-I/usr/realtime/include \
				-I$(m3_dir)\
				-I$(rtai_inc) \
				-D__IN_RTAI__ -ffast-math 







