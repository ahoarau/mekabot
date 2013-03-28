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

ACLOCAL_AMFLAGS = -I m4
AM_CPPFLAGS = -g 

#SUBDIRS =   src/m3rt/base  src/m3rt/rt_system src/m3/toolbox src/m3/hardware src/m3/inverse_kine src/m3/chains \
#	src/m3/robots src/m3/robot_ctrl  python

#ros/m3meka_msgs
#src/m3rt/bootloader 

SUBDIRS =   src/m3/inverse_kine \
	python

	


python_install:
	$(MAKE) -C python install
		
ik:
	$(MAKE) -C src/m3/inverse_kine

ik_install:
	$(MAKE) -C src/m3/inverse_kine install
		
