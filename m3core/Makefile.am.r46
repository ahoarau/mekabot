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

# NOTE:  must add ros path to ROS_PACKAGE_PATH first...

# NOTE: to do make install you must 'su root'

#ros/m3_msgs 

if RTAI
SUBDIRS = src/m3rt/base \
	src/m3rt/rt_system \
	python \
	src/m3rt/ethercat 
else
SUBDIRS = src/m3rt/base \
	src/m3rt/rt_system \
	python
endif


dist_bin_SCRIPTS = \
	scripts/m3rt_rotate_logs \
	scripts/m3rt_server_run \
	scripts/m3rt_server_kill \
	scripts/lsec scripts/m3rt_insmods \
	scripts/m3rt_rmmods	

EXTRA_DIST = \
        autogen.sh \
        configure

pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = m3base.pc

includedir =  /usr/local/include/m3_msgs

include_HEADERS = ros/m3_msgs/msg_gen/cpp/include/m3_msgs/M3BaseStatus.h \
		ros/m3_msgs/srv_gen/cpp/include/m3_msgs/M3ComponentCmd.h \
		ros/m3_msgs/srv_gen/cpp/include/m3_msgs/M3ComponentParam.h \
		ros/m3_msgs/srv_gen/cpp/include/m3_msgs/M3ComponentStatus.h



		
package: 
	dpkg-buildpackage
