# M3 -- Meka Robotics Real-Time Control System
# Copyright (c) 2010 Meka Robotics
# Author: Pierre-Luc Bacon <pierrelucbacon@mekabot.com>
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

noinst_HEADERS = \
	m3ec.c

EXTRA_DIST = \
	Kbuild.in
	
BUILT_SOURCES = \
	Kbuild

modules:
	$(MAKE) -C "$(LINUX_SOURCE_DIR)" M="@abs_srcdir@" modules

modules_install:
	$(MAKE) -C "$(LINUX_SOURCE_DIR)" M="@abs_srcdir@" \
		INSTALL_MOD_PATH="$(DESTDIR)" modules_install
clean-local:
	$(MAKE) -C "$(LINUX_SOURCE_DIR)" M="@abs_srcdir@" clean

all:modules

install:modules_install


	
