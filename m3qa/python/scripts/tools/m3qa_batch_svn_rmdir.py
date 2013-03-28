#! /usr/bin/python

#M3 -- Meka Robotics Robot Components
#Copyright (c) 2010 Meka Robotics
#Author: edsinger@mekabot.com (Aaron Edsinger)

#M3 is free software: you can redistribute it and/or modify
#it under the terms of the GNU Lesser General Public License as published by
#the Free Software Foundation, either version 3 of the License, or
#(at your option) any later version.

#M3 is distributed in the hope that it will be useful,
#but WITHOUT ANY WARRANTY; without even the implied warranty of
#MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#GNU Lesser General Public License for more details.

#You should have received a copy of the GNU Lesser General Public License
#along with m3.  If not, see <http://www.gnu.org/licenses/>.

import m3.toolbox as m3t
import os

stdout_handle = os.popen("find | grep .svn","r")
s = stdout_handle.read()
ss=s.split('\n')
for x in ss:
	if x[-4:]=='.svn':
		print 'Delete',x,'[y]?'
		if m3t.get_yes_no('y'):
			os.system('sudo rm -rf '+x)
		
