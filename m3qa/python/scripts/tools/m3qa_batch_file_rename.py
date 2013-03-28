#! /usr/bin/python
# -*- coding: utf-8 -*-

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

import glob
import m3.toolbox as m3t
import os

print 'M3QA File Renaming Script...'
print '------------------------'
print 'Enter old substring'
old=raw_input()
print 'Enter new substring'
new=raw_input()
print 'Enter file wildcard'
wc=raw_input()
print 'Recursive [y]?'
if m3t.get_yes_no('y'):
	cmd='find ./ -name \''+wc+'\''
	stdout_handle = os.popen(cmd, "r")
	s = stdout_handle.read()
	files=s.split('\n')
	print 'F',files
else:
	files=glob.glob('./'+wc)
for f in files:
	if f.find(old)!=-1:
		fnew=f.replace(old,new)
		print 'Replace: ',f, 'with:',fnew,' [y]?' 
		if m3t.get_yes_no('y'):
			cmd='mv '+f+' '+fnew
			os.system(cmd) 
			