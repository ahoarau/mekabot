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

import m3.toolbox as m3t
import os

print 'M3QA Search and Replace Script...'
print 'This will recursively replace a substring in all files matching wildcard'
print '-------------------------------------------------------------------------'
print 'Enter old substring'
old=raw_input()
print 'Enter new substring'
new=raw_input()
print 'Enter file wildcard'
wc=raw_input()

cmd='find . -name "'+wc+'" -print | xargs sed -i "s/'+old+'/'+new+'/g"'
#cmd='find . -name "'+wc+'" -exec sed -i "s|'+old+'|'+new+'|g" {}\;'
#find . -name "*" -exec sed -i 's|snake|bip|g' {} \;
#find /path/to/dir -name \*.html -exec sed -i 's|string1|string2|g' {} \;

print 'Cmd: ',cmd
print 'Execute [y]?'
if m3t.get_yes_no('y'):
  os.system(cmd)
print 'Done'