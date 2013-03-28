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
#along with M3.  If not, see <http://www.gnu.org/licenses/>.



from m3.component import M3Component
import m3.async_io_pb2 as mab

class M3AsyncIO(M3Component):
    
    def __init__(self,name,ctype='m3async_io'):
        M3Component.__init__(self,name,type=ctype)
	self.read_config()
        self.status=mab.M3AsyncIOStatus()
        self.command=mab.M3AsyncIOCommand()
        self.param=mab.M3AsyncIOParam()

    def set_test(self, x):
        self.command.tmp = 1.0
        
    def get_test(self):
        return self.status.async.tmp