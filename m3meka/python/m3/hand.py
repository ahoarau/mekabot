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



from m3.joint_array import M3JointArray

#Wrapper only
class M3Hand(M3JointArray):
    """Wrapper for 5DOF SEA Hand"""
    def __init__(self,name):
        M3JointArray.__init__(self,name,ndof=5,ctype='m3hand')

class M3HandH1(M3JointArray):
    """Wrapper for 4DOF SEA Hand"""
    def __init__(self,name):
        M3JointArray.__init__(self,name,ndof=4,ctype='m3hand_h1')