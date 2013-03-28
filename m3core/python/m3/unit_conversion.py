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

import math

# #########################################################################

inlb_per_mnm=.00885666
in_per_meter = 39.370079
feet_per_meter=3.2808399
newtons_on_earth_per_kg=9.80665204821735
grams_per_kg=1000
mm_per_meter=1000
kg_per_pound=1.0/2.2046226218
grams_per_newton = grams_per_kg/newtons_on_earth_per_kg
newtons_per_pound=kg_per_pound*newtons_on_earth_per_kg


def N2g(a):
	return a*grams_per_newton
def Nm2inLb(a):
	return a*inlb_per_mnm*1000
def g2mN(a):
	return 1000*a/grams_per_newton
def mN2g(a):
	return a*grams_per_newton/1000.0
def mN2Kg(a):
	return mN2g(a)/1000.0
def Kg2mN(a):
	return g2mN(a*1000.0)
def mN2Lb(a):
	return (a/1000)/newtons_per_pound
def inLb2mNm(a):
	return a/inlb_per_mnm
def mNm2inLb(a):
	return a*inlb_per_mnm
def F2C(a):
	return (a-32.0)*(5.0/9.0)
def C2F(a):
	return 9.0*a/5.0+32.0
def deg2rad(a):
	return math.pi*a/180.0
def rad2deg(a):
	return 180.0*a/math.pi
def m2in(a):
	return a * in_per_meter
def in2m(a):
	return a/in_per_meter
def kg2Lb(a):
	return a/kg_per_pound
def Lb2kg(a):
	return a*kg_per_pound
def Lb2mN(a):
	return g2mN(Lb2kg(a)*1000)
def Lb2N(a):
	return Lb2mN(a)/1000.0
# #########################################################################


