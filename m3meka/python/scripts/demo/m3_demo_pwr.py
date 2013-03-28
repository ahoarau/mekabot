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
#along with M3.  If not, see <http://www.gnu.org/licenses/>.

import m3.rt_proxy as m3p
import m3.pwr as m3rt
import m3.pwr_ec as m3e
import m3.toolbox as m3t


proxy = m3p.M3RtProxy()
proxy.start()
pwr_name=proxy.get_available_components('m3pwr')
if len(pwr_name)>1:
            pwr_name=m3t.user_select_components_interactive(pwr_name,single=True)
pwr_ec_name=pwr_name[0].replace('m3pwr','m3pwr_ec')
comp=m3rt.M3Pwr(pwr_name[0])
comp_ec=m3e.M3PwrEc(pwr_ec_name)
proxy.subscribe_status(comp)
proxy.publish_command(comp) 
proxy.subscribe_status(comp_ec)
proxy.make_operational(pwr_name[0])
proxy.make_operational(pwr_ec_name)
while True:
            proxy.step()
            print '--------------'
            print 'Hit any key'
            print 'q: quit'
            print 'o: pwr on'
            print 'f: pwr off'
            print '--------------'
            print
            k=m3t.get_keystroke()
            if k=='q':
                        break
            if k=='o':
                        comp.set_motor_power_on()
            if k=='f':
                        comp.set_motor_power_off()
            print '****************'
            print comp.status
            print 
            print comp_ec.status
            print '****************'
proxy.stop()
