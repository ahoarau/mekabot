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
import m3.omnibase as m3o
import time

proxy = m3p.M3RtProxy()
proxy.start()
pwr_name=proxy.get_available_components('m3pwr')
if len(pwr_name)>1:
            pwr_name=m3t.user_select_components_interactive(pwr_name,single=True)

pwr=m3rt.M3Pwr(pwr_name[0])

proxy.subscribe_status(pwr)

base_name=proxy.get_available_components('m3omnibase')
if len(base_name)!=1:
            print 'Invalid number of base components available'
            proxy.stop()
            exit()
omni=m3o.M3OmniBase(base_name[0])
proxy.subscribe_status(omni)

scope_steer = m3t.M3ScopeN(xwidth=100,n=5,title='Steer and Total Currents')
scope_roll = m3t.M3ScopeN(xwidth=100,n=4,title='Roll Currents')


try:
       ts=time.time()
       while True:
            proxy.step()                        
            motor_current = omni.get_motor_torques()
            roll_total_current = []
            str_current = []
            for i in range(4):
                        roll_current.append(motor_current[i*2])
                        str_current.append(motor_current[i*2+1])
            str_current.append(pwr.get_bus_current_mA())
            scope_steer.plot(str_current)
            scope_roll.plot(roll_current)
            #print 'Time: ',60.0-(time.time()-ts),field,':',v
          
            time.sleep(0.1)
            if False:
                        if time.time()-ts>60.0:
                                    print 'Continue [y]?'
                                    if m3t.get_yes_no('y'):
                                           ts=time.time()
                                    else:
                                           break
except (KeyboardInterrupt,EOFError):
       pass
proxy.stop(force_safeop=False) #allow other clients to continue running
