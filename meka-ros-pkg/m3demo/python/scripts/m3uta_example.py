#! /usr/bin/python


import time
import m3.rt_proxy as m3p
import m3.toolbox as m3t
import m3.component_factory as m3f

import m3uta.example as me
	
proxy = m3p.M3RtProxy()
proxy.start()
rex = me.M3Example('m3uta_example_ex0')
proxy.publish_param(rex)
proxy.subscribe_status(rex)
proxy.publish_command(rex)

bot_name=m3t.get_robot_name()
bot=m3f.create_component(bot_name)
#proxy.publish_param(bot) #allow to set payload
proxy.subscribe_status(bot)
#proxy.publish_command(bot)
proxy.make_operational_all()

try:
	while True:
		print 'Enter Fx [0]'
		rex.set_fx(m3t.get_float(0.0))
		print 'Enter Fy [0]'
		rex.set_fy(m3t.get_float(0.0))
		print 'Enter Fz [0]'
		rex.set_fz(m3t.get_float(0.0))
		print 'Run duration?'
		d=max(0,m3t.get_float())
		ts=time.time()
		print 'Hit enter to run'
		raw_input()
		while time.time()-ts<d:
			rex.set_enable_on()
			proxy.step()
			time.sleep(0.1)
			print 'Running...',time.time()-ts
		print 'Done'
		rex.set_enable_off()
		proxy.step()
		time.sleep(0.1)
except (KeyboardInterrupt,EOFError):
	proxy.stop()
