#Copyright  2010, Meka Robotics
#All rights reserved.
#http://mekabot.com

#Redistribution and use in source and binary forms, with or without
#modification, are permitted. 


#THIS SOFTWARE IS PROVIDED BY THE Copyright HOLDERS AND CONTRIBUTORS
#"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#Copyright OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES INCLUDING,
#BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#POSSIBILITY OF SUCH DAMAGE.


import time
import m3.rt_proxy as m3p
import m3.toolbox as m3t
import m3.component_factory as m3f
import Numeric as nu
import m3.humanoid 

# ######################################################	
proxy = m3p.M3RtProxy()
proxy.start()
bot_name=m3t.get_robot_name()
if bot_name == "":
	print 'Error: no robot components found:', bot_name
bot=m3f.create_component(bot_name)
proxy.publish_param(bot) #allow to set payload
proxy.subscribe_status(bot)
proxy.publish_command(bot)
proxy.make_operational_all()
bot.set_motor_power_on()
chains=bot.get_available_chains()
print 'Select chain'
chains=m3t.user_select_components_interactive(chains,single=True)
for c in chains:
	ndof=bot.get_num_dof(c)
	bot.set_mode_theta_gc(c)
	bot.set_theta_deg(c,[0.0]*ndof)
	bot.set_stiffness(c,[0.0]*ndof)	
try:
	while True:
		proxy.step()
		for c in chains:
			print '---------------------------------------------'
			print 'Chain: ',c
			print 'Tool Position: (m)',bot.get_tool_position(c)
			print 'Theta (Deg): ',bot.get_theta_deg(c)
			print 'Tool Velocity (m/S)',bot.get_tool_velocity(c)
		time.sleep(0.1)
except (KeyboardInterrupt,EOFError):
	proxy.stop()
	
	
	
	