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

import m3.toolbox as m3t
from m3qa.config_toolbox import *
import copy



# ##############################################################
urdf_a2r2_right=m3t.get_m3_ros_config_path()+'defs/A2R2_r_arm_defs.urdf.xacro' #set MA*
urdf_a2r2_right=m3t.get_m3_ros_config_path()+'defs/A2R2_l_arm_defs.urdf.xacro' #set MA*
urdf_a2r2_right=m3t.get_m3_ros_config_path()+'robots/m3_A2R2_T2R2_torso_arms.urdf.xacro' #set MT*

client=get_client_code()
f=open(urdf_a2r2_right,'r')
config_dir=m3t.get_m3_ros_config_path()+client
try:
		os.mkdir(config_dir)
		print 'Made dir: ',config_dir
except OSError:
		print 'Dir exists: ', config_dir
fn_d=config_dir+d['name']+'.yml'
f=file(fn_d,'w')
print 'Saving...',fn_d
f.write(yaml.safe_dump(d, default_flow_style=False,width=200))
f.close()

def generate_rviz():
    pass


