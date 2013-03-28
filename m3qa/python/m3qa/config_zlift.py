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

import config_zlift_z1r1 as z1r1

# ##############################################################
config_zlift_def={'Z1.R1':z1r1.config_zlift_full_z1r1}
versions=['Z1.R1']                                           
# ##############################################################
 
def generate_zlift_full():
    version=get_version(versions)
    zlift_name=get_zlift_name()
    pwr_name=get_pwr_name()
    config_dir=m3t.get_m3_config_path()+zlift_name+'/'
    data=[]
    cd,x=generate_joint(zlift_name,version,0,pwr_name)
    data.extend(x)
    return config_dir,data


def generate_joint(zlift_name=None,version=None, jid=None, pwr_name=None):
    if version is None:
        version=get_version(versions)
    if zlift_name is None:
        zlift_name=get_zlift_name()
    if jid is None:
        jid=get_joint_id()
    if pwr_name is None:
        pwr_name=get_pwr_name()
    actuator_ec=copy.deepcopy(config_zlift_def[version]['actuator_ec'][jid])
    actuator=copy.deepcopy(config_zlift_def[version]['actuator'][jid])
    joint=copy.deepcopy(config_zlift_def[version]['joint'][jid])
    name_actuator_ec='m3actuator_ec_'+zlift_name+'_j'+str(jid)
    name_actuator='m3actuator_'+zlift_name+'_j'+str(jid)
    name_joint='m3joint_'+zlift_name+'_j'+str(jid)
    print 'Enter Serial Number of EC board for joint ','J'+str(jid),'(e.g. 300 for EC300):'
    x=m3t.get_int()
    actuator_ec['ethercat']['serial_number']=x
    actuator_ec['name']=name_actuator_ec
    actuator_ec['pwr_component']=pwr_name
    actuator['name']=name_actuator
    actuator['ec_component']=name_actuator_ec
    actuator['joint_component']=name_joint
    joint['name']=name_joint
    joint['actuator_component']=name_actuator
    joint['transmission']['act_name']=name_actuator
    config_dir=m3t.get_m3_config_path()+zlift_name+'/'
    return config_dir, [actuator_ec, actuator, joint]
