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
import config_head_s2r1 as s2r1
import config_head_s2r2 as s2r2


# ##############################################################
config_head_def={'S2.R1.ISS':s2r1.config_full_s2r1_iss, 'S2.R1.UTA':s2r1.config_full_s2r1_uta, 'S2.R2.UTA':s2r2.config_full_s2r2_ens,}
versions=['S2.R1.ISS','S2.R1.UTA','S2.R2.ENS']                                           
# ##############################################################
 

def generate_head(head_name=None,version=None):
    if version is None:
        version=get_version(versions)
    if head_name is None:
        head_name=get_head_name()  
    head=copy.deepcopy(config_head_def[version]['head'])
    head['name']='m3head_'+head_name
    head['limb_name']='head'
    head['dynamatics_component']=head['dynamatics_component'].replace('xxx',head_name)
    #head['dynamatics_component']=select_dynamatics(head_name,version)
    for jn in head['joint_components']:
	head['joint_components'][jn]='m3joint_'+head_name+'_'+jn.lower()
    config_dir=m3t.get_m3_config_path()+head_name+'/'
    return config_dir, [head]

def generate_head_full():
    version=get_version(versions)
    head_name=get_head_name()
    pwr_name=get_pwr_name()
    config_dir=m3t.get_m3_config_path()+head_name+'/'
    data=[]
    ndof=len(config_head_def[version]['actuator_ec'])
    for jid in range(ndof):
        cd,x=generate_joint(head_name,version,jid,pwr_name)
        data.extend(x)
    cd,x=generate_head(head_name,version)
    data.extend(x)
    cd,x=generate_dynamatics(head_name,version)
    data.extend(x)
    return config_dir,data

def generate_joint(head_name=None,version=None, jid=None, pwr_name=None):
    if version is None:
        version=get_version(versions)
    if head_name is None:
        head_name=get_head_name()
    if jid is None:
        jid=get_joint_id()
    if pwr_name is None:
        pwr_name=get_pwr_name()
    joint=copy.deepcopy(config_head_def[version]['joint'][jid])
    config_dir=m3t.get_m3_config_path()+head_name+'/'
    joint['name']='m3joint_'+head_name+'_j'+str(jid)
    actuator_ec=copy.deepcopy(config_head_def[version]['actuator_ec'][jid])
    actuator=copy.deepcopy(config_head_def[version]['actuator'][jid])
    name_actuator_ec='m3actuator_ec_'+head_name+'_j'+str(jid)
    name_actuator='m3actuator_'+head_name+'_j'+str(jid)
    name_joint='m3joint_'+head_name+'_j'+str(jid)
    print 'Enter Serial Number of EC board for joint ','J'+str(jid),'(e.g. 300 for EC300):'
    x=m3t.get_int()
    actuator_ec['ethercat']['serial_number']=x
    actuator_ec['name']=name_actuator_ec
    actuator_ec['pwr_component']=pwr_name    
    actuator['name']=name_actuator
    actuator['ec_component']=name_actuator_ec
    actuator['joint_component']=name_joint
    joint['actuator_component']=name_actuator
    joint['transmission']['act_name']=name_actuator
    return config_dir, [actuator_ec, actuator, joint]

    
def generate_dynamatics(head_name=None,version=None):
    if version is None:
        version=get_version(versions)
    if head_name is None:
        head_name=get_head_name()
    dyn=copy.deepcopy(config_head_def[version]['dynamatics'])
    dyn['chain_component']='m3head_'+head_name
    dyn['name']=dyn['name'].replace('xxx',head_name)
    config_dir=m3t.get_m3_config_path()+head_name+'/'
    return config_dir, [dyn]

