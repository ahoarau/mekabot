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

import config_omnibase_b1r1 as b1r1

# ##############################################################
config_omnibase_def={'B1.R1':b1r1.config_full_b1r1}
versions=['B1.R1']                                           
# ##############################################################
 


def generate_omnibase(base_name=None,version=None, pwr_name=None):
    if version is None:
        version=get_version(versions)
    if base_name is None:
        base_name=get_base_name()  
    if pwr_name is None:
        pwr_name=get_pwr_name()
    omnibase=copy.deepcopy(config_omnibase_def[version]['base'])
    omnibase['name']='m3omnibase_'+base_name
    omnibase['pwr_component']=pwr_name
    ndof=len(config_omnibase_def[version]['actuator_ec'])
    for jid in range(ndof):
        omnibase['joint_components']['J'+str(jid)]='m3joint_'+base_name+'_j'+str(jid)
    config_dir=m3t.get_m3_config_path()+base_name+'/'
    return config_dir, [omnibase]

def generate_omnibase_full():
    version=get_version(versions)
    base_name=get_base_name()
    pwr_name=get_pwr_name()
    config_dir=m3t.get_m3_config_path()+base_name+'/'
    data=[]
    ndof=len(config_omnibase_def[version]['actuator'])
    for jid in range(ndof):
        cd,x=generate_joint(base_name,version,jid,pwr_name)
        data.extend(x)
    cd,x=generate_omnibase(base_name,version)
    data.extend(x)
    return config_dir,data
     
    
def generate_joint(base_name=None,version=None, jid=None, pwr_name=None):
    if version is None:
        version=get_version(versions)
    if base_name is None:
        base_name=get_base_name()
    if jid is None:
        jid=get_joint_id()
    if pwr_name is None:
        pwr_name=get_pwr_name()
    actuator_ec=copy.deepcopy(config_omnibase_def[version]['actuator_ec'][jid])
    actuator=copy.deepcopy(config_omnibase_def[version]['actuator'][jid])
    joint=copy.deepcopy(config_omnibase_def[version]['joint'][jid])
    name_actuator_ec='m3actuator_ec_'+base_name+'_j'+str(jid)
    name_actuator='m3actuator_'+base_name+'_j'+str(jid)
    name_joint='m3joint_'+base_name+'_j'+str(jid)
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
    config_dir=m3t.get_m3_config_path()+base_name+'/'
    return config_dir, [actuator_ec, actuator, joint]
