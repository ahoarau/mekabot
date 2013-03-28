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
#import config_torso_t2r1 as t2r1
import config_torso_t2r2 as t2r2
import config_torso_t2r3 as t2r3
# ##############################################################
config_torso_def={'T2.R2':t2r2.config_full_t2r2,'T2.R3':t2r3.config_full_t2r3}
versions=['T2.R2','T2.R3']                                           
# ##############################################################
 

def generate_dynamatics(torso_name=None,version=None):
    if version is None:
        version=get_version(versions)
    if torso_name is None:
        torso_name=get_torso_name()
    dyn=copy.deepcopy(config_torso_def[version]['dynamatics'])
    ret=[]
    for k in dyn.keys():
	d=dyn[k]
	d['chain_component']='m3torso_'+torso_name
	d['name']='m3dynamatics_'+k+'_'+torso_name
	ret.append(d)
    config_dir=m3t.get_m3_config_path()+torso_name+'/'
    return config_dir, ret

def select_dynamatics(torso_name,version):
    dyn=config_torso_def[version]['dynamatics']
    print 'Select default dynamatics type for torso'
    print '----------------------------------------'
    kk=dyn.keys()
    for i in range(len(kk)):
	print i,':',kk[i]
    idx=m3t.get_int()
    return 'm3dynamatics_'+kk[idx]+'_'+torso_name

def generate_torso(torso_name=None,version=None):
    if version is None:
        version=get_version(versions)
    if torso_name is None:
        torso_name=get_torso_name()  
    torso=copy.deepcopy(config_torso_def[version]['torso'])
    torso['name']='m3torso_'+torso_name
    torso['limb_name']='torso'
    torso['dynamatics_component']=select_dynamatics(torso_name,version)
    torso['joint_components']['J0']='m3joint_'+torso_name+'_j0'
    torso['joint_components']['J1']='m3joint_'+torso_name+'_j1'
    torso['joint_components']['J2']='m3joint_slave_'+torso_name+'_j2'
    config_dir=m3t.get_m3_config_path()+torso_name+'/'
    return config_dir, [torso]

def generate_torso_full():
    version=get_version(versions)
    torso_name=get_torso_name()
    pwr_name=get_pwr_name()
    config_dir=m3t.get_m3_config_path()+torso_name+'/'
    data=[]
    ndof=len(config_torso_def[version]['joint'])
    for jid in range(ndof):
        cd,x=generate_joint(torso_name,version,jid,pwr_name)
        data.extend(x)
    cd,x=generate_torso(torso_name,version)
    data.extend(x)
    cd,x=generate_dynamatics(torso_name,version)
    data.extend(x)
    return config_dir,data
     
def generate_joint(torso_name=None,version=None, jid=None, pwr_name=None):
    if version is None:
        version=get_version(versions)
    if torso_name is None:
        torso_name=get_torso_name()
    if jid is None:
        jid=get_joint_id()
    if pwr_name is None:
        pwr_name=get_pwr_name()
    
    joint=copy.deepcopy(config_torso_def[version]['joint'][jid])
    config_dir=m3t.get_m3_config_path()+torso_name+'/'
    if jid<2:
	joint['name']='m3joint_'+torso_name+'_j'+str(jid)
	actuator_ec=copy.deepcopy(config_torso_def[version]['actuator_ec'][jid])
	actuator=copy.deepcopy(config_torso_def[version]['actuator'][jid])
	name_actuator_ec='m3actuator_ec_'+torso_name+'_j'+str(jid)
	name_actuator='m3actuator_'+torso_name+'_j'+str(jid)
	name_joint='m3joint_'+torso_name+'_j'+str(jid)
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
    if jid==2:
	joint['cpj_component']='m3joint_'+torso_name+'_j1'
	joint['name']='m3joint_slave_'+torso_name+'_j'+str(jid)
	return config_dir, [joint]
    
    