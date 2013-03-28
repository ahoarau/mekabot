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
import config_arm_a2r1 as a2r1
import config_arm_a2r2 as a2r2
import config_arm_a2r3 as a2r3
import config_arm_a2r4 as a2r4
# ##############################################################
config_arm_def={'A2.R1':a2r1.config_full_a2r1,
		'A2.R2':a2r2.config_full_a2r2,
                'A2.R3':a2r3.config_full_a2r3,
                'A2.R4':a2r4.config_full_a2r4}
versions=['A2.R1','A2.R2','A2.R3','A2.R4']                  
limbs=['right_arm','left_arm']                         
# ##############################################################
 
def generate_dynamatics(arm_name=None,version=None,limb_name=None):
    if version is None:
        version=get_version(versions)
    if arm_name is None:
        arm_name=get_arm_name()
    if limb_name is None:
        limb_name=get_limb_name(limbs)  
    dyn=copy.deepcopy(config_arm_def[version]['dynamatics'])
    ret=[]
    kk=dyn.keys()
    if limb_name=='right_arm':
	xx=[yy for yy in kk if yy.find('right')>=0]
    if limb_name=='left_arm':
	xx=[yy for yy in kk if yy.find('left')>=0]
    for x in xx:
	d=dyn[x]
	d['chain_component']='m3arm_'+arm_name
	d['name']='m3dynamatics_'+x+'_'+arm_name
	ret.append(d)
    config_dir=m3t.get_m3_config_path()+arm_name+'/'
    return config_dir, ret

def select_dynamatics(arm_name,version,limb_name):
    dyn=config_arm_def[version]['dynamatics']
    print 'Select default dynamatics type for arm'
    print '----------------------------------------'
    kk=dyn.keys()
    if limb_name=='right_arm':
	xx=[yy for yy in kk if yy.find('right')>=0]
    if limb_name=='left_arm':
	xx=[yy for yy in kk if yy.find('left')>=0]
    for i in range(len(xx)):
	print i,':',xx[i]
    idx=m3t.get_int()
    return 'm3dynamatics_'+xx[idx]+'_'+arm_name

def generate_arm(arm_name=None,version=None, limb_name=None):
    if version is None:
        version=get_version(versions)
    if arm_name is None:
        arm_name=get_arm_name()
    if limb_name is None:
        limb_name=get_limb_name(limbs)  
    arm=copy.deepcopy(config_arm_def[version]['arm'])
    arm['name']='m3arm_'+arm_name
    arm['limb_name']=limb_name
    arm['dynamatics_component']=select_dynamatics(arm_name,version,limb_name)
    ndof=len(config_arm_def[version]['actuator_ec'])
    for jid in range(ndof):
        arm['joint_components']['J'+str(jid)]='m3joint_'+arm_name+'_j'+str(jid)
    config_dir=m3t.get_m3_config_path()+arm_name+'/'
    return config_dir, [arm]

def generate_arm_full():
    version=get_version(versions)
    arm_name=get_arm_name()
    limb_name=get_limb_name(limbs)
    pwr_name=get_pwr_name()
    config_dir=m3t.get_m3_config_path()+arm_name+'/'
    data=[]
    ndof=len(config_arm_def[version]['actuator_ec'])
    for jid in range(ndof):
        cd,x=generate_joint(arm_name,version,jid,pwr_name)
        data.extend(x)
    cd,x=generate_arm(arm_name,version, limb_name)
    data.extend(x)
    cd,x=generate_dynamatics(arm_name,version, limb_name)
    data.extend(x)
    return config_dir,data
     
def generate_joint(arm_name=None,version=None, jid=None, pwr_name=None):
    if version is None:
        version=get_version(versions)
    if arm_name is None:
        arm_name=get_arm_name()
    if jid is None:
        jid=get_joint_id()
    if pwr_name is None:
        pwr_name=get_pwr_name()
        
    actuator_ec=copy.deepcopy(config_arm_def[version]['actuator_ec'][jid])
    actuator=copy.deepcopy(config_arm_def[version]['actuator'][jid])
    joint=copy.deepcopy(config_arm_def[version]['joint'][jid])
    name_actuator_ec='m3actuator_ec_'+arm_name+'_j'+str(jid)
    name_actuator='m3actuator_'+arm_name+'_j'+str(jid)
    name_joint='m3joint_'+arm_name+'_j'+str(jid)
    
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
    
    if version=='A2.R1' or version=='A2.R2' or version=='A2.R3' or version=='A2.R4':
        if jid==5:
            joint['transmission']['cpj_name']=name_joint[:-1]+'6'
        if jid==6:
            joint['transmission']['cpj_name']=name_joint[:-1]+'5'
    
    config_dir=m3t.get_m3_config_path()+arm_name+'/'

    return config_dir, [actuator_ec, actuator, joint]
    