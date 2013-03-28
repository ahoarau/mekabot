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
import copy
#import config_hand_h2r1 as h2r1
import m3qa.config_hand_h2r2 as h2r2
import m3qa.config_hand_h2r3 as h2r3
import m3qa.config_hand_h2r4 as h2r4
from m3qa.config_toolbox import *


# ##############################################################
config_hand_def={'H2.R2':h2r2.config_full_h2r2,'H2.R3':h2r3.config_full_h2r3,'H2.R4':h2r4.config_full_h2r4}
versions=['H2.R2','H2.R3','H2.R4']                  
limbs=['right_hand','left_hand']
# ##############################################################   
       
    
def get_hand_name():
    while True:
        print 'Enter Hand Number (e.g., 6 for MH6)'
        return 'mh'+str(m3t.get_int())
        
def generate_hand(hand_name=None,version=None, limb_name=None):
    if version is None:
        version=get_version(versions)
    if hand_name is None:
        hand_name=get_hand_name()
    if limb_name is None:
        limb_name=get_limb_name(limbs)  
    hand=copy.deepcopy(config_hand_def[version]['hand'])
    hand['name']='m3hand_'+hand_name
    hand['limb_name']=limb_name
    ndof=len(config_hand_def[version]['actuator_ec'])
    for jid in range(ndof):
        hand['joint_components']['J'+str(jid)]='m3joint_'+hand_name+'_j'+str(jid)
    config_dir=m3t.get_m3_config_path()+hand_name+'/'
    return config_dir, [hand]

def generate_hand_full():
    version=get_version(versions)
    hand_name=get_hand_name()
    limb_name=get_limb_name(limbs)
    pwr_name=get_pwr_name()
    config_dir=m3t.get_m3_config_path()+hand_name+'/'
    data=[]
    ndof=len(config_hand_def[version]['actuator_ec'])
    for jid in range(ndof):
        cd,x=generate_joint(hand_name,version,jid,pwr_name)
        data.extend(x)
    cd,x=generate_hand(hand_name,version, limb_name)
    data.extend(x)
    return config_dir,data
     
def generate_joint(hand_name=None,version=None, jid=None, pwr_name=None):
    if version is None:
        version=get_version(versions)
    if hand_name is None:
        hand_name=get_hand_name()
    if jid is None:
        jid=get_joint_id()
    if pwr_name is None:
        pwr_name=get_pwr_name()
    
    actuator_ec=copy.deepcopy(config_hand_def[version]['actuator_ec'][jid])
    actuator=copy.deepcopy(config_hand_def[version]['actuator'][jid])
    joint=copy.deepcopy(config_hand_def[version]['joint'][jid])
    name_actuator_ec='m3actuator_ec_'+hand_name+'_j'+str(jid)
    name_actuator='m3actuator_'+hand_name+'_j'+str(jid)
    name_joint='m3joint_'+hand_name+'_j'+str(jid)
    
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
    
    config_dir=m3t.get_m3_config_path()+hand_name+'/'

    return config_dir, [actuator_ec, actuator, joint]
    