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
from m3qa.config_toolbox import *

# ############## BMW V0.3 in A2.R2 systems ##############
config_loadx6_ec_a2r2={
    'ethercat': {'pdo_version': 'loadx6_pdo_v1',
                 'product_code': 1003,
                 'serial_number': -1},
    'name': 'm3loadx6_ec_ma*_l0',
    'version':'iss',
    'param': {'config': 0}}

config_loadx6_ec_a2r3={
    'ethercat': {'pdo_version': 'loadx6_pdo_v1',
                 'product_code': 1003,
                 'serial_number': -1},
    'name': 'm3loadx6_ec_ma*_l0',
    'version':'iss',
    'firmware': 'm3_controller_loadx6_a2r3',
    'param': {'config': 0}}

config_loadx6_a2r2={ 'description': 'BMW LoadX6',
                        'ec_component': 'm3loadx6_ec_ma*_l0',
                        'name': 'm3loadx6_ma*_l0',
                        'version':'iss',
                        'calib':{
                                'wrench':{
                                        'serial_number':'FTXXXXX',
                                        'type': '6x6_linear',
                                        'name': 'ATI MINI-40',
                                        'cb_fx': [1,0,0, 0,0,0],
                                        'cb_fy': [0,1,0, 0,0,0],
                                        'cb_fz': [0,0,1, 0,0,0],
                                        'cb_tx': [0,0,0, 1,0,0],
                                        'cb_ty': [0,0,0, 0,1,0],
                                        'cb_tz': [0,0,0, 0,0,1],
                                        'cb_adc_bias': [0,0,0, 0,0,0],
                                        'cb_scale': 1.0,
                                        'cb_bias': [0,0,0, 0,0,0]},
                                'wrench_df': {
                                        'cutoff_freq': 20,
                                        'order': 3,
                                        'type': 'butterworth'}
                        }}

config_loadx6_a2r3={ 'description': 'BMW LoadX6',
                        'ec_component': 'm3loadx6_ec_ma*_l0',
                        'name': 'm3loadx6_ma*_l0',
                        'version':'iss',
                        'calib':{
                                'wrench':{
                                        'serial_number':'FTXXXXX',
                                        'type': '6x6_linear',
                                        'name': 'ATI MINI-40',
                                        'cb_fx': [1,0,0, 0,0,0],
                                        'cb_fy': [0,1,0, 0,0,0],
                                        'cb_fz': [0,0,1, 0,0,0],
                                        'cb_tx': [0,0,0, 1,0,0],
                                        'cb_ty': [0,0,0, 0,1,0],
                                        'cb_tz': [0,0,0, 0,0,1],
                                        'cb_adc_bias': [0,0,0, 0,0,0],
                                        'cb_scale': 1.0,
                                        'cb_bias': [0,0,0, 0,0,0]},
                                'wrench_df': {
                                        'cutoff_freq': 20,
                                        'order': 3,
                                        'type': 'butterworth'}
                        }}

# ##############################################################
        
def generate():
        ec=copy.deepcopy(config_loadx6_ec_a2r3)
        rt=copy.deepcopy(config_loadx6_a2r3)
        arm_name=get_arm_name()
        name_ec='m3loadx6_ec_'+arm_name+'_l0'
        name_rt='m3loadx6_'+arm_name+'_l0'
        print 'Enter Serial Number of EC board (e.g. 300)'
        x=m3t.get_int()
        print 'Enter Serial Number of Loadcell (e.g. 3333 for FT333)'
        y=m3t.get_int()
        rt['calib']['wrench']['serial_number']
        ec['ethercat']['serial_number']='FT'+str(y)
        ec['name']=name_ec
        rt['name']=name_rt
        rt['ec_component']=name_ec
        config_dir=m3t.get_m3_config_path()+arm_name+'/'
        return config_dir, [ec, rt]
    