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

# ############## V0.3 in A2.R1 systems ##############
config_pwr_ec_v0_3={
    'ethercat': {'pdo_version': 'pwr_pdo_v1',
                 'product_code': 1002,
                 'serial_number': -1},
    'name': 'm3pwr_ec_pwr000',
    'param': {'config': 0}}

config_pwr_v0_3={
    'calib': {'current': {'cb_bias': 0.0,
                          'cb_mV_per_A': 185.0,
                          'cb_scale': 1.0,
                          'cb_ticks_at_zero_a': 2048.0,
                          'cb_ticks_at_zero_b': 2048.0,
                          'name': 'Allegro ACS712-05',
                          'type': 'adc_linear_5V'},
              'voltage': {'cb_bias': 0.0,
                          'cb_inv_voltage':[1.0,0.0],
                          'cb_scale': 1.0,
                          'cb_voltage':[1.0,0.0],
                          'type': 'adc_poly'}},
    'ec_component': 'm3pwr_ec_pwr000',
    'ignore_bounds': 0,
    'name': 'm3pwr_pwr000',
    'param': {'max_bus_voltage': 36.0,
              'max_current_digital': 5000,
              'min_bus_voltage': 18.0
              }}

# ############## V0.4 in A2.R2 systems ##############
config_pwr_ec_v0_4={
    'ethercat': {'pdo_version': 'pwr_pdo_v2',
                 'product_code': 1002,
                 'serial_number': -1},
    'name': 'm3pwr_ec_pwr000',
    'version':'iss',
    'param': {'config': 0}}

config_pwr_v0_4={
    'calib': {'current': {'cb_bias': 0.0,
                          'cb_mV_per_A': -185.0,
                          'cb_scale': 1.0,
                          'cb_ticks_at_zero_a': 2048.0,
                          'cb_ticks_at_zero_b': 2048.0,
                          'name': 'Allegro ACS712-05',
                          'type': 'adc_linear_5V'},
              'voltage': {'cb_bias': 0.0,
                          'cb_inv_voltage':[1.0,0.0],
                          'cb_scale': 1.0,
                          'cb_voltage':[1.0,0.0],
                          'type': 'adc_poly'}},
    'ec_component': 'm3pwr_ec_pwr000',
    'ignore_bounds': 0,
    'name': 'm3pwr_pwr000',
    'version':'iss',
    'param': {'max_bus_voltage': 36.0,
              'max_current_digital': 5000,
              'min_bus_voltage': 18.0
              }}
# ############## V0.5 in A2.R3 systems ##############
config_pwr_ec_v0_5={
    'ethercat': {'pdo_version': 'pwr_pdo_v2',
                 'product_code': 1002,
                 'serial_number': -1},
    'name': 'm3pwr_ec_pwr000',
    'version':'iss',
    'param': {'config': 0}}

config_pwr_v0_5={
    'calib': {'current': {'cb_bias': 0.0,
                          'cb_mV_per_A': -185.0,
                          'cb_scale': 1.0,
                          'cb_ticks_at_zero_a': 2048.0,
                          'cb_ticks_at_zero_b': 2048.0,
                          'name': 'Allegro ACS712-05',
                          'type': 'adc_linear_5V'},
              'voltage': {'cb_bias': 0.0,
                          'cb_inv_voltage':[1.0,0.0],
                          'cb_scale': 1.0,
                          'cb_voltage':[1.0,0.0],
                          'type': 'adc_poly'}},
    'ec_component': 'm3pwr_ec_pwr000',
    'ignore_bounds': 0,
    'name': 'm3pwr_pwr000',
    'version':'iss',
    'firmware': 'm3_controller_pwr_0_5',
    'param': {'max_bus_voltage': 36.0,
              'max_current_digital': 5000,
              'min_bus_voltage': 18.0
              }}
# ##############################################################

def get_version():
    versions=['0.3','0.4','0.5'] 
    while True:
        print 'Enter version'
        for i in range(len(versions)):
            print i,' : ',versions[i]
        idd=m3t.get_int()
        if idd<len(versions):
            return versions[idd]
        print 'Incorrect value'
        
def generate():
    version=get_version()
    if version=='0.3':
        ec=copy.deepcopy(config_pwr_ec_v0_3)
        rt=copy.deepcopy(config_pwr_v0_3)
    if version=='0.4':
        ec=copy.deepcopy(config_pwr_ec_v0_4)
        rt=copy.deepcopy(config_pwr_v0_4)
    if version=='0.5':
        ec=copy.deepcopy(config_pwr_ec_v0_5)
        rt=copy.deepcopy(config_pwr_v0_5)
    print 'Enter PWR board ID (e.g., 10 for pwr010)'
    x=m3t.get_int()
    name_ec='m3pwr_ec_pwr'+'0'*(3-len(str(x)))+str(x)
    name_rt='m3pwr_pwr'+'0'*(3-len(str(x)))+str(x)
    print 'Enter Serial Number of EC board (e.g. 300)'
    x=m3t.get_int()
    ec['ethercat']['serial_number']=x
    ec['name']=name_ec
    rt['name']=name_rt
    rt['ec_component']=name_ec
    config_dir=m3t.get_m3_config_path()+'pwr/'
    return config_dir, [ec, rt]
    