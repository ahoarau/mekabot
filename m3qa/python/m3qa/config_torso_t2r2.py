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

import copy
import config_toolbox as ct
# ########################## ACTUATOR_EC T2.R2.J0-J1 ########################################

config_torso_t2r2_actuator_ec={
    'chid': 0,
    'ethercat': {'pdo_version': 'actx1_pdo_v1',
                 'product_code': 1010,
                 'serial_number': 0},
    'name': 'm3actuator_ec_xxx_j0',
    'version':'iss',
    'param': {'config': 2,
              'k_d': 0,
              'k_d_shift': 0,
              'k_ff': 0,
              'k_ff_shift': 0,
              'k_ff_zero': 0,
              'k_i': 0,
              'k_i_limit': 0,
              'k_i_shift': 0,
              'k_p': 0,
              'k_p_shift': 0,
              'pwm_db': 0,
              'pwm_max': 3186,
              'qei_max': 0,
              'qei_min': 0,
              't_max': 4096,
              't_min': 0},
    'pwr_component': 'm3pwr_pwr000'}

config_torso_t2r2_actuator_ec_j0=config_torso_t2r2_actuator_ec
config_torso_t2r2_actuator_ec_j1=config_torso_t2r2_actuator_ec

# ########################## ACTUATOR T2.R2.J0 ########################################

config_torso_t2r2_actuator_j0={
    'calib': {'amp_temp': {'cb_bias': 0.0,
                           'cb_mV_at_25C': 750.0,
                           'cb_mV_per_C': 10.0,
                           'cb_scale': 1.0,
                           'name': 'Microchip TC1047',
                           'type': 'adc_linear_5V'},
              'angle_df': {'theta_df': {'cutoff_freq': 80,
                                        'order': 3,
                                        'type': 'butterworth'},
                           'thetadot_df': {'cutoff_freq': 20,
                                           'order': 3,
                                           'type': 'diff_butterworth'},
                           'thetadotdot_df': {'cutoff_freq': 20,
                                              'order': 3,
                                              'type': 'diff_butterworth'},
                           'type': 'df_chain'},
              'current': {'cb_bias': 0.0,
                          'cb_mV_per_A': 66.0,
                          'cb_scale': 1.0,
                          'cb_ticks_at_zero_a': 2048,
                          'cb_ticks_at_zero_b': 2048,
                          'name': 'Allegro ACS712-30',
                          'type': 'adc_linear_5V'},
              'motor': {'model_type': 'model_v1',
                        'temp_sensor_type': 'ambient',
                        'current_sensor_type': 'measured',
                        'name': 'Maxon EC45 Flat 50W 24V 339286',
                        'nominal_voltage':24.0, #V
                        'no_load_speed':4700.0, #rpm
                        'no_load_current': 116.0, #mA
                        'nominal_speed': 3410.0, #rpm
                        'nominal_torque': 64.2, #mNm
                        'nominal_current': 1.32, #A
                        'stall_torque': 337.0, #mNm
                        'starting_current':7.10, #A
                        'max_efficiency': 77.0, #%
                        'winding_resistance': 3.38,#Ohm
                        'winding_inductance': 1.15,#mH
                        'torque_constant': 47.5,#mNm/A
                        'speed_constant':201.0,#rpm/V
                        'speed_torque_gradient':14.3,#rpm/mNm
                        'mechanical_time_constant':20.3,#ms
                        'rotor_inertia': 135,#gcm^2
                        'thermal_resistance_housing_ambient': 4.25,#K/W
                        'thermal_resistance_rotor_housing': 4.5,#K/W
                        'thermal_time_constant_winding': 16.6,#s
                        'thermal_time_constant_motor': 212.0,#s
                        'max_winding_temp': 125.0,#C
                        'gear_ratio':120.0,
                        'amplifier_resistance':0.0136,#Ohm
                        'max_pwm_duty':3186.0
                        },
              'ext_temp': {'type': 'temp_25C'},
              'theta': {'cb_bias': 0.0,
                        'cb_scale': 1.0,
                        'name': 'ContElec VertX13',
                        'type': 'vertx_14bit'},
              'torque': {'cb_bias': 0.0,
                         'cb_inv_torque': [1.0,0.0],
                         'cb_scale': 1.0,
                         'cb_torque': [1.0,0.0],
                         'description':'TRT-500',
                         'name': 'Linear torque-load cell',
                         'type': 'adc_poly'},
              'torquedot_df': {'cutoff_freq': 100,
                               'order': 3,
                               'type': 'diff_butterworth'}},
    'description': 'max2_v0.2_seax2_v1.2',
    'ec_component': 'm3actuator_ec_xxx_jx',
    'ignore_bounds': 0,
    'joint_component': 'm3joint_xxx_jx',
    'version':'iss',
    'name': 'm3actuator_xxx_jx',
    'param': {'max_overload_time': 3.0,
              'max_amp_temp': 125.0,
              'max_amp_current': 10000.0,
              'max_tq': 25000.0,
              'min_tq': -25000.0}}

# ########################## ACTUATOR T2.R2.J1 ########################################

config_torso_t2r2_actuator_j1={
    'calib': {'amp_temp': {'cb_bias': 0.0,
                           'cb_mV_at_25C': 750.0,
                           'cb_mV_per_C': 10.0,
                           'cb_scale': 1.0,
                           'name': 'Microchip TC1047',
                           'type': 'adc_linear_5V'},
              'angle_df': {'theta_df': {'cutoff_freq': 80,
                                        'order': 3,
                                        'type': 'butterworth'},
                           'thetadot_df': {'cutoff_freq': 20,
                                           'order': 3,
                                           'type': 'diff_butterworth'},
                           'thetadotdot_df': {'cutoff_freq': 20,
                                              'order': 3,
                                              'type': 'diff_butterworth'},
                           'type': 'df_chain'},
              'current': {'cb_bias': 0.0,
                          'cb_mV_per_A': 66.0,
                          'cb_scale': 1.0,
                          'cb_ticks_at_zero_a': 2048,
                          'cb_ticks_at_zero_b': 2048,
                          'name': 'Allegro ACS712-30',
                          'type': 'adc_linear_5V'},
              'motor': {'model_type': 'model_v1',
                        'temp_sensor_type': 'ambient',
                        'current_sensor_type': 'measured',
                        'name': 'Maxon RE50 200W 24V 370354',
                        'nominal_voltage':24.0, #V
                        'no_load_speed':5778.0, #rpm
                        'no_load_current': 248.0, #mA
                        'nominal_speed': 5538.0, #rpm
                        'nominal_torque': 349.0, #mNm
                        'nominal_current': 9.04, #A
                        'stall_torque': 8416, #mNm
                        'starting_current':212.0, #A
                        'max_efficiency': 93.0, #%
                        'winding_resistance': .113,#Ohm
                        'winding_inductance': .094,#mH
                        'torque_constant': 39.6,#mNm/A
                        'speed_constant':241.0,#rpm/V
                        'speed_torque_gradient':.687,#rpm/mNm
                        'mechanical_time_constant':4.20,#ms
                        'rotor_inertia': 584.0,#gcm^2
                        'thermal_resistance_housing_ambient': 3.81,#K/W
                        'thermal_resistance_rotor_housing': 2.27,#K/W
                        'thermal_time_constant_winding': 137.0,#s
                        'thermal_time_constant_motor': 406.0,#s
                        'max_winding_temp': 125.0,#C
                        'gear_ratio':512.0,
                        'amplifier_resistance':0.0136,#Ohm
                        'max_pwm_duty':3186.0
                        },
              'ext_temp': {'type': 'temp_25C'},
              'theta': {'cb_bias': 0.0,
                        'cb_scale': 1.0,
                        'name': 'ContElec VertX13',
                        'type': 'vertx_14bit'},
              'torque': {'cb_bias': 0.0,
                         'cb_inv_torque': [1.0,0.0],
                         'cb_scale': 1.0,
                         'cb_torque': [1.0,0.0],
                         'description':'TRS-1K',
                         'name': 'Linear torque-load cell',
                         'type': 'adc_poly'},
              'torquedot_df': {'cutoff_freq': 100,
                               'order': 3,
                               'type': 'diff_butterworth'}},
    'description': 'max2_v0.2_seax2_v1.2',
    'ec_component': 'm3actuator_ec_xxx_jx',
    'ignore_bounds': 0,
    'joint_component': 'm3joint_xxx_jx',
    'version':'iss',
    'name': 'm3actuator_xxx_jx',
    'param': {'max_overload_time': 3.0,
              'max_amp_temp': 125.0,
              'max_amp_current': 10000.0,
              'max_tq': 200000.0,
              'min_tq': -200000.0}}

# ########################## JOINT T2.R2.J0-J1 ########################################

config_torso_t2r2_joint_j0={
    'actuator_component': 'm3actuator_xxx_j0', 
    'name': 'm3joint_xxx_j0',
    'version':'iss',
    'brake': 'none',
    'param': {'kq_d': 0.0,
              'kq_g': 1.0,
              'kq_i': 0.0,
              'kq_i_limit': 0.0,
              'kq_i_range': 0.0,
              'kq_p': 0.0,
              'kt_d': 0.0,
              'kt_i': 0.0,
              'kt_i_limit': 0.0,
              'kt_i_range': 0.0,
              'kt_p': 0.0,
              'max_q': 360.0,
              'max_q_pad': 0.0,
              'max_q_slew_rate': 25.0,
              'min_q': 0.0,
              'min_q_pad': 0.0},
    'transmission':{'act_name': 'm3actuator_xxx_j0',
                    'qj_to_qa':[1.0],
                    'qs_to_qj':[45.0/60.0], #gearing on encoder
                    'tqj_to_tqa':[1.0],
                    'tqs_to_tqj':[1.0],
                    'type': 'gear'}}

config_torso_t2r2_joint_j1=copy.deepcopy(config_torso_t2r2_joint_j0)
config_torso_t2r2_joint_j1['transmission']['qs_to_qj']=[1.0]
config_torso_t2r2_joint_j1['brake']='auto'

# ########################## JOINT T2.R2.J2 ########################################

config_torso_t2r2_joint_j2={
'name': 'm3joint_slave_mtx_j2',
'cpj_component': 'm3joint_mtx_j1',
'calib': {'cb_ms_ratio': 1.0},
'param':{'max_q': 29.0,
         'min_q': -29.0,
         'max_q_slew_rate': 25.0}}

# ########################## TORSO T2.R2 ########################################

config_torso_t2r2={
    'name': 'm3torso_xxx',
    'ndof': 3,
    'limb_name': 'torso',
    'dynamatics_component': 'm3dynamatics_xxx',
    'version':'iss',
'joint_components':{'J0': 'm3joint_xxx_j0',
                    'J1': 'm3joint_xxx_j1',
                    'J2': 'm3joint_slave_xxx_j2'}}

 # ########################## Dynamatics Torso T2.R2 w/o Link3  #######
t2r2_dynamatics={
    'chain_component': 'm3torso_xxx',
    'links': [{'Ixx': 0.03898206, #link 1
               'Ixy': -0.00000043,#
               'Ixz': -0.00007909,#
               'Iyy': 0.02598195,#
               'Iyz': -0.00023046,#
               'Izz': 0.02305461,#
               'a': 0.0,#
               'alpha': 0.0,#
               'cx': 0.00011774,#
               'cy': 0.00380734,#
               'cz': -0.05227532,#
               'd': .167875,#
               'joint_offset': 0,#
               'm': 4.411},#
              {'Ixx': 0.02596625, #link 2
               'Ixy': 0.00084615,#
               'Ixz': 0.00153073,#
               'Iyy': 0.05453951,#
               'Iyz': 0.00011260,#
               'Izz': 0.03939563,#
               'a': 0.0,#
               'alpha': -90.0,#
               'cx': 0.04625888,#
               'cy': 0.00117857,#
               'cz': -0.00299276,#
               'd': 0.0,#
               'joint_offset': 270.0,#
               'm': 4.934},#
              {'Ixx': 0, #link 3
               'Ixy': 0,#
               'Ixz': 0,#
               'Iyy': 0,#
               'Iyz': 0,#
               'Izz': 0,#
               'a': 0.1397,#
               'alpha': 0,#
               'cx': 0,#
               'cy': 0,#
               'cz': 0,#
               'd': 0.0,#
               'joint_offset': 0.0,#
               'm': 0.0},#
              {'a': .233738,#
               'alpha': 0.0,#
               'd': 0,#
               'joint_offset': 0.0,#
               'm': 0}],#
    'name': 'm3dynamatics_xxx',
    'ndof': 3,
    'param': {'payload_com': [0.0, 0.0, 0.0], #payload is a head
              'payload_inertia': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
              'payload_mass': 0.0,
              'use_accelerations': False,
              'use_velocities': False}} 

# ########################## Dynamatics Torso T2.R2 with Bimanual A2.R2 ##########
t2r2_dynamatics_bimanual_a2r2=copy.deepcopy(t2r2_dynamatics)
t2r2_dynamatics_bimanual_a2r2['links'][2]={
 'Ixx':  0.05181, #link 3
 'Ixy': 0.00276,#
 'Ixz': -0.00001,#
 'Iyy': 0.24780,#
 'Iyz': 0.00001,#
 'Izz':  0.22181,#
 'a': 0.1397,#
 'alpha': 0,#
 'cx':  0.13946,#
 'cy': 0.00134,#
 'cz': 0.00000,#
 'd': 0.0,#
 'joint_offset': 0.0,#
 'm': 8.493} #
# ########################## Dynamatics Torso T2.R2 with right-arm A2.R2 ##########
t2r2_dynamatics_a2r2_right=copy.deepcopy(t2r2_dynamatics)
t2r2_dynamatics_a2r2_right['links'][2]={
 'Ixx': 0.04188, #link 3
 'Ixy': 0.00838,#
 'Ixz': 0.01441,#
 'Iyy': 0.17534,#
 'Iyz': 0.00018,#
 'Izz': 0.15324,#
 'a': 0.1397,#
 'alpha': 0,#
 'cx': 0.12974,#
 'cy': 0.00870,#
 'cz': 0.01147,#
 'd': 0.0,#
 'joint_offset': 0.0,#
 'm': 6.245} # 
# ########################## Payloads T2.R2 ########################################

  
t2r2_payload_none={
    'payload_com': [0.0, 0.0, 0.0],
    'payload_inertia': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    'payload_mass': 0.0
    }


t2r2_payload_s2r1={
    'payload_com': [0.06577595, 0.00058848, 0.00757901],
    'payload_inertia': [0.00052085, 0.00000700, 0.00016131, 0.00182462, 0.00000010, 0.00149477],
    'payload_mass': 0.289
    }

t2r2_dynamatics_a2r2_right=copy.deepcopy(t2r2_dynamatics_a2r2_right)
t2r2_dynamatics_a2r2_right_s2r1=copy.deepcopy(t2r2_dynamatics_a2r2_right)
t2r2_dynamatics_bimanual_a2r2=copy.deepcopy(t2r2_dynamatics_bimanual_a2r2)
t2r2_dynamatics_bimanual_a2r2_s2r1=copy.deepcopy(t2r2_dynamatics_bimanual_a2r2)

ct.set_payload(t2r2_dynamatics_a2r2_right, t2r2_payload_none)
ct.set_payload(t2r2_dynamatics_a2r2_right_s2r1, t2r2_payload_s2r1)
ct.set_payload(t2r2_dynamatics_bimanual_a2r2, t2r2_payload_none)
ct.set_payload(t2r2_dynamatics_bimanual_a2r2_s2r1, t2r2_payload_s2r1)

# ########################## T2.R2 ########################################

config_full_t2r2={'actuator_ec':[config_torso_t2r2_actuator_ec_j0,
                                 config_torso_t2r2_actuator_ec_j1],
                  'actuator':[config_torso_t2r2_actuator_j0,
                              config_torso_t2r2_actuator_j1],
                  'joint':[config_torso_t2r2_joint_j0,
                           config_torso_t2r2_joint_j1,
                           config_torso_t2r2_joint_j2,],
                  'dynamatics':{'a2r2_right':t2r2_dynamatics_a2r2_right,
                                'a2r2_right_s2r1':t2r2_dynamatics_a2r2_right_s2r1,
                                'bimanual_a2r2': t2r2_dynamatics_bimanual_a2r2,
                                'bimanual_a2r2_s2r1': t2r2_dynamatics_bimanual_a2r2_s2r1},
                  'torso':config_torso_t2r2}
