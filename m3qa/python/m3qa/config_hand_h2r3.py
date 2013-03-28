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

# ########################## ACTUATOR_EC H2.R3 J0 ########################################

config_hand_h2r3_actuator_ec_j0j1={
    'chid': 0,
    'version':'esp',
    'firmware': 'm3_controller_hb2_h2r3_j0j1',
    'ethercat': {'pdo_version': 'actx2_pdo_v1',
                 'product_code': 1011,
                 'serial_number': 0},
    'name': 'm3actuator_ec_xxx_j0',
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
              'pwm_max': 2000, #protect motor against over-temp
              'qei_max': 0,
              'qei_min': 0,
              't_max': 4096,
              't_min': 0},
    'pwr_component': 'm3pwr_pwr000'}

config_hand_h2r3_actuator_ec_j2j3j4={
    'chid': 0,
    'version':'esp',
    'firmware': 'm3_controller_hb2_h2r3_j2j3j4',
    'ethercat': {'pdo_version': 'actx3_pdo_v1',
                 'product_code': 1012,
                 'serial_number': 0},
    'name': 'm3actuator_ec_xxx_j0',
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
              'pwm_max': 2000, #protect motor against over-temp
              'qei_max': 0,
              'qei_min': 0,
              't_max': 4096,
              't_min': 0},
    'pwr_component': 'm3pwr_pwr000'}

config_hand_h2r3_actuator_ec_j0=copy.deepcopy(config_hand_h2r3_actuator_ec_j0j1)
config_hand_h2r3_actuator_ec_j1=copy.deepcopy(config_hand_h2r3_actuator_ec_j0j1)
config_hand_h2r3_actuator_ec_j2=copy.deepcopy(config_hand_h2r3_actuator_ec_j2j3j4)
config_hand_h2r3_actuator_ec_j3=copy.deepcopy(config_hand_h2r3_actuator_ec_j2j3j4)
config_hand_h2r3_actuator_ec_j4=copy.deepcopy(config_hand_h2r3_actuator_ec_j2j3j4)
config_hand_h2r3_actuator_ec_j0['chid']=0
config_hand_h2r3_actuator_ec_j1['chid']=1
config_hand_h2r3_actuator_ec_j2['chid']=0
config_hand_h2r3_actuator_ec_j3['chid']=1
config_hand_h2r3_actuator_ec_j4['chid']=2

# ########################## ACTUATOR H2.R3.J0 ########################################

config_hand_h2r3_actuator_j0={
    'calib': {'amp_temp': {'cb_bias': 0.0,
                           'cb_mV_at_25C': 750.0,
                           'cb_mV_per_C': 10.0,
                           'cb_scale': 1.0,
                           'name': 'Microchip TC1047',
                           'type': 'adc_linear_3V3'},
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
              'current': {'type': 'none'}, #None due to bldc magnets interference with sensor
              'motor': {'model_type': 'model_v1',
                        'temp_sensor_type': 'ambient',
                        'current_sensor_type': 'none',#None due to bldc magnets interference with sensor
                        'name': 'Maxon EC13 6W 24V 305194',
                        'nominal_voltage':24.0, #V
                        'no_load_speed':27200.0, #rpm
                        'no_load_current': 45.8, #mA
                        'nominal_speed': 19300.0, #rpm
                        'nominal_torque': 2.19, #mNm
                        'nominal_current': .321, #A
                        'stall_torque': 7.83, #mNm
                        'starting_current':0.976, #A
                        'max_efficiency': 62.0, #%
                        'winding_resistance': 24.6,#Ohm
                        'winding_inductance': 0.357,#mH
                        'torque_constant': 8.03,#mNm/A
                        'speed_constant':1190,#rpm/V
                        'speed_torque_gradient':3640,#rpm/mNm
                        'mechanical_time_constant':6.29,#ms
                        'rotor_inertia': .165,#gcm^2
                        'thermal_resistance_housing_ambient': 32.0,#K/W
                        'thermal_resistance_rotor_housing': 2.46,#K/W
                        'thermal_time_constant_winding': 0.72,#s
                        'thermal_time_constant_motor': 188.0,#s
                        'max_winding_temp': 155.0,#C
                        'gear_ratio':131.0,
                        'amplifier_resistance':0.62,#Ohm
                        'max_pwm_duty':3186.0,
                        'safe_thermal_pct': 0.75,
                        'i_scale': 1.1
                        },
              'ext_temp': {'cb_bias': 0.0,
                           'cb_mV_at_25C': 750.0,
                           'cb_mV_per_C': 10.0,
                           'cb_scale': 1.0,
                           'name': 'Microchip TC1047',
                           'type': 'adc_linear_3V3'},
              'theta': {'cb_bias': 0.0,
                        'cb_scale': 1.0,
                        'name': 'US Digital MA3',
                        'type': 'ma3_12bit'},
              'torque': {'cb_bias': 0.0,
                         'cb_inv_torque': [1.0,0.0],
                         'cb_scale': 1.0,
                         'cb_torque': [1.0,0.0],
                         'name': 'Panasonic Potentiometer',
                         'type': 'adc_poly'},
              'torquedot_df': {'cutoff_freq': 100,
                               'order': 3,
                               'type': 'diff_butterworth'}},
    'description': 'hb2dig_v0.2_hb2amp_v0.2',
    'ec_component': 'm3actuator_ec_xxx_jx',
    'ignore_bounds': 0,
    'safe_pwm_limit': 1,
    'joint_component': 'm3joint_xxx_jx',
    'name': 'm3actuator_xxx_jx',
    'version':'iss',
    'param': {'max_overload_time': 3.0,
              'max_amp_temp': 70.0,
              'max_amp_current': 1000.0,
              'max_tq': 200.0, #0.45Nm max intermittent torque on gearhead
              'min_tq': -200.0}}

# ########################## ACTUATOR H2.R3.J1-J4 ########################################

config_hand_h2r3_actuator_j1={
    'calib': {'amp_temp': {'cb_bias': 0.0,
                           'cb_mV_at_25C': 750.0,
                           'cb_mV_per_C': 10.0,
                           'cb_scale': 1.0,
                           'name': 'Microchip TC1047',
                           'type': 'adc_linear_3V3'},
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
                          'cb_mV_per_A': 185.0,
                          'cb_scale': 1.0,
                          'cb_ticks_at_zero_a': 3103,
                          'cb_ticks_at_zero_b': 0,
                          'name': 'Allegro ACS712-05',
                          'type': 'adc_linear_5V_ns'},
              'motor': {'model_type': 'model_v1',
                        'temp_sensor_type': 'ambient',
                        'current_sensor_type': 'none',
                        'name': 'Maxon REMax-13 2.5W 24V 203950',
                        'nominal_voltage':24.0, #V
                        'no_load_speed':11600.0, #rpm
                        'no_load_current': 3.55, #mA
                        'nominal_speed': 8080, #rpm
                        'nominal_torque': 2.66, #mNm
                        'nominal_current': 0.138, #A
                        'stall_torque': 8.85, #mNm
                        'starting_current':0.450, #A
                        'max_efficiency': 83.0, #%
                        'winding_resistance': 53.3,#Ohm
                        'winding_inductance': 1.31,#mH
                        'torque_constant': 19.6,#mNm/A
                        'speed_constant':486,#rpm/V
                        'speed_torque_gradient':1320,#rpm/mNm
                        'mechanical_time_constant':7.26,#ms
                        'rotor_inertia': 0.526,#gcm^2
                        'thermal_resistance_housing_ambient': 37.0,#K/W
                        'thermal_resistance_rotor_housing': 10.0,#K/W
                        'thermal_time_constant_winding': 6.93,#s
                        'thermal_time_constant_motor': 444.0,#s
                        'max_winding_temp': 85.0,#C
                        'gear_ratio':275.0,
                        'amplifier_resistance':0.62,#Ohm
                        'max_pwm_duty':3186.0,
                        'safe_thermal_pct': 0.80,
                        'i_scale': 1.1
                        },
              'ext_temp': {'cb_bias': 0.0,
                           'cb_mV_at_25C': 750.0,
                           'cb_mV_per_C': 10.0,
                           'cb_scale': 1.0,
                           'name': 'Microchip TC1047',
                           'type': 'adc_linear_3V3'},
              'theta': {'cb_bias': 0.0,
                        'cb_scale': 1.0,
                        'name': 'US Digital MA3',
                        'type': 'ma3_12bit'},
              'torque': {'cb_bias': 0.0,
                         'cb_inv_torque': [1.0,0.0],
                         'cb_scale': 1.0,
                         'cb_torque': [1.0,0.0],
                         'name': 'Panasonic Potentiometer',
                         'type': 'adc_poly'},
              'torquedot_df': {'cutoff_freq': 100,
                               'order': 3,
                               'type': 'diff_butterworth'}},
    'description': 'hb2dig_v0.2_hb2amp_v0.2',
    'ec_component': 'm3actuator_ec_xxx_jx',
    'ignore_bounds': 0,
    'safe_pwm_limit': 1,
    'joint_component': 'm3joint_xxx_jx',
    'name': 'm3actuator_xxx_jx',
    'version':'iss',
    'param': {'max_overload_time': 3.0,
              'max_amp_temp': 70.0,
              'max_amp_current': 1000.0,
              'max_tq': 300.0, #0.3Nm max continuous torque on gearhead
              'min_tq': -100.0}}

config_hand_h2r3_actuator_j2=config_hand_h2r3_actuator_j1
config_hand_h2r3_actuator_j3=config_hand_h2r3_actuator_j1
config_hand_h2r3_actuator_j4=config_hand_h2r3_actuator_j1

# ########################## JOINT H2.R3.J0-J4 ########################################

config_hand_h2r3_joint_jx={
    'actuator_component': 'm3actuator_xxx_j0', 
    'name': 'm3joint_xxx_j0',
    'version':'iss',
    'brake': 'none',
    'param': {'kq_d': 0.0,
              'kq_g': 0.0,
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
              'max_q_slew_rate': 200.0,
              'min_q': 0.0,
              'min_q_pad': 0.0},
    'transmission':{'act_name': 'm3actuator_xxx_j0',
                    'qj_to_qa':[1.0],
                    'qs_to_qj':[1.0],
                    'tqj_to_tqa':[1.0],
                    'tqs_to_tqj':[1.0],
                    'type': 'gear'}}

config_hand_h2r3_joint_j0=copy.deepcopy(config_hand_h2r3_joint_jx)
config_hand_h2r3_joint_j0['param']['max_q_slew_rate']=150
config_hand_h2r3_joint_j1=config_hand_h2r3_joint_jx
config_hand_h2r3_joint_j2=config_hand_h2r3_joint_jx
config_hand_h2r3_joint_j3=config_hand_h2r3_joint_jx
config_hand_h2r3_joint_j4=config_hand_h2r3_joint_jx

# ########################## HAND H2.R3 ########################################

config_hand_h2r3={
    'name': 'm3hand_xxx',
    'version':'iss',
    'ndof': 5,
    'limb_name': 'xxx_hand',
'joint_components':{'J0': 'm3joint_xxx_j0',
                    'J1': 'm3joint_xxx_j1',
                    'J2': 'm3joint_xxx_j2',
                    'J3': 'm3joint_xxx_j3',
                    'J4': 'm3joint_xxx_j4'}}

# ########################## H2.R3 ########################################

config_full_h2r3={'actuator_ec':[config_hand_h2r3_actuator_ec_j0,
                                 config_hand_h2r3_actuator_ec_j1,
                                 config_hand_h2r3_actuator_ec_j2,
                                 config_hand_h2r3_actuator_ec_j3,
                                 config_hand_h2r3_actuator_ec_j4],
                  'actuator':[config_hand_h2r3_actuator_j0,
                              config_hand_h2r3_actuator_j1,
                              config_hand_h2r3_actuator_j2,
                              config_hand_h2r3_actuator_j3,
                              config_hand_h2r3_actuator_j4],
                  'joint':[config_hand_h2r3_joint_j0,
                           config_hand_h2r3_joint_j1,
                           config_hand_h2r3_joint_j2,
                           config_hand_h2r3_joint_j3,
                           config_hand_h2r3_joint_j4],
                  'hand':config_hand_h2r3}
