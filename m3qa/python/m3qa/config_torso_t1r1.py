# -*- coding: utf-8 -*-
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

# ########################## ACTUATOR_EC A2.R3.J0-J6 ########################################

config_torso_t1r1_actuator_ec={
    'chid': 0,
    'ethercat': {'pdo_version': 'actx1_pdo_v1',
                 'product_code': 1001,
                 'serial_number': 0},
    'name': 'm3actuator_ec_xxx_j0',
    'version':'default',
    'firmware': 'm3_controller_sea_a1r1',
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

config_torso_t1r1_actuator_ec_j0=copy.deepcopy(config_torso_t1r1_actuator_ec)
config_torso_t1r1_actuator_ec_j1=copy.deepcopy(config_torso_t1r1_actuator_ec)

# ########################## ACTUATOR A2.R3.J0-J1 ########################################

config_torso_t1r1_actuator_j0={
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
                        'name': 'Maxon RE40 150W 24V 14887',
                        'nominal_voltage':24.0, #V
                        'no_load_speed':7580.0, #rpm
                        'no_load_current': 137.0, #mA
                        'nominal_speed': 6930.0, #rpm
                        'nominal_torque': 170.0, #mNm
                        'nominal_current': 5.77, #A
                        'stall_torque': 2280.0, #mNm
                        'starting_current':75.7, #A
                        'max_efficiency': 91.0, #%
                        'winding_resistance': 0.317,#Ohm
                        'winding_inductance': .0823,#mH
                        'torque_constant': 30.2,#mNm/A
                        'speed_constant':317.0,#rpm/V
                        'speed_torque_gradient':3.33,#rpm/mNm
                        'mechanical_time_constant':4.81,#ms
                        'rotor_inertia': 138,#gcm^2
                        'thermal_resistance_housing_ambient': 4.65,#K/W
                        'thermal_resistance_rotor_housing': 1.93,#K/W
                        'thermal_time_constant_winding': 41.6,#s
                        'thermal_time_constant_motor': 1120.0,#s
                        'max_winding_temp': 155.0,#C
                        'gear_ratio':120.0,
                        'amplifier_resistance':0.0136,#Ohm
                        'max_pwm_duty':3186.0,
                        'safe_thermal_pct': 0.90,
                        'i_scale': 1.1
                        },
              'ext_temp': {'cb_bias': 0.0,
                             'cb_mV_at_25C': 750.0,
                             'cb_mV_per_C': 10.0,
                             'cb_scale': 1.0,
                             'name': 'Analog TMP36',
                             'type': 'adc_linear_3V3'},
              'theta': {'cb_bias': 0.0,
                        'cb_scale': 1.0,
                        'cb_theta':[0.087890625,0],
                        'name': 'US Digital MA13',
                        'type': 'ma3_12bit_poly'},
              'torque': {'cb_bias': 0.0,
                         'cb_inv_torque': [1.0,0.0],
                         'cb_scale': 1.0,
                         'cb_torque': [1.0,0.0],
                         'name': 'ContElec VertX13',
                         'type': 'sea_vertx_14bit'},
              'torquedot_df': {'cutoff_freq': 100,
                               'order': 3,
                               'type': 'diff_butterworth'}},
    'description': 'max2_v0.2_seax2_v1.2',
    'ec_component': 'm3actuator_ec_xxx_jx',
    'ignore_bounds': 0,
    'safe_pwm_limit': 1,
    'joint_component': 'm3joint_xxx_jx',
    'name': 'm3actuator_xxx_jx',
    'version':'iss',
    'param': {'max_overload_time': 3.0,
              'max_amp_temp': 125.0,
              'max_amp_current': 20000.0,
              'max_tq': 40000.0,
              'min_tq': -40000.0}}

config_torso_t1r1_actuator_j1=copy.deepcopy(config_torso_t1r1_actuator_j0)
config_torso_t1r1_actuator_j1['calib']['motor']['gear_ratio']=100.0

