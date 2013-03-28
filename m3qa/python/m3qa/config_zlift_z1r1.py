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

# ########################## ACTUATOR_EC ZLIFT ########################################

config_zlift_z1r1_actuator_ec_j0={
    'chid': 0,
    'ethercat': {'pdo_version': 'actx1_pdo_v1',
                 'product_code': 1010,
                 'serial_number': 0},
    'name': 'm3actuator_ec_xxx_j0',
    'firmware': 'm3_controller_elmo_z1r1',
    'version':'iss',
    'param': {'config': 2048, #enable calibration on negative limit switch
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
              'pwm_max': 1000,
              'qei_max': 0,
              'qei_min': 0,
              't_max': 1000,
              't_min': -1000},
    'pwr_component': 'm3pwr_pwr000'}


# ########################## ACTUATOR Z1.R1.J0 (ZLIFT) ########################################

config_zlift_z1r1_actuator_j0={
    'calib': {'amp_temp': {'type': 'none'},
              'angle_df': {'theta_df': {'type': 'identity'},#raw readings only
                           'thetadot_df': {'cutoff_freq': 20,
                                           'order': 3,
                                           'type': 'diff_butterworth'},
                           'thetadotdot_df': {'cutoff_freq': 20,
                                              'order': 3,
                                              'type': 'diff_butterworth'},
                           'type': 'df_chain'},
              'current': {'cb_bias': 0.0,
                          'cb_dac_mV_per_tick': 10000.0/2048.0,
                          'cb_amp_mA_per_mV': 20000.0/10000.0,
                          'cb_scale': 1.0,
                          'name': 'Elmo Ocarina 10/60',
                          'type': 'linear_amp_via_dac'},
              'torque': {'cb_bias': 0.0,
                         'cb_scale': 1.0,
                         'cb_mNm_per_mA':.1796,
                         'cb_mA_per_tick': (20000.0/10000.0)*(10000.0/2048.0),
                         'name': 'Elmo Ocarina 10/60',
                         'type': 'ff_current_ctrl'},
              'motor': {'model_type': 'model_v2',
                        'temp_sensor_type': 'ambient',
                        'current_sensor_type': 'controlled',
                        'name': 'TorqueMaster 3515-E',
                        'nominal_voltage':24.0, #V
                        'no_load_speed':1312.0, #rpm
                        'no_load_current': 188.6, #mA
                        'nominal_speed': 987.9, #rpm
                        'nominal_torque': 727.6, #mNm
                        'nominal_current': 4.05, #A
                        'stall_torque': 6361.0, #mNm
                        'starting_current':0.0, #A (Unknown)
                        'max_efficiency': 90.0, #% (Estimated)
                        'winding_resistance': 1.34,#Ohm
                        'winding_inductance': 0.0,#mH (Unknown)
                        'torque_constant': 179.6,#mNm/A
                        'speed_constant':53.2,#rpm/V
                        'speed_torque_gradient':4.85,#rpm/mNm
                        'mechanical_time_constant':3.9,#ms
                        'rotor_inertia': 10846.5,#gcm^2
                        'thermal_resistance_housing_ambient': 3.1,#K/W 
                        'thermal_resistance_rotor_housing': 0.0,#K/W (Unknown)
                        'thermal_time_constant_winding': 0.0,#s (Unknown)
                        'thermal_time_constant_motor': 0.0,#s (Unknown)
                        'max_winding_temp': 155.0,#C
                        'gear_ratio': 0.0,
                        'amplifier_resistance':0.0136,#Ohm (Estimated)
                        'max_pwm_duty':2048.0, #Not used as DAC amp
                        'safe_thermal_pct': 0.90,
                        'i_scale': 1.0
                        },
              'ext_temp': {'type': 'temp_25C'},
              'theta': {'cb_bias': 0.0,
                        'cb_scale': 1.0,
                        'cb_ticks_per_rev':8000.0,
                        'name': 'DataTorque 14414-671',
                        'type': 'qei'},
              'torquedot_df': {'cutoff_freq': 100,
                               'order': 3,
                               'type': 'diff_butterworth'}},
    'description': 'm3_elmo_v0_1_ocarina_10/60',
    'ec_component': 'm3actuator_ec_xxx_jx',
    'ignore_bounds': 0,
    'safe_pwm_limit': 1,
    'encoder_calib_req': 1,
    'joint_component': 'm3joint_xxx_jx',
    'name': 'm3actuator_xxx_jx',
    'version':'iss',
    'param': {'max_overload_time': 3.0,
              'max_amp_temp': 125.0,
              'max_amp_current': 10000.0,
              'max_tq': 1500.0,
              'min_tq': -1500.0}}


# ########################## ZLIFT Z1.R1 J0 ########################################

config_zlift_z1r1_joint_j0={
    'actuator_component': 'm3actuator_xxx_j0', 
    'name': 'm3joint_xxx_j0',
    'version':'iss',
    'brake': 'auto',
    'calib': {'cb_payload_mass': 1.0, #Kg
              'cb_gearing': 5.0,
              'cb_screw_pitch': 30.0, #mm/rev
              'cb_screw_efficiency': 0.90},
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
                    'qs_to_qj':[1.0],
                    'tqj_to_tqa':[1.0],
                    'tqs_to_tqj':[1.0],
                    'type': 'gear'}}


config_zlift_full_z1r1={'actuator_ec':[config_zlift_z1r1_actuator_ec_j0],
                  'actuator':[config_zlift_z1r1_actuator_j0],
                  'joint':[config_zlift_z1r1_joint_j0]}




