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
# ########################## ACTUATOR_EC S2.R1 J0-J6 ########################################

config_head_s2r2_actuator_ec={
 'chid': 0,
 'ethercat': {'pdo_version': 'actx2_pdo_v1',
              'product_code': 1011,
              'serial_number': 0},
 'name': 'm3actuator_ec_xxx_j0',
 'version':'iss',
 'firmware': 'm3_controller_hex4_s2r2',
 'param': {'config': 2,
           'k_d': 0.0,
           'k_d_shift': 0,
           'k_ff': 0.0,
           'k_ff_shift': 0,
           'k_ff_zero': 0.0,
           'k_i': 0.0,
           'k_i_limit': 0.0,
           'k_i_shift': 0,
           'k_p': 0.0,
           'k_p_shift': 0,
           'pwm_db': 0,
           'pwm_max': 3186,
           'qei_max': 0,
           'qei_min': 0,
           't_max': 0,
           't_min': 0},
 'pwr_component': 'm3pwr_pwr000'}

config_head_s2r2_actuator_ec_j0=copy.deepcopy(config_head_s2r2_actuator_ec)
config_head_s2r2_actuator_ec_j1=copy.deepcopy(config_head_s2r2_actuator_ec)
config_head_s2r2_actuator_ec_j2=copy.deepcopy(config_head_s2r2_actuator_ec)
config_head_s2r2_actuator_ec_j3=copy.deepcopy(config_head_s2r2_actuator_ec)
config_head_s2r2_actuator_ec_j4=copy.deepcopy(config_head_s2r2_actuator_ec)
config_head_s2r2_actuator_ec_j5=copy.deepcopy(config_head_s2r2_actuator_ec)
config_head_s2r2_actuator_ec_j6=copy.deepcopy(config_head_s2r2_actuator_ec)
config_head_s2r2_actuator_ec_j7_ens_eyelids=copy.deepcopy(config_head_s2r2_actuator_ec)


config_head_s2r2_actuator_ec_j1['chid']=1
config_head_s2r2_actuator_ec_j3['chid']=1
config_head_s2r2_actuator_ec_j5['chid']=1
config_head_s2r2_actuator_ec_j7_ens_eyelids['chid']=1

# ########################## ACTUATOR S2.R1.J0, J1, J2, J3 ########################################

config_head_s2r2_actuator_j0={
 'calib': { 'amp_control_input': 'pwm',
			'amp_temp': {'cb_bias': 0.0,
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
           'current': {'type': 'none'},
           'motor': {'model_type': 'model_v1',
                     'temp_sensor_type': 'ambient',
                     'current_sensor_type': 'none',
                     'name': 'Maxon EC32 Flat 15W 24V 267121',
                     'nominal_voltage':24.0, #V
                     'no_load_speed':4390.0, #rpm
                     'no_load_current': 73.40, #mA
                     'nominal_speed': 2800.0, #rpm
                     'nominal_torque': 23.2, #mNm
                     'nominal_current': 0.49, #A
                     'stall_torque': 85.0, #mNm
                     'starting_current':1.75, #A
                     'max_efficiency': 64.0, #%
                     'winding_resistance': 13.7,#Ohm
                     'winding_inductance': 7.73,#mH
                     'torque_constant': 49.0,#mNm/A
                     'speed_constant': 195.0,#rpm/V
                     'speed_torque_gradient': 54.5,#rpm/mNm
                     'mechanical_time_constant': 20.0,#ms
                     'rotor_inertia': 35.0,#gcm^2
                     'thermal_resistance_housing_ambient': 9.74,#K/W
                     'thermal_resistance_rotor_housing': 4.63,#K/W
                     'thermal_time_constant_winding': 8.1,#s
                     'thermal_time_constant_motor': 108.0,#s
                     'max_winding_temp': 125.0,#C
                     'gear_ratio':100.0,
                     'amplifier_resistance':0.019,#Ohm
                     'max_pwm_duty':3186.0,
                     'safe_thermal_pct': 0.80,
                     'i_scale': 1.0
                     },
           'ext_temp': {'cb_bias': 0.0,
                        'cb_mV_at_25C': 750.0,
                        'cb_mV_per_C': 10.0,
                        'cb_scale': 1.0,
                        'name': 'Analog TMP36',
                        'type': 'adc_linear_3V3'},
           'theta': {'cb_bias': 0.0,
                     'cb_scale': 1.0,
                     'name': 'ContElec VertX13',
                     'type': 'vertx_14bit'},
           'torque': {'type': 'none'},
           'torquedot_df': {'cutoff_freq': 100,
                            'order': 3,
                            'type': 'diff_butterworth'}},
 'description': 'm3_hex4_v1.0',
 'ec_component': 'm3actuator_ec_xxx_jx',
 'ignore_bounds': 0,
 'safe_pwm_limit': 1,
 'joint_component': 'm3joint_xxx_jx',
 'version':'iss',
 'name': 'm3actuator_xxx_jx',
 'param': {'max_overload_time': 3.0,
           'max_amp_temp': 100.0,
           'max_amp_current': 8800.0,
           'max_tq': 0.0,
           'min_tq': 0.0}}

config_head_s2r2_actuator_j1=copy.deepcopy(config_head_s2r2_actuator_j0);
config_head_s2r2_actuator_j2=copy.deepcopy(config_head_s2r2_actuator_j0);
config_head_s2r2_actuator_j3=copy.deepcopy(config_head_s2r2_actuator_j0);

# ########################## ACTUATOR S2.R1.J4, J5, J6 ########################################

config_head_s2r2_actuator_j4=copy.deepcopy(config_head_s2r2_actuator_j0);
config_head_s2r2_actuator_j5=copy.deepcopy(config_head_s2r2_actuator_j0);
config_head_s2r2_actuator_j6=copy.deepcopy(config_head_s2r2_actuator_j0);
config_head_s2r2_actuator_j4['calib']['motor']['gear_ratio']=12.5
config_head_s2r2_actuator_j5['calib']['motor']['gear_ratio']=12.5
config_head_s2r2_actuator_j6['calib']['motor']['gear_ratio']=18.3333333

# ########################## ACTUATOR S2.R1.J7-ENS-Eyelids ########################################

config_head_s2r2_actuator_j7_ens_eyelids={
 'calib': { 'amp_control_input': 'pwm',
			'amp_temp': {'cb_bias': 0.0,
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
           'current': {'type': 'none'},
           'motor': {'model_type': 'model_v1',
                     'temp_sensor_type': 'ambient',
                     'current_sensor_type': 'none',
                     'name': 'Maxon RE-Max 2.5W 24V 203950',
                     'nominal_voltage':24.0, #V
                     'no_load_speed':11600.0, #rpm
                     'no_load_current': 3.55, #mA
                     'nominal_speed': 8080.0, #rpm
                     'nominal_torque': 2.66, #mNm
                     'nominal_current': 0.138, #A
                     'stall_torque': 8.85, #mNm
                     'starting_current':0.45, #A
                     'max_efficiency': 83.0, #%
                     'winding_resistance': 53.3,#Ohm
                     'winding_inductance': 1.31,#mH
                     'torque_constant': 19.6,#mNm/A
                     'speed_constant':486.0,#rpm/V
                     'speed_torque_gradient':1320.0,#rpm/mNm
                     'mechanical_time_constant':7.26,#ms
                     'rotor_inertia': 0.526,#gcm^2
                     'thermal_resistance_housing_ambient': 37.0,#K/W
                     'thermal_resistance_rotor_housing': 10.0,#K/W
                     'thermal_time_constant_winding': 6.93,#s
                     'thermal_time_constant_motor': 444.0,#s
                     'max_winding_temp': 85.0,#C
                     'gear_ratio':67.0,
                     'amplifier_resistance':0.019,#Ohm
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
                     'name': 'ContElec VertX13',
                     'type': 'vertx_14bit'},
           'torque': {'type': 'none'},
           'torquedot_df': {'cutoff_freq': 100,
                            'order': 3,
                            'type': 'diff_butterworth'}},
 'description': 'm3_hex4_v1.0',
 'ec_component': 'm3actuator_ec_xxx_jx',
 'ignore_bounds': 0,
 'safe_pwm_limit': 1,
 'joint_component': 'm3joint_xxx_jx',
 'version':'iss',
 'name': 'm3actuator_xxx_jx',
 'param': {'max_overload_time': 3.0,
           'max_amp_temp': 100.0,
           'max_amp_current': 8800.0,
           'max_tq': 0.0,
           'min_tq': 0.0}}


# ########################## JOINT S2.R1.J0-J11 ########################################

config_head_s2r2_joint_j0={
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
           'max_q_slew_rate': 25.0,
           'min_q': 0.0,
           'min_q_pad': 0.0},
 'transmission':{'act_name': 'm3actuator_xxx_j0',
                 'qj_to_qa':[1.0],
                 'qs_to_qj':[1.0], #gearing on encoder'qs_to_qj':[45.0/60.0], #gearing on encoder
                 'tqj_to_tqa':[1.0],
                 'tqs_to_tqj':[1.0],
                 'type': 'gear'}}

config_head_s2r2_joint_j1=copy.deepcopy(config_head_s2r2_joint_j0)
config_head_s2r2_joint_j2=copy.deepcopy(config_head_s2r2_joint_j0)
config_head_s2r2_joint_j3=copy.deepcopy(config_head_s2r2_joint_j0)
config_head_s2r2_joint_j4=copy.deepcopy(config_head_s2r2_joint_j0)
config_head_s2r2_joint_j5=copy.deepcopy(config_head_s2r2_joint_j0)
config_head_s2r2_joint_j6=copy.deepcopy(config_head_s2r2_joint_j0)
config_head_s2r2_joint_j7_ens_eyelids=copy.deepcopy(config_head_s2r2_joint_j0)


# ########################## HEAD S2.R1 ########################################
#Note: need to set payload for each type of head at head frame
#But do kinematics up to eye-tilt frame
# and also not do full gravity calcs, just get effective com and inertia
config_head_s2r2={
 'name': 'm3head_xxx',
 'ndof': 7,
 'limb_name': 'head',
 'dynamatics_component': 'm3dynamatics_s2r2_xxx',
 'version':'iss',
 'eyes':{
  'right':{
   'rotation': [1.0,
                0.0,
                0.0,
                0.0,
                0.0,
                1.0,
                0.0,
                -1.0,
                0.0],
   'translation':[0.0, 0.0, .053],
   },
  'left':{
   'rotation': [1.0,
                0.0,
                0.0,
                0.0,
                0.0,
                1.0,
                0.0,
                -1.0,
                0.0],
   'translation':[0.0, 0.0, -.053],
   }},
 'joint_components':{'J0': 'm3joint_xxx_j0',
                     'J1': 'm3joint_xxx_j1',
                     'J2': 'm3joint_xxx_j2',
                     'J3': 'm3joint_xxx_j3',
                     'J4': 'm3joint_xxx_j4',
                     'J5': 'm3joint_xxx_j5',
                     'J6': 'm3joint_xxx_j6',
                     'J7': 'm3joint_xxx_j7'}}

config_head_s2r2_ens=copy.deepcopy(config_head_s2r2)
config_head_s2r2_ens['dynamatics_component']='m3dynamatics_s2r2_ens_xxx'


 # ########################## Dynamatics Head S2.R1  #######
s2r2_dynamatics_ens={
 'chain_component': 'm3head_xxx',
 'links': [
  #link 1
  {'Ixx': 0.00211990,  #
   'Ixy': 0.00000132,#
   'Ixz': 0.00000000,#
   'Iyy': 0.00040181,#
   'Iyz': 0.00000298,#
   'Izz': 0.00206147,#
   'a': 0.0,#
   'alpha': 90.0,#
   'cx': 0.00006103,#
   'cy': 0.02879863,#
   'cz': -0.00331851,#
   'd': 0.0,#
   'joint_offset': 0.0,#
   'm': .940},#
  #link 2
  {'Ixx': 0.00019239,  #
   'Ixy': 0.00000038,#
   'Ixz': -0.00000083,#
   'Iyy': 0.00017357,#
   'Iyz': 0.00000030,#
   'Izz': 0.00016513,#
   'a': 0.0,#
   'alpha': -90.0,#
   'cx': 0.00015121,#
   'cy': -0.00181278,#
   'cz': -0.00211466,#
   'd': .13849,#
   'joint_offset': -90.0,#
   'm': .409},#

  #link 3
  {'Ixx': 0.00276741,  #
   'Ixy': 0.00000128,#
   'Ixz': 0.00000005,#
   'Iyy': 0.00036949,#
   'Iyz': 0.00000985,#
   'Izz': 0.00258092,#
   'a': 0.0,#
   'alpha': 90.0,#
   'cx':0.00004540,
   'cy': 0.05398226,
   'cz': 0.01069054,
   'd': 0.0,#
   'joint_offset': 90.0,#
   'm': .496},
  #link 4: main head mass
  {'Ixx':  0.03386569, #
   'Ixy': 0.00358819,#
   'Ixz': -0.00002813,#
   'Iyy': 0.03066558,#
   'Iyz': 0.00012964,#
   'Izz': 0.0366365,#
   'a': 0.0,##
   'alpha': 90.0,##
   'cx': 0.03494425,#
   'cy': 0.05866859,#
   'cz': -0.00056344,#
   'd': 0.0,#
   'joint_offset': -90.0,##
   'm': 3.178},#
  #link 5 : eye tilt mass (lumped in with main head, ignore)
  {'Ixx': 1.0,  #
   'Ixy': 0.0,#
   'Ixz': 0.0,#
   'Iyy': 1.0,#
   'Iyz': 0.0,#
   'Izz': 1.0,#
   'a': .12508,#
   'alpha': 00.0,#
   'cx': 0.0,#
   'cy': 0.0,#
   'cz': 0.0,#
   'd': 0.0,#
   'joint_offset': 0.0,#
   'm': 0.0},#
  #link 6
  {'a': 0.0,#
   'alpha': 0.0,#
   'd': 0.0,#
   'joint_offset': 0.0#
   }],
 'name': 'm3dynamatics_s2r2_xxx',
 'ndof': 5,
 'param': {'payload_com': [0.0, 0.0, 0.0], 
           'payload_inertia': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
           'payload_mass': 0.0,
           'use_accelerations': False,
           'use_velocities': False}} 



s2r2_dynamatics_ens['name']='m3dynamatics_s2r2_ens_xxx'

# ########################## S2.R1 ########################################

config_full_s2r2_ens={'actuator_ec':[config_head_s2r2_actuator_ec_j0,
                                     config_head_s2r2_actuator_ec_j1,
                                     config_head_s2r2_actuator_ec_j2,
                                     config_head_s2r2_actuator_ec_j3,                                
                                     config_head_s2r2_actuator_ec_j4,
                                     config_head_s2r2_actuator_ec_j5,
                                     config_head_s2r2_actuator_ec_j6,
                                     config_head_s2r2_actuator_ec_j7_ens_eyelids],
                      'actuator':[config_head_s2r2_actuator_j0,
                                  config_head_s2r2_actuator_j1,
                                  config_head_s2r2_actuator_j2,
                                  config_head_s2r2_actuator_j3,                              
                                  config_head_s2r2_actuator_j4,
                                  config_head_s2r2_actuator_j5,
                                  config_head_s2r2_actuator_j6,
                                  config_head_s2r2_actuator_j7_ens_eyelids],
                      'joint':[config_head_s2r2_joint_j0,
                               config_head_s2r2_joint_j1,
                               config_head_s2r2_joint_j2,
                               config_head_s2r2_joint_j3,
                               config_head_s2r2_joint_j4,
                               config_head_s2r2_joint_j5,
                               config_head_s2r2_joint_j6,
                               config_head_s2r2_joint_j7_ens_eyelids],
                      'dynamatics':s2r2_dynamatics_ens,
                      'head':config_head_s2r2_ens}

