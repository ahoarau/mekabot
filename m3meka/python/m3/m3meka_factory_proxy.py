
import m3.toolbox as m3t
import m3.arm
import m3.torso
import m3.head
import m3.hand
#import m3.hand_ua
import m3.gripper
import m3.actuator_ec
import m3.actuator
import m3.joint
import m3.joint_slave
import m3.joint_zlift
import m3.pwr
import m3.pwr_ec
import m3.joint
import m3.loadx1
import m3.loadx1_ec
import m3.loadx6
import m3.loadx6_ec
import m3.humanoid
import m3.dynamatics
import m3.omnibase
import m3.ledx2xn_ec
import m3.led_matrix_ec
#import m3.head_s2csp_ctrl
import m3.ctrl_simple

######################


import m3.m3_echub_eep
import m3.m3_1100hub_eep 
import  m3.m3_pwr_eep 
import  m3.m3_loadx6_eep  
import  m3.m3_ledx2_eep  
import  m3.m3_bld_eep  
import  m3.m3_actx1_eep  
import  m3.m3_actx2_eep  
import  m3.m3_actx3_eep  
import  m3.m3_actx4_eep  
#import  m3.m3_tactx2_eep  



#############################


component_map['m3arm'] = m3.arm.M3Arm
component_map['m3torso'] = m3.torso.M3Torso
component_map['m3head'] = m3.head.M3Head
component_map['m3hand'] = m3.hand.M3Hand
component_map['m3gripper'] = m3.gripper.M3Gripper
component_map['m3actuator_ec'] = m3.actuator_ec.M3ActuatorEc
component_map['m3actuator'] = m3.actuator.M3Actuator
component_map['m3actuator_virtual'] = m3.actuator.M3Actuator
component_map['m3joint'] = m3.joint.M3Joint
component_map['m3joint_slave'] = m3.joint_slave.M3JointSlave
component_map['m3pwr'] = m3.pwr.M3Pwr
component_map['m3pwr_ec'] = m3.pwr_ec.M3PwrEc
component_map['m3pwr_virtual'] = m3.pwr.M3Pwr
component_map['m3loadx1'] = m3.loadx1.M3LoadX1
component_map['m3loadx1_ec'] = m3.loadx1_ec.M3LoadX1Ec
component_map['m3loadx6'] = m3.loadx6.M3LoadX6
component_map['m3loadx6_ec'] = m3.loadx6_ec.M3LoadX6Ec
component_map['m3humanoid'] = m3.humanoid.M3Humanoid
component_map['m3dynamatics'] = m3.dynamatics.M3Dynamatics
#component_map['m3head_s2csp_ctrl'] = m3.head_s2csp_ctrl.M3HeadS2CSPCtrl
component_map['m3joint_zlift'] = m3.joint_zlift.M3JointZLift
component_map['m3omnibase'] = m3.omnibase.M3OmniBase
component_map['m3ledx2xn_ec'] = m3.ledx2xn_ec.M3LedX2XNEc
component_map['m3led_matrix_ec'] = m3.led_matrix_ec.M3LedMatrixEc
component_map['m3ctrl_simple'] = m3.ctrl_simple.M3CtrlSimple

    #'m3hand_ua': m3.hand_ua.M3HandUA,
    #'m3taxel_array_ec': m3skin.taxel_array_ec.M3TaxelArrayEc,
    #'m3tactile_pps22_ec':m3.tactile_pps22_ec.M3TactilePPS22Ec,

m3meka_actx1 = 1010L
m3meka_actx2 = 1011L
m3meka_actx3 = 1012L
m3meka_actx4 = 1013L
m3meka_ledx2 = 1009L
m3meka_bld = 1007L
m3meka_echub = 1000L
m3meka_echub4 = 1001L
m3meka_pwr = 1002L
m3meka_loadx6 = 1003L

m3_fpfx[m3meka_echub] = 'echub'
m3_fpfx[m3meka_echub4] = 'echub4'
m3_fpfx[m3meka_pwr] = 'pwr'
m3_fpfx[m3meka_loadx6] = 'loadx6'
m3_fpfx[m3meka_ledx2] = 'ledx2'
m3_fpfx[m3meka_bld] = 'bld'
m3_fpfx[m3meka_actx1] = 'actx1'
m3_fpfx[m3meka_actx2] = 'actx2'
m3_fpfx[m3meka_actx3] = 'actx3'
m3_fpfx[m3meka_actx4] = 'actx4'


m3_product_codes['m3_actx1'] = m3meka_actx1
m3_product_codes['m3_actx2'] = m3meka_actx2
m3_product_codes['m3_actx3'] = m3meka_actx3
m3_product_codes['m3_actx4'] = m3meka_actx4
m3_product_codes['m3_ledx2'] = m3meka_ledx2
m3_product_codes['m3_bld'] = m3meka_bld
m3_product_codes['m3_echub'] = m3meka_echub
m3_product_codes['m3_echub4'] = m3meka_echub4
m3_product_codes['m3_pwr'] = m3meka_pwr
m3_product_codes['m3_loadx6'] = m3meka_loadx6
#m3_product_codes['m3_armh'] = 1015L
#m3_product_codes['m3_skinv0'] = 1016L
#m3_product_codes['m3_tactx2'] = 1014L

m3_eeprom_cfgs[m3meka_echub] = m3.m3_echub_eep.slave_cfg_dicts
m3_eeprom_cfgs[m3meka_echub4] = m3.m3_1100hub_eep.slave_cfg_dicts
m3_eeprom_cfgs[m3meka_pwr] = m3.m3_pwr_eep.slave_cfg_dicts
m3_eeprom_cfgs[m3meka_loadx6] = m3.m3_loadx6_eep.slave_cfg_dicts
m3_eeprom_cfgs[m3meka_ledx2] = m3.m3_ledx2_eep.slave_cfg_dicts
m3_eeprom_cfgs[m3meka_bld] = m3.m3_bld_eep.slave_cfg_dicts
m3_eeprom_cfgs[m3meka_actx1] = m3.m3_actx1_eep.slave_cfg_dicts
m3_eeprom_cfgs[m3meka_actx2] = m3.m3_actx2_eep.slave_cfg_dicts
m3_eeprom_cfgs[m3meka_actx3] = m3.m3_actx3_eep.slave_cfg_dicts
m3_eeprom_cfgs[m3meka_actx4] = m3.m3_actx4_eep.slave_cfg_dicts
#m3_eeprom_cfgs[1014L] = m3.m3_tactx2_eep.slave_cfg_dicts


 
