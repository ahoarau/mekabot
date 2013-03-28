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



# ##############################################################
config_robot_humanoid_t2r2={'chains': 
			    {'torso': 
			     {'base_rotation_in_parent': [1.0,
							  0.0,
							  0.0,
							  0.0,
							  1.0,
							  0.0,
							  0.0,
							  0.0,
							  1.0],
			      'base_translation_in_parent': [0.0, 0.0, 0.0],
			      'chain_component': 'm3torso_xxx'}},
			    'name': 'm3humanoid_xxx',
			    'pwr_component': 'm3pwr_pwrxxx'}
config_robot_humanoid_t2r3=copy.deepcopy(config_robot_humanoid_t2r2)
# ##############################################################
config_robot_humanoid_a2r2_left={'chains': 
				      {'left_arm': {'arm_model': 'a2r2_left',
						    'base_rotation_in_parent': [1.0,
										0.0,
										0.0,
										0.0,
										1.0,
										0.0,
										0.0,
										0.0,
										1.0],
						    'base_translation_in_parent': [0.0, 0.0, 0.0],
						    'chain_component': 'm3arm_xxx',
						    'tool_rotation_in_end': [-1.0,
									     0.0,
									     0.0,
									     0.0,
									     -1.0,
									     0.0,
									     0.0,
									     0.0,
									     1.0],
						    'tool_translation_in_end': [0.0, 0.0, 0.0]}},
			    'name': 'm3humanoid_xxx',
			    'pwr_component': 'm3pwr_pwrxxx'}
config_robot_humanoid_a2r3_left=copy.deepcopy(config_robot_humanoid_a2r2_left)
# ##############################################################
config_robot_humanoid_a2r2_right={'chains': 
				  {'right_arm': 
				   {'arm_model': 'a2r2_right',
				    'base_rotation_in_parent': [1.0,
								0.0,
								0.0,
								0.0,
								1.0,
								0.0,
								0.0,
								0.0,
								1.0],
				    'base_translation_in_parent': [0.0, 0.0, 0.0],
				    'chain_component': 'm3arm_xxx',
				    'tool_rotation_in_end': [-1.0,
							     0.0,
							     0.0,
							     0.0,
							     -1.0,
							     0.0,
							     0.0,
							     0.0,
							     1.0],
				    'tool_translation_in_end': [0.0, 0.0, 0.0]}},
	   'name': 'm3humanoid_xxx',
	   'pwr_component': 'm3pwr_pwrxxx'}
config_robot_humanoid_a2r3_right=copy.deepcopy(config_robot_humanoid_a2r2_right)
# ##############################################################
config_robot_humanoid_a2r2_bimanual_t2r2={'chains': 
					  {'left_arm': 
					   {'arm_model': 'a2r2_left',
					    'base_rotation_in_parent': [0.0,
									0.0,
									1.0,
									1.0,
									0.0,
									0.0,
									0.0,
									1.0,
									0.0],
					    'base_translation_in_parent': [0.0, 0.0, 0.0],
					    'chain_component': 'm3arm_xxx'},
					   'right_arm': 
					   {'arm_model': 'a2r2_right',
					    'base_rotation_in_parent': [0.0,
									0.0,
									1.0,
									1.0,
									0.0,
									0.0,
									0.0,
									1.0,
									0.0],
					    'base_translation_in_parent': [0.0, 0.0, 0.0],
					    'chain_component': 'm3arm_xxx'},
					   'torso': 
					   {'base_rotation_in_parent': [1.0,
									0.0,
									0.0,
									0.0,
									1.0,
									0.0,
									0.0,
									0.0,
									1.0],
					    'base_translation_in_parent': [0.0, 0.0, 0.0],
					    'chain_component': 'm3torso_xxx'}},
					  'name': 'm3humanoid_xxx',
					  'pwr_component': 'm3pwr_pwrxxx'}
config_robot_humanoid_a2r3_bimanual_t2r3=copy.deepcopy(config_robot_humanoid_a2r2_bimanual_t2r2)
# #############################################################
config_robot_humanoid_a2r3_bimanual_t2r3_s2r1={'chains': 
					  {'left_arm': 
					   {'arm_model': 'a2r2_left',
					    'base_rotation_in_parent': [0.0,
									0.0,
									1.0,
									1.0,
									0.0,
									0.0,
									0.0,
									1.0,
									0.0],
					    'base_translation_in_parent': [0.0, 0.0, 0.0],
					    'chain_component': 'm3arm_xxx'},
					   'right_arm': 
					   {'arm_model': 'a2r2_right',
					    'base_rotation_in_parent': [0.0,
									0.0,
									1.0,
									1.0,
									0.0,
									0.0,
									0.0,
									1.0,
									0.0],
					    'base_translation_in_parent': [0.0, 0.0, 0.0],
					    'chain_component': 'm3arm_xxx'},
					   'torso': 
					   {'base_rotation_in_parent': [1.0,
									0.0,
									0.0,
									0.0,
									1.0,
									0.0,
									0.0,
									0.0,
									1.0],
					    'base_translation_in_parent': [0.0, 0.0, 0.0],
					    'chain_component': 'm3torso_xxx'},
                                          'head': 
					   {'base_rotation_in_parent': [1.0,
									0.0,
									0.0,
									0.0,
									1.0,
									0.0,
									0.0,
									0.0,
									1.0],
					    'base_translation_in_parent': [0.0, 0.0, 0.07785],
					    'chain_component': 'm3head_xxx'}},
					  'name': 'm3humanoid_xxx',
					  'pwr_component': 'm3pwr_pwrxxx'}
config_robot_humanoid_a2r2_bimanual_t2r2_s2r1=copy.deepcopy(config_robot_humanoid_a2r3_bimanual_t2r3_s2r1)
# ##############################################################
config_robot_humanoid_a2r3_right_t2r3_s2r1={'chains': 
					  {'right_arm': 
					   {'arm_model': 'a2r2_right',
					    'base_rotation_in_parent': [0.0,
									0.0,
									1.0,
									1.0,
									0.0,
									0.0,
									0.0,
									1.0,
									0.0],
					    'base_translation_in_parent': [0.0, 0.0, 0.0],
					    'chain_component': 'm3arm_xxx'},
					   'torso': 
					   {'base_rotation_in_parent': [1.0,
									0.0,
									0.0,
									0.0,
									1.0,
									0.0,
									0.0,
									0.0,
									1.0],
					    'base_translation_in_parent': [0.0, 0.0, 0.0],
					    'chain_component': 'm3torso_xxx'},
                                          'head': 
					   {'base_rotation_in_parent': [1.0,
									0.0,
									0.0,
									0.0,
									1.0,
									0.0,
									0.0,
									0.0,
									1.0],
					    'base_translation_in_parent': [0.0, 0.0, 0.07785],
					    'chain_component': 'm3head_xxx'}},
					  'name': 'm3humanoid_xxx',
					  'pwr_component': 'm3pwr_pwrxxx'}

# ##############################################################

config_robot_humanoid_s2r1={'chains': {'head': 
				       {'base_rotation_in_parent': [1.0,
								    0.0,
								    0.0,
								    0.0,
								    1.0,
								    0.0,
								    0.0,
								    0.0,
								    1.0],
					'base_translation_in_parent': [0.0, 0.0, 0.07785],
					'chain_component': 'm3head_xxx'}},
				      'name': 'm3humanoid_xxx',
				      'pwr_component': 'm3pwr_pwrxxx'}

# ##############################################################

config_robot_humanoid_a2r2_right_t2r2={'chains': 
					  {'right_arm': 
					   {'arm_model': 'a2r2_right',
					    'base_rotation_in_parent': [0.0,
									0.0,
									1.0,
									1.0,
									0.0,
									0.0,
									0.0,
									1.0,
									0.0],
					    'base_translation_in_parent': [0.0, 0.0, 0.0],
					    'chain_component': 'm3arm_xxx'},
					   'torso': 
					   {'base_rotation_in_parent': [1.0,
									0.0,
									0.0,
									0.0,
									1.0,
									0.0,
									0.0,
									0.0,
									1.0],
					    'base_translation_in_parent': [0.0, 0.0, 0.0],
					    'chain_component': 'm3torso_xxx'}},
					  'name': 'm3humanoid_xxx',
					  'pwr_component': 'm3pwr_pwrxxx'}
config_robot_humanoid_a2r3_right_t2r3=copy.deepcopy(config_robot_humanoid_a2r2_right_t2r2)
# ##############################################################
config_robot_humanoid_a2r2_left_t2r2={'chains': 
					  {'left_arm': 
					   {'arm_model': 'a2r2_left',
					    'base_rotation_in_parent': [0.0,
									0.0,
									1.0,
									1.0,
									0.0,
									0.0,
									0.0,
									1.0,
									0.0],
					    'base_translation_in_parent': [0.0, 0.0, 0.0],
					    'chain_component': 'm3arm_xxx'},
					   'torso': 
					   {'base_rotation_in_parent': [1.0,
									0.0,
									0.0,
									0.0,
									1.0,
									0.0,
									0.0,
									0.0,
									1.0],
					    'base_translation_in_parent': [0.0, 0.0, 0.0],
					    'chain_component': 'm3torso_xxx'}},
					  'name': 'm3humanoid_xxx',
					  'pwr_component': 'm3pwr_pwrxxx'}
config_robot_humanoid_a2r3_left_t2r3=copy.deepcopy(config_robot_humanoid_a2r2_left_t2r2)
# ##############################################################
config_robot_humanoid_a2r2_bimanual={'chains': 
					  {'left_arm': {'arm_model': 'a2r2_left',
							'base_rotation_in_parent': [1.0,
										    0.0,
										    0.0,
										    0.0,
										    1.0,
										    0.0,
										    0.0,
										    0.0,
										    1.0],
							'base_translation_in_parent': [0.0, 0.0, 0.0],
							'chain_component': 'm3arm_xxx',
							'tool_rotation_in_end': [-1.0,
										 0.0,
										 0.0,
										 0.0,
										 -1.0,
										 0.0,
										 0.0,
										 0.0,
										 1.0],
							'tool_translation_in_end': [0.0, 0.0, 0.0]},
					   'right_arm': {'arm_model': 'a2r2_right',
							 'base_rotation_in_parent': [1.0,
										     0.0,
										     0.0,
										     0.0,
										     1.0,
										     0.0,
										     0.0,
										     0.0,
										     1.0],
							 'base_translation_in_parent': [0.0, 0.0, 0.0],
							 'chain_component': 'm3arm_xxx',
							 'tool_rotation_in_end': [-1.0,
										  0.0,
										  0.0,
										  0.0,
										  -1.0,
										  0.0,
										  0.0,
										  0.0,
										  1.0],
							 'tool_translation_in_end': [0.0, 0.0, 0.0]}},
				'name': 'm3humanoid_xxx',
				'pwr_component': 'm3pwr_pwrxxx'}
config_robot_humanoid_a2r3_bimanual=copy.deepcopy(config_robot_humanoid_a2r2_bimanual)

# ############################################################################################
 
config_robot_A2T2={'Humanoid.A2R2-Bimanual.T2R2.S2R1':config_robot_humanoid_a2r2_bimanual_t2r2_s2r1,
                   'Humanoid.A2R2-Bimanual.T2R2':config_robot_humanoid_a2r2_bimanual_t2r2,
		   'Humanoid.A2R2-Bimanual':config_robot_humanoid_a2r2_bimanual,
		   'Humanoid.A2R2-Right.T2R2':config_robot_humanoid_a2r2_right_t2r2,
		   'Humanoid.A2R2-Left.T2R2' :config_robot_humanoid_a2r2_left_t2r2,
		   'Humanoid.A2R2-Right':config_robot_humanoid_a2r2_right,
		   'Humanoid.A2R2-Left':config_robot_humanoid_a2r2_left,
		   'Humanoid.T2R2':config_robot_humanoid_t2r2,
		   }

config_robot_A3T3={'Humanoid.A2R3-Bimanual.T2R3':config_robot_humanoid_a2r3_bimanual_t2r3,
		   'Humanoid.A2R3-Bimanual':config_robot_humanoid_a2r3_bimanual,
		   'Humanoid.A2R3-Right.T2R3':config_robot_humanoid_a2r3_right_t2r3,
		   'Humanoid.A2R3-Left.T2R3' :config_robot_humanoid_a2r3_left_t2r3,
		   'Humanoid.A2R3-Right':config_robot_humanoid_a2r3_right,
		   'Humanoid.A2R3-Left':config_robot_humanoid_a2r3_left,
		   'Humanoid.T2R3':config_robot_humanoid_t2r3,
		   'Humanoid.S2R1':config_robot_humanoid_s2r1,
                   'Humanoid.A2R3-Bimanual.T2R3.S2R1':config_robot_humanoid_a2r3_bimanual_t2r3_s2r1,
		   'Humanoid.A2R3-Right.T2R3.S2R1':config_robot_humanoid_a2r3_right_t2r3_s2r1
		   }
# ############################################################################################
# Generate configs for latest generation
def generate_humanoid():
    robot_name=get_robot_name()
    has_torso=False
    has_right_arm=False
    has_left_arm=False
    has_head=False
    print 'Torso present [y]?'
    if m3t.get_yes_no('y'):
	has_torso=True
	torso_name=get_torso_name()
	torso_version=get_version(['T2.R2','T2.R3'])
    print 'Right arm present [y]?'
    if m3t.get_yes_no('y'):
	has_right_arm=True
	print 'For right arm: ',
	right_arm_name=get_arm_name()
	right_arm_version=get_version(['A2.R2','A2.R3'])
    print 'Left arm present [y]?'
    if m3t.get_yes_no('y'):
	has_left_arm=True
	print 'For left arm: ',
	left_arm_name=get_arm_name()
	left_arm_version=get_version(['A2.R2','A2.R3'])
    print 'Head present [y]?'
    if m3t.get_yes_no('y'):
	has_head=True
	print 'For head: ',
	head_name=get_head_name()
	head_version='S2.R1'
    pwr_name=get_pwr_name()
	
    config_dir=m3t.get_m3_config_path()+robot_name+'/'
    data=[]
    #descending number of limbs
    #4
    if has_left_arm and has_right_arm and has_torso and has_head:
	if torso_version=='T2.R3' and right_arm_version=='A2.R3' and left_arm_version=='A2.R3' and head_version=='S2.R1':
	    x=copy.deepcopy(config_robot_A3T3['Humanoid.A2R3-Bimanual.T2R3.S2R1'])
	    x['name']='m3humanoid_bimanual_torso_head_'+robot_name
	    x['pwr_component']=pwr_name
	    x['chains']['right_arm']['chain_component']='m3arm_'+right_arm_name
	    x['chains']['left_arm']['chain_component']='m3arm_'+left_arm_name
	    x['chains']['torso']['chain_component']='m3torso_'+torso_name
	    x['chains']['head']['chain_component']='m3head_'+head_name
	    data.append(x)
	    return config_dir,data
	if torso_version=='T2.R2' and right_arm_version=='A2.R2' and left_arm_version=='A2.R2' and head_version=='S2.R1':
	    x=copy.deepcopy(config_robot_A2T2['Humanoid.A2R2-Bimanual.T2R2.S2R1'])
	    x['name']='m3humanoid_bimanual_torso_head_'+robot_name
	    x['pwr_component']=pwr_name
	    x['chains']['right_arm']['chain_component']='m3arm_'+right_arm_name
	    x['chains']['left_arm']['chain_component']='m3arm_'+left_arm_name
	    x['chains']['torso']['chain_component']='m3torso_'+torso_name
	    x['chains']['head']['chain_component']='m3head_'+head_name
	    data.append(x)
	    return config_dir,data
    #3
    if has_right_arm and has_torso and has_head:
	if torso_version=='T2.R3' and right_arm_version=='A2.R3' and head_version=='S2.R1':
	    x=copy.deepcopy(config_robot_A3T3['Humanoid.A2R3-Right.T2R3.S2R1'])
	    x['name']='m3humanoid_right_arm_torso_head_'+robot_name
	    x['pwr_component']=pwr_name
	    x['chains']['right_arm']['chain_component']='m3arm_'+right_arm_name
	    x['chains']['torso']['chain_component']='m3torso_'+torso_name
	    x['chains']['head']['chain_component']='m3head_'+head_name
	    data.append(x)
	    return config_dir,data
	if torso_version=='T2.R2' and right_arm_version=='A2.R2' and head_version=='S2.R1':
	    x=copy.deepcopy(config_robot_A2T2['Humanoid.A2R2-Right.T2R2.S2R1'])
	    x['name']='m3humanoid_right_arm_torso_head_'+robot_name
	    x['pwr_component']=pwr_name
	    x['chains']['right_arm']['chain_component']='m3arm_'+right_arm_name
	    x['chains']['torso']['chain_component']='m3torso_'+torso_name
	    x['chains']['head']['chain_component']='m3head_'+head_name
	    data.append(x)
	    return config_dir,data
    if has_left_arm and has_right_arm and has_torso:
	x=copy.deepcopy(config_robot_A3T3['Humanoid.A2R3-Bimanual.T2R3'])
	x['name']='m3humanoid_bimanual_torso_'+robot_name
	x['pwr_component']=pwr_name
	x['chains']['right_arm']['chain_component']='m3arm_'+right_arm_name
	x['chains']['left_arm']['chain_component']='m3arm_'+left_arm_name
	x['chains']['torso']['chain_component']='m3torso_'+torso_name
	data.append(x)
	return config_dir,data
    
    #2
    if has_left_arm and has_torso:
	x=copy.deepcopy(config_robot_A3T3['Humanoid.A2R3-Left.T2R3'])
	x['name']='m3humanoid_left_arm_torso_'+robot_name
	x['pwr_component']=pwr_name
	x['chains']['torso']['chain_component']='m3torso_'+torso_name
	x['chains']['left_arm']['chain_component']='m3arm_'+left_arm_name
	data.append(x)
	return config_dir,data
    if has_right_arm and has_torso:
	x=copy.deepcopy(config_robot_A3T3['Humanoid.A2R3-Right.T2R3'])
	x['name']='m3humanoid_right_arm_torso_'+robot_name
	x['pwr_component']=pwr_name
	x['chains']['torso']['chain_component']='m3torso_'+torso_name
	x['chains']['right_arm']['chain_component']='m3arm_'+right_arm_name
	data.append(x)
	return config_dir,data
    if has_left_arm and has_right_arm:
	x=copy.deepcopy(config_robot_A3T3['Humanoid.A2R3-Bimanual'])
	x['name']='m3humanoid_bimanual_'+robot_name
	x['pwr_component']=pwr_name
	x['chains']['right_arm']['chain_component']='m3arm_'+right_arm_name
	x['chains']['left_arm']['chain_component']='m3arm_'+left_arm_name
	data.append(x)
	return config_dir,data
    
    #1
    if has_torso:
	x=copy.deepcopy(config_robot_A3T3['Humanoid.T2R3'])
	x['name']='m3humanoid_torso_'+robot_name
	x['pwr_component']=pwr_name
	x['chains']['torso']['chain_component']='m3torso_'+torso_name
	data.append(x)
	return config_dir,data
    if has_right_arm:
	x=copy.deepcopy(config_robot_A3T3['Humanoid.A2R3-Right'])
	x['name']='m3humanoid_right_arm_'+robot_name
	x['pwr_component']=pwr_name
	x['chains']['right_arm']['chain_component']='m3arm_'+right_arm_name
	data.append(x)
	return config_dir,data
    if has_left_arm:
	x=copy.deepcopy(config_robot_A3T3['Humanoid.A2R3-Left'])
	x['name']='m3humanoid_left_arm_'+robot_name
	x['pwr_component']=pwr_name
	x['chains']['left_arm']['chain_component']='m3arm_'+left_arm_name
	data.append(x)
	return config_dir,data
    if has_head:
	x=copy.deepcopy(config_robot_A3T3['Humanoid.S2R1'])
	x['name']='m3humanoid_head_'+robot_name
	x['pwr_component']=pwr_name
	x['chains']['head']['chain_component']='m3head_'+head_name
	data.append(x)
	return config_dir,data


    
    






