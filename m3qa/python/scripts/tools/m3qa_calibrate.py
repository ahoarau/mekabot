#! /usr/bin/python

#Copyright  2008, Meka Robotics
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


from m3qa.calibrate_arm_a1r1 import *
from m3qa.calibrate_arm_a2r1 import *
from m3qa.calibrate_arm_a2r2 import *
from m3qa.calibrate_arm_a2r3 import *
from m3qa.calibrate_arm_a2r4 import *
from m3qa.calibrate_hand_h2r1 import *
from m3qa.calibrate_hand_h2r2 import *
from m3qa.calibrate_hand_h2r3 import *
from m3qa.calibrate_hand_h2r4 import *
from m3qa.calibrate_torso_t1r1 import *
from m3qa.calibrate_torso_t2r1 import *
from m3qa.calibrate_torso_t2r2 import *
from m3qa.calibrate_torso_t2r3 import *
from m3qa.calibrate_load_cell import *
from m3qa.calibrate_pwr import *
from m3qa.calibrate_gripper_g1r1 import *
from m3qa.calibrate_head_s2r1 import *
from m3qa.calibrate_head_s2r2 import *

# ###############################################################################################################
if __name__ == '__main__':
	ctypes = {	'loadx1_ec': {'comp_ec':'m3loadx1_ec','comp_rt':'m3loadx1','inst':M3CalibrateLoadX1Ec()},
				'arm_a1r1':{'comp_ec':'m3actuator_ec','comp_rt':'m3actuator','comp_j':'m3joint','inst':M3Calibrate_Arm_A1R1()},
				'arm_a2r1':{'comp_ec':'m3actuator_ec','comp_rt':'m3actuator','comp_j':'m3joint','inst':M3Calibrate_Arm_A2R1()},
				'arm_a2r2':{'comp_ec':'m3actuator_ec','comp_rt':'m3actuator','comp_j':'m3joint','inst':M3Calibrate_Arm_A2R2()},
				'arm_a2r3':{'comp_ec':'m3actuator_ec','comp_rt':'m3actuator','comp_j':'m3joint','inst':M3Calibrate_Arm_A2R3()},
				'arm_a2r4':{'comp_ec':'m3actuator_ec','comp_rt':'m3actuator','comp_j':'m3joint','inst':M3Calibrate_Arm_A2R4()},
				'hand_h2r1':{'comp_ec':'m3actuator_ec','comp_rt':'m3actuator','comp_j':'m3joint','inst':M3Calibrate_Hand_H2R1()},
				'hand_h2r2':{'comp_ec':'m3actuator_ec','comp_rt':'m3actuator','comp_j':'m3joint','inst':M3Calibrate_Hand_H2R2()},
				'hand_h2r3':{'comp_ec':'m3actuator_ec','comp_rt':'m3actuator','comp_j':'m3joint','inst':M3Calibrate_Hand_H2R3()},
				'hand_h2r4':{'comp_ec':'m3actuator_ec','comp_rt':'m3actuator','comp_j':'m3joint','inst':M3Calibrate_Hand_H2R4()},
				'gripper_g1r1':{'comp_ec':'m3actuator_ec','comp_rt':'m3actuator','comp_j':'m3joint','inst':M3Calibrate_Gripper_G1R1()},
				'torso_t1r1':{'comp_ec':'m3actuator_ec','comp_rt':'m3actuator','comp_j':'m3joint','inst':M3Calibrate_Torso_T1R1()},
				'torso_t2r1':{'comp_ec':'m3actuator_ec','comp_rt':'m3actuator','comp_j':'m3joint','inst':M3Calibrate_Torso_T2R1()},
				'torso_t2r2':{'comp_ec':'m3actuator_ec','comp_rt':'m3actuator','comp_j':'m3joint','inst':M3Calibrate_Torso_T2R2()},
				'torso_t2r3':{'comp_ec':'m3actuator_ec','comp_rt':'m3actuator','comp_j':'m3joint','inst':M3Calibrate_Torso_T2R3()},
				'head_s2r1':{'comp_ec':'m3actuator_ec','comp_rt':'m3actuator','comp_j':'m3joint','inst':M3Calibrate_Head_S2R1()},
				'head_s2r2':{'comp_ec':'m3actuator_ec','comp_rt':'m3actuator','comp_j':'m3joint','inst':M3Calibrate_Head_S2R2()},
				'm3pwr_v0.3':{'comp_ec':'m3pwr_ec','comp_rt':'m3pwr','inst':M3CalibratePwrEc_V0_3()},
				'm3pwr_v0.4':{'comp_ec':'m3pwr_ec','comp_rt':'m3pwr','inst':M3CalibratePwrEc_V0_4()},
				'm3pwr_v0.5':{'comp_ec':'m3pwr_ec','comp_rt':'m3pwr','inst':M3CalibratePwrEc_V0_5()},
				'loadx6_ec': {'comp_ec':'m3loadx6_ec','comp_rt':'m3loadx6','inst':M3CalibrateLoadX6Ec()}
		  }
	
	kk=ctypes.keys()
	print 'Select calibration type'
	for i in range(len(kk)):
		print i,':',kk[i]
	ctype=kk[m3t.get_int()]
	calib=ctypes[ctype]['inst']

	try:
		if calib.start(ctypes[ctype]): 
			calib.get_task()
	except (KeyboardInterrupt,EOFError):
		pass
	except m3t.M3Exception,e:
		print 'M3Exception',e
	finally:
		print 'Shutting down system'
		calib.stop()
		
