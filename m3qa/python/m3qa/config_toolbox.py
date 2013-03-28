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

#copy into param field
def set_payload(dyn,pyl):
	for k in pyl.keys():
		dyn['param'][k]=copy.deepcopy(pyl[k])

#pass in a list of version strings
def get_version(versions):
	if len(versions)==0:
		return versions[0]
	while True:
		print 'Enter version'
		for i in range(len(versions)):
			print i,' : ',versions[i]
		idd=m3t.get_int()
		if idd<len(versions):
			return versions[idd]
		print 'Incorrect value'

#pass in a list of limb strings        
def get_limb_name(limbs):
	if len(limbs)==0:
		return limbs[0]
	while True:
		print 'Enter limb name'
		for i in range(len(limbs)):
			print i,' : ',limbs[i]
		idd=m3t.get_int()
		if idd<len(limbs):
			return limbs[idd]
		print 'Incorrect value'

def get_client_code():
	print 'Enter Client Code (e.g., uta)'
	while True:
		s=m3t.get_string()
		if len(s)!=3:
			print 'Code must be length 3'
		else:
			return s

def get_robot_name():
	while True:
		print 'Enter Robot Number (e.g., 7 for MR7)'
		return 'mr'+str(m3t.get_int())

def get_torso_name():
	while True:
		print 'Enter Torso Number (e.g., 2 for MT2)'
		return 'mt'+str(m3t.get_int())

def get_head_name():
	while True:
		print 'Enter Head Number (e.g., 2 for MS2)'
		return 'ms'+str(m3t.get_int())

def get_arm_name():
	while True:
		print 'Enter Arm Number (e.g., 6 for MA6)'
		return 'ma'+str(m3t.get_int())

def get_base_name():
	while True:
		print 'Enter Base Number (e.g., 1 for MB1)'
		return 'mb'+str(m3t.get_int())
	
def get_zlift_name():
	while True:
		print 'Enter ZLift Number (e.g., 1 for MZ1)'
		return 'mz'+str(m3t.get_int())
	
def get_joint_id():
	print 'Enter Joint ID (e.g., 2 for J2)'
	return m3t.get_int()


def get_pwr_name():
	print 'Enter PWR board ID (e.g., 10 for pwr010)'
	x=m3t.get_int()
	return 'm3pwr_pwr'+'0'*(3-len(str(x)))+str(x)

