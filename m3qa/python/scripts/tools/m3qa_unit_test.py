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



from m3qa.unit_test_actuator_a2r2 import *


# ###############################################################################################################
if __name__ == '__main__':
	ctypes = {'actuator_a2r2':{'comp_ec':'m3actuator_ec','comp_rt':'m3actuator','comp_j':'m3joint','inst':M3UnitTest_Actuator_A2R2()}}
	
	kk=ctypes.keys()
	print 'Select unit test type'
	for i in range(len(kk)):
		print i,':',kk[i]
	ctype=kk[m3t.get_int()]
	ut=ctypes[ctype]['inst']

	try:
		if ut.start(ctypes[ctype]): 
			ut.get_test()
	except (KeyboardInterrupt,EOFError):
		pass
	except m3t.M3Exception,e:
		print 'M3Exception',e
	finally:
		print 'Shutting down system'
		ut.stop()
		
