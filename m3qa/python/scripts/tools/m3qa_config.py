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

import m3.toolbox as m3t
import os
import yaml

import m3qa.config_pwr as m3p
import m3qa.config_arm as m3a
import m3qa.config_torso as m3tr
import m3qa.config_omnibase as m3bs
import m3qa.config_zlift as m3zl
import m3qa.config_hand as m3h
import m3qa.config_loadx6 as m3l
import m3qa.config_robot as m3r
import m3qa.config_head as m3hd
#import m3qa.config_rviz as m3f
# ###############################################################################################################

def create_config_file(config_dir, config_data):
	try:
		os.mkdir(config_dir)
		print 'Made dir: ',config_dir
	except OSError:
		print 'Dir exists: ', config_dir
	for d in config_data:
		print 'D',d
		print
		fn_d=config_dir+d['name']+'.yml'
		f=file(fn_d,'w')
		print 'Saving...',fn_d
		f.write(yaml.safe_dump(d, default_flow_style=False,width=200))
		f.close()

	
if __name__ == '__main__':
	ctypes = {'M3Pwr':{'generate':m3p.generate},
		  'S2.Head':{'generate':m3hd.generate_head},
		  'S2.Head.Full':{'generate':m3hd.generate_head_full},
		  'S2.Head.Dynamatics':{'generate':m3hd.generate_dynamatics},
		  'S2.Head.Joint':{'generate':m3hd.generate_joint},
		  'A2.Joint':{'generate':m3a.generate_joint},
		  'A2.Arm.Full':{'generate':m3a.generate_arm_full},
		  'A2.Dynamatics':{'generate':m3a.generate_dynamatics},
		  'A2.Arm':{'generate':m3a.generate_arm},
		  'T2.Joint':{'generate':m3tr.generate_joint},
		  'T2.Torso.Full':{'generate':m3tr.generate_torso_full},
		  'T2.Dynamatics':{'generate':m3tr.generate_dynamatics},
		  'T2.Torso':{'generate':m3tr.generate_torso},
		  'B1.Joint':{'generate':m3bs.generate_joint},
		  'B1.OmniBase.Full':{'generate':m3bs.generate_omnibase_full},
		  'B1.OmniBase':{'generate':m3bs.generate_omnibase},
		  'B1.OmniBase.Joint':{'generate':m3bs.generate_joint},
		  'Z1.ZLift.Full':{'generate':m3zl.generate_zlift_full},
		  'H2.Joint':{'generate':m3h.generate_joint},
		  'H2.Hand.Full':{'generate':m3h.generate_hand_full},
		  'H2.Hand':{'generate':m3h.generate_hand},
		  'A2.LoadX6':{'generate':m3l.generate},
		  'M3Humanoid':{'generate':m3r.generate_humanoid}}#,
		  #'M3.RViz':{'generate':m3f.generate_rviz}}
	
	kk=ctypes.keys()
	kk.sort()
	print 'Select configuration type'
	for i in range(len(kk)):
		print i,':',kk[i]
	ctype=kk[m3t.get_int()]
	gen=ctypes[ctype]['generate']
	try:
		config_dir, config_data=apply(gen)
		print '---------------------------------'
		print 'Config Dir',config_dir
		print
		print 'Config Data',config_data
		print
		print '---------------------------------'
		print
		print 'Save generated data [y]?'
		if m3t.get_yes_no('y'):
			create_config_file(config_dir,config_data)
	except (KeyboardInterrupt,EOFError):
		print 'Exception. Exiting now...'

		
    
    
    
    
    