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

from scipy import polyval
import time
import numpy.numarray as na
import Numeric as nu
import math
import os 
import sys

M3EC_ADC_TICKS_MAX = 4096.0

# ####################################################################################################

#TC1047 
#10mV/C, 750mV@25C. 3.3V supply, so 4096*750/3300 around 930 ticks for room temp
#10mV/C, 750mV@25C. 5.0V supply, so 4096*750/5000 around 614 ticks for room temp
#4096*10/3300 ticks per C, gives 12.412ticks/C, or scale of .0805C/tick for 3.3V supply
#4096*(10*3300/5000)/3300 ticks per C, gives  8.192ticks/C, or scale of .12207C/tick for 5V supply
# 750mV at 25C spec, so 0mv at -50C, and bias of -50.0 

#LM60
#6.25mV/C, 580mV@25C. 3.3V supply, so 4096*580/3300 around 719 ticks for room temp
#6.25mV/C, 580mV@25C. 5.0V supply, so 4096*580/5000 around 475 ticks for room temp
#4096*6.25/3300 ticks per C, gives 7.75ticks/C, or scale of .1289 for 3.3V supply
#4096*(6.25*3300/5000)/3300 ticks per C, gives 5.12ticks/C, or scale of .1953 for 5V supply
# 580mV at 25C spec, so 0mv at -50C, and bias of -67.8

#TMP36
#10mV/C, 750mV@25C. 3.3V supply, so 4096*750/3300 around 930 ticks for room temp
#10mV/C, 750mV@25C. 5.0V supply, so 4096*750/5000 around 614 ticks for room temp
#4096*10/3300 ticks per C, gives 12.412ticks/C, or scale of .0805C/tick for 3.3V supply
#4096*(10*3300/5000)/3300 ticks per C, gives  8.192ticks/C, or scale of .12207C/tick for 5V supply
# 750mV at 25C spec, so 0mv at -50C, and bias of -50.0 

class M3TempSensor:
	def __init__(self,ctype):
		self.ctype=ctype
		print "type:", self.ctype
	
	def raw_2_C(self,calib,ticks):
		print "type1:", self.ctype
		#ticks: adc sensor value
		#calib: yaml config calibration map
		if self.ctype=='adc_linear_3V3':
			mV=ticks/(M3EC_ADC_TICKS_MAX/3300.0)
			bias = 25.0-(calib['cb_mV_at_25C']/calib['cb_mV_per_C'])
			val = mV/calib['cb_mV_per_C'] + bias
		if self.ctype=='adc_linear_5V':
			mV=ticks/(M3EC_ADC_TICKS_MAX/5000.0)
			bias = 25.0-(calib['cb_mV_at_25C']/calib['cb_mV_per_C'])
			val = mV/calib['cb_mV_per_C'] + bias
		if self.ctype=='adc_linear_5V_ns':
			mV=ticks/(M3EC_ADC_TICKS_MAX/3300.0)
			bias = 25.0-(calib['cb_mV_at_25C']/calib['cb_mV_per_C'])
			val = mV/calib['cb_mV_per_C'] + bias
		if self.ctype=='adc_poly':
			val = float(polyval(calib['cb_temp'],ticks))
		if self.ctype=='temp_25C':
			return 25.0
		if self.ctype=='none':
			return 0.0
		if self.ctype == 'dsp_calib':
			val = ticks
			print "type2:", self.ctype
		return val*calib['cb_scale']+calib['cb_bias']
		
# ####################################################################################################

class M3CurrentSensor:
	def __init__(self,ctype):
		self.ctype=ctype
	def raw_2_mA(self,calib,ticks_a,ticks_b):
		#ticks: adc sensor value
		#calib: yaml config calibration map
		if self.ctype=='adc_linear_5V':
			mV_a = 5000.0*(ticks_a-calib['cb_ticks_at_zero_a'])/M3EC_ADC_TICKS_MAX
			mV_b = 5000.0*(ticks_b-calib['cb_ticks_at_zero_b'])/M3EC_ADC_TICKS_MAX
			i_a = 1000.0*mV_a/calib['cb_mV_per_A']
			i_b = 1000.0*mV_b/calib['cb_mV_per_A']
			val=max(abs(i_a), abs(i_b))
			return max(0,val*calib['cb_scale']+calib['cb_bias'])	
		if self.ctype=='adc_poly':
			i_a= float(polyval(calib['cb_current_a'],ticks_a))
			i_b= float(polyval(calib['cb_current_b'],ticks_b))
			val=max(abs(i_a), abs(i_b))
			return val*calib['cb_scale']+calib['cb_bias']
		return 0.0
	
# ####################################################################################################

class M3VoltageSensor:
	def __init__(self,ctype):
		self.ctype=ctype
	def raw_2_V(self,calib,ticks):
		#ticks: adc sensor value
		#calib: yaml config calibration map
		if self.ctype=='adc_poly':
			v= float(polyval(calib['cb_voltage'],ticks))
			return v*calib['cb_scale']+calib['cb_bias']
# ####################################################################################################

MA3_10BIT_MAX_PULSE_US =1024.0
MA3_10BIT_PERIOD_US  =MA3_10BIT_MAX_PULSE_US+1
MA3_12BIT_MAX_PULSE_US =4096.0
MA3_12BIT_PERIOD_US  =MA3_12BIT_MAX_PULSE_US+1
VERTX_14BIT_MAX  =16383.0

class M3AngleSensor:
	def __init__(self,ctype):
		self.ctype=ctype
	def raw_2_deg(self,calib,qei_on,qei_period,qei_rollover, qs_to_qj = 1.0):
		#qei: raw sensor value
		#calib: yaml config calibration map
			
		if self.ctype=="vertx_14bit":
			val = qs_to_qj*360.0*(qei_on)/VERTX_14BIT_MAX
			return val*calib['cb_scale']+calib['cb_bias']
		
		if self.ctype=="ma3_12bit":
			if qei_period==0.0:
				return 0.0
			val = ((qei_on*MA3_12BIT_PERIOD_US)/qei_period)-1 
			val= qs_to_qj*360.0*(val/MA3_12BIT_MAX_PULSE_US)
			return val*calib['cb_scale']+calib['cb_bias']
		
		if self.ctype=="ma3_10bit":
			if qei_period==0.0:
				return 0.0
			val = ((qei_on*MA3_10BIT_PERIOD_US)/qei_period)-1
			val= qs_to_qj*360.0*(val/MA3_10BIT_MAX_PULSE_US)
			return val*calib['cb_scale']+calib['cb_bias']
		
		if self.ctype=="ma3_12bit_poly":
			if qei_period==0.0:
				return 0.0
			val = qs_to_qj*(((qei_on*MA3_12BIT_PERIOD_US)/qei_period)-1)
			val=float(polyval(calib['cb_theta'],val))
			return val*calib['cb_scale']+calib['cb_bias']

# ####################################################################################################

class M3TorqueSensor:
	def __init__(self,ctype):
		self.ctype=ctype
		
	def raw_2_mNm(self,calib,ticks):
		#ticks: raw sensor value
		#calib: yaml config calibration map
		if self.ctype=='sea_vertx_14bit' or self.ctype=='adc_poly':
			val=float(polyval(calib['cb_torque'],ticks))
			return val*float(calib['cb_scale'])+float(calib['cb_bias'])
		return 0.0
	
	def mNm_2_raw(self,calib,tq):
		#ticks: calibrated sensor value
		#calib: yaml config calibration map
		if self.ctype=='sea_vertx_14bit':
			val=(tq-calib['cb_bias'])/calib['cb_scale']
			val=float(polyval(calib['cb_inv_torque'],val))
			return max(0,min(val,VERTX_14BIT_MAX))
		if self.ctype=='adc_poly':
			val=(tq-calib['cb_bias'])/calib['cb_scale']
			val=float(polyval(calib['cb_inv_torque'],val))
			return max(0,min(val,M3EC_ADC_TICKS_MAX))

# ####################################################################################################

class M3WrenchSensor:
	def __init__(self,ctype):
		self.ctype=ctype
		
	def raw_2_mNm(self,calib,ticks_0,ticks_1,ticks_2,ticks_3,ticks_4,ticks_5):
		#ticks: raw sensor value
		#calib: yaml config calibration map
		if self.ctype=='6x6_linear':
			C=nu.array([calib['cb_fx'],calib['cb_fy'],calib['cb_fz'],calib['cb_tx'],calib['cb_ty'],calib['cb_tz']])
			z=nu.array(calib['cb_adc_bias'])
			ticks=nu.array([ticks_0,ticks_1,ticks_2,ticks_3,ticks_4,ticks_5],nu.Float)
			w= nu.matrixmultiply(C,(ticks-z))
			return w*calib['cb_scale']+nu.array(calib['cb_bias'])
		return [0.0]*6
