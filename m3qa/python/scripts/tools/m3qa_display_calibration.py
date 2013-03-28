#! /usr/bin/python
import pylab as pyl
import m3.toolbox as m3t
import m3.toolbox_ctrl as m3tc
import yaml
import glob

path= m3t.get_m3_config_path()+'data/'
files=glob.glob(path+'calibration_data_raw_m3actuator_ec_*.yml')
files.sort()
if len(files)==0:
	print 'No calibration files found'
	exit()
comps=[]
for f in files:
	comps.append(f[f.find('calibration_data_raw')+21:len(f)-4])
print 'Enter component ID'
print '------------------'
for i in range(len(comps)):
	print i,' : ',comps[i]
idd=m3t.get_int()
if idd<0 or idd>=len(comps):
	print 'Invalid ID'
	exit()

fn=files[idd]
print 'Display calibration file: ',fn
try:
	f=file(fn,'r')
	d= yaml.safe_load(f.read())
	f.close()
	n=len(d['cb_torque'])-1
	#poly,inv_poly=m3tc.get_polyfit_to_data(x,y,n)
	s=m3tc.PolyEval(d['cb_torque'],d['log_adc_torque'])
	m3t.mplot2(range(len(d['log_adc_torque'])),d['log_load_mNm'],s,xlabel='Samples',ylabel='Torque (mNm)')
except (IOError, EOFError):
	print 'Raw data file',fn,'not available'
