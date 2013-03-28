#M3 -- Meka Robotics Robot Components
#Copyright (c) 2010 Meka Robotics
#Author: edsinger@mekabot.com (Aaron Edsinger)

#M3 is free software: you can redistribute it and/or modify
#it under the terms of the GNU Lesser General Public License as published by
#the Free Software Foundation, either version 3 of the License, or
#(at your option) any later version.

#M3 is distributed in the hope that it will be useful,
#but WITHOUT ANY WARRANTY; without even the implied warranty of
#MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#GNU Lesser General Public License for more details.

#You should have received a copy of the GNU Lesser General Public License
#along with M3.  If not, see <http://www.gnu.org/licenses/>.
import pylab
import math
import numpy as nu
import m3.unit_conversion as m3u
import  scipy 
import numpy as np
import numpy.numarray as na
from scipy import *
import scipy.signal
from pylab import *

# ##############################################
# Useful control related functions
# ##############################################

# make 3x3 rotation matrix out of 9 element list
def rotation_array_to_matrix(r):
        g = np.zeros([9],np.float)	
        g[:]=r[:]
        g.resize([3,3])
        return g

#make 4x4 transform out of 3x3 rotation matrix and 3 element list translation
def rot_trans_to_frame(r, t):
        pass

def position(T):
        """Extract position vector from transform"""
        return T[0:3,3]

def rotation(T):
        """Extract rotation matrix from transform"""
        return T[0:3,0:3]

def PolyFitN(x,y,n,draw=True,verbose=True):
        """Polynomial fit to data of degree n"""
        poly=scipy.polyfit(x,y,n)
        yr=scipy.polyval(poly,x)
        #compute the mean square error
        err=scipy.sqrt(sum((yr-y)**2)/len(y))
	if verbose:
		print('Linear regression using polyfit')
		print 'regression: poly, ms error',poly,err
        #matplotlib ploting
	if draw:
		pylab.title('Linear Regression')
		pylab.plot(x,y,'k.')
		pylab.plot(x,yr,'g.--')
		pylab.legend(['original','regression'])
		pylab.show()
        return poly

def PolyEval(poly,x):
        return scipy.polyval(poly,x)

def get_polyfit_to_data(x,y,n=3,draw=True,verbose=True):
	#Fit degree n poly to data
	xx=np.array(x,na.Float32)
	yy=np.array(y,na.Float32)
	yy=yy[xx.argsort()]
	xx.sort()
	if verbose:
		print 'X',xx
		print 'Y',yy
	poly=PolyFitN(xx,yy,n,draw)
	inv_poly=PolyFitN(yy,xx,n,draw)
	poly=[float(q) for q in poly]
	inv_poly=[float(q) for q in inv_poly]
	return poly,inv_poly

def force_moment_transform(S2T):
        """Get the transfrom that converts a wrench in Frame S to a wrench in Frame T
    From Craig. Intro to Robotics. Ch. 5.
    Input: Homegenous transform S2T converts a point in Frame S to a point in Frame T"""
        #Compute Jt for fh=Jt*ft , for a force a pt in hand coord frame
        R=S2T[:3,:3] #rotation
        pt=S2T[:3,3] #translation
        Px=nu.array([[0,-pt[2],pt[1]],[pt[2],0,-pt[0]],[-pt[1],pt[0],0]],float) #cross product
        t=nu.concatenate([R,nu.zeros([3,3],float)],1)
        tt=nu.concatenate([Px,R],1)
        FS2FT=nu.concatenate([t,tt],0) #6x6
        return FS2FT	

def plot_bode(inputs, outputs, samp_freq):
	dt=1.0/samp_freq
	T = len(inputs)* dt
	t=arange(0.0,T,dt)
	df=1.0/T
	f=arange(0,samp_freq,df)
        print len(f), len(inputs), len(outputs)
	N=len(f)
	ycfft=fft(outputs)*2/N
	ucfft=fft(inputs)*2/N
	tf=ycfft/ucfft
        grid(True)
	subplot(211)
	semilogx(f,20*log10(abs(tf)))
	ylabel('Mag. Ratio (dB)')
        fN=f[int(N/2)]
        xlim([0.1,fN])
        grid(True)
	subplot(212)
	semilogx(f,arctan2(imag(tf),real(tf))*180.0/pi)
        grid(True)
        xlim([0.1,fN])
	ylabel('Phase (deg.)')
	xlabel('Freq. (Hz)')
	show()

#Pass in list of frequencies, the peak-to-peak signal magnitude, and the sine-wave set-point amplitude
#Need to add phase plot...
def plot_bode_simple(freq,p2p,amp,ptitle='Bode'):
	gain=nu.array(p2p)/amp
	cutoff=nu.array([-3]*len(gain)) #70.7% : 10^-.15 : 20*log10(10^-.15)=-3
	semilogx(freq,20*pylab.log10(gain))
	semilogx(freq,cutoff,'r-')
	ylabel('Mag. Ratio (dB)')
	xlabel('Freq. (Hz)')
        title(ptitle)
        grid(True)
        show()








