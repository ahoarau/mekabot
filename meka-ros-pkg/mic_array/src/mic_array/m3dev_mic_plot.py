#! /usr/bin/python
from pylab import *
import Gnuplot
import m3.toolbox as m3t
import u3
from time import sleep
from datetime import datetime
import struct
import math
#import numpy
from scipy.signal import convolve, remez
from scipy import array
# MAX_REQUESTS is the number of packets to be read.
# At high frequencies ( >5 kHz), the number of samples will be MAX_REQUESTS times 48 (packets per request) times 25 (samples per packet)

class RingBuffer:
    def __init__(self, size):
        self.data = [None for i in xrange(size)]

    def append(self, x):
        self.data.pop(0)
        self.data.append(x)

    def get(self):
        return self.data

d = u3.U3()
#d.configIO(FIOAnalog = 1)
num_chan=8
samp_freq=math.floor(10000/8)

filt = remez(40, array([0, 70, 71, 270, 271, math.floor(samp_freq/2)]),
            array([0, 1, 0]), Hz = samp_freq)

p = 2
scale = 100.0
window_size = int(math.floor(0.1*samp_freq))
#FIOEIOAnalog = ( 2 ** num_chan ) - 1
FIOEIOAnalog = ( 2 ** 8 ) - 1
fios = FIOEIOAnalog & (0xFF)
d.configIO( FIOAnalog = fios )
if window_size < 400:
    buf_size = 400
else:
    buf_size = window_size
buf_0 = RingBuffer(buf_size)
buf_1 = RingBuffer(buf_size)
buf_2 = RingBuffer(buf_size)
buf_3 = RingBuffer(buf_size)
buf_4 = RingBuffer(buf_size)
buf_5 = RingBuffer(buf_size)
buf_6 = RingBuffer(buf_size)
buf_7 = RingBuffer(buf_size)


gain_0 = 1.0
gain_1 = 1.0
gain_2 = 1.0
#for i in range(num_chan):
for j in range(buf_size):
    buf_0.append(0.0)
    buf_1.append(0.0)
    buf_2.append(0.0)
    buf_3.append(0.0)
    buf_4.append(0.0)
    buf_5.append(0.0)
    buf_6.append(0.0)
    buf_7.append(0.0)
    
#d.streamConfig(NumChannels = num_chan, SamplesPerPacket = 25, PChannels = range(num_chan), NChannels = [ 31 ]*num_chan, Resolution = 2, SampleFrequency = samp_freq )
d.streamConfig(NumChannels = 8, PChannels = range(8), NChannels = [ 31 ]*8, Resolution = 1, SampleFrequency = samp_freq)

print "samps per pkt:", d.streamSamplesPerPacket
print "pkts per req:", d.packetsPerRequest
#buff_size = (d.streamSamplesPerPacket * d.packetsPerRequest)/6
#print "buff size:", buff_size
energy = [0.0] * num_chan
#scope_1 = m3t.M3Scope()
#scope_2 = m3t.M3Scope()
#scope_3 = m3t.M3Scope()
x=range(buf_size)
y=[0.0]*len(x)
y_lim=[0.0, 150.0]
g0=None
g1=None
g2=None
g3=None
g4=None
g5=None
g6=None
g7=None

#g = Gnuplot.Gnuplot(persist = 1)
#g.title('M3 Plot')
#g('set data style lines')
#g('set term x11 noraise')
#yrange =y_lim
#g('set yrange ['+str(yrange[0])+':'+str(yrange[1])+']')

g0 = m3t.gplot(y,x,g0,y_lim)
g0.title('mic 0')
g1 = m3t.gplot(y,x,g1,y_lim)
g1.title('mic 1')
g2 = m3t.gplot(y,x,g2,y_lim)
g2.title('mic 2')
g3 = m3t.gplot(y,x,g3,y_lim)
g3.title('mic 3')
g4 = m3t.gplot(y,x,g4,y_lim)
g4.title('mic 4')
g5 = m3t.gplot(y,x,g5,y_lim)
g5.title('mic 5')
g6 = m3t.gplot(y,x,g6,y_lim)
g6.title('mic 6')
g7 = m3t.gplot(y,x,g7,y_lim)
g7.title('mic 7')


try:
    start = datetime.now()
    
    d.streamStart()
    
    missed = 0
    start = datetime.now()
    dataCount = 0
    byteCount = 0
    while (True):
        for r in d.streamData():
            if r is not None:
                # Our stop condition
                            
                if r['errors'] != 0:
                    print "Error: %s ; " % r['errors'], datetime.now()
    
                if r['numPackets'] != d.packetsPerRequest:
                    print "----- UNDERFLOW : %s : " % r['numPackets'], datetime.now()
    
                if r['missed'] != 0:
                    missed += r['missed']
                    print "+++ Missed ", r['missed']
    
                for i in range(len(r['AIN0'])):                    
                    #print "V:", r['AIN0'][i], r['AIN1'][i], r['AIN2'][i]
                    buf_0.append(((r['AIN0'][i])*scale*gain_0))
                    buf_1.append(((r['AIN1'][i])*scale*gain_1))
                    buf_2.append(((r['AIN2'][i])*scale*gain_2))
                    buf_3.append(((r['AIN3'][i])*scale*gain_0))
                    buf_4.append(((r['AIN4'][i])*scale*gain_1))
                    buf_5.append(((r['AIN5'][i])*scale*gain_2))
                    buf_6.append(((r['AIN6'][i])*scale*gain_0))
                    buf_7.append(((r['AIN7'][i])*scale*gain_1))

            
                dataCount += 1
                
                                  
                buf_0_filt = convolve(filt, buf_0.get())
                buf_1_filt = convolve(filt, buf_1.get())
                buf_2_filt = convolve(filt, buf_2.get())
                buf_3_filt = convolve(filt, buf_3.get())
                buf_4_filt = convolve(filt, buf_4.get())
                buf_5_filt = convolve(filt, buf_5.get())
                buf_6_filt = convolve(filt, buf_6.get())
                buf_7_filt = convolve(filt, buf_7.get())
                
                for i in range(len(buf_0_filt)):
                    buf_0_filt[i] = buf_0_filt[i] ** p
                    buf_1_filt[i] = buf_1_filt[i] ** p
                    buf_2_filt[i] = buf_2_filt[i] ** p
                    buf_3_filt[i] = buf_3_filt[i] ** p
                    buf_4_filt[i] = buf_4_filt[i] ** p
                    buf_5_filt[i] = buf_5_filt[i] ** p
                    buf_6_filt[i] = buf_6_filt[i] ** p
                    buf_7_filt[i] = buf_7_filt[i] ** p
                    
                    
                #for i in range(len(r['AIN0'])):
                    #scope_1.plot(buf_0_filt[len(buf_0_filt)-len(r['AIN0'])])
                '''g0 = m3t.gplot(buf_0_filt,x,g0,y_lim)
                g1 = m3t.gplot(buf_1_filt,x,g1,y_lim)
                g2 = m3t.gplot(buf_2_filt,x,g2,y_lim)'''
                
                g0 = m3t.gplot(buf_0_filt,x,g0,y_lim)
                g1 = m3t.gplot(buf_1_filt,x,g1,y_lim)
                g2 = m3t.gplot(buf_2_filt,x,g2,y_lim)
                g3 = m3t.gplot(buf_3_filt,x,g3,y_lim)
                g4 = m3t.gplot(buf_4_filt,x,g4,y_lim)
                g5 = m3t.gplot(buf_5_filt,x,g5,y_lim)
                g6 = m3t.gplot(buf_6_filt,x,g6,y_lim)
                g7 = m3t.gplot(buf_7_filt,x,g7,y_lim)
                
                #g.plot(zip(x,buf_0_filt,x,buf_1_filt,x,buf_2_filt))
                
                avg_e_0 = []
                avg_e_1 = []
                avg_e_2 = []
                    
                T_all = []
                for i in range(buf_size-window_size):
                    if i % 2 == 0:
                        energy[0] = sum(buf_0_filt[i:(i+window_size)])/window_size
                        energy[1] = sum(buf_1_filt[i:(i+window_size)])/window_size
                        energy[2] = sum(buf_2_filt[i:(i+window_size)])/window_size                    
                        #print energy
                        D = 0.0
                        B = 0.0
                        A = 0.0
                        thresh = 11.0
                       
                        baseline = 0.0
                        baseline = 99999999.0
                        for i in range(len(energy)):
                            if energy[i] < baseline:
                                baseline = energy[i]   
                        
                        energy[0] = energy[0] - baseline
                        energy[1] = energy[1] - baseline
                        energy[2] = energy[2] - baseline
                        
                        avg_e_0.append(energy[0])
                        avg_e_1.append(energy[1])
                        avg_e_2.append(energy[2])
                        
                        if sum(energy) > thresh:
                            if (energy[0]+energy[1]) > (energy[1]+energy[2]):
                                D = -30.0
                                A = energy[0]
                                B = energy[1]
                            else:
                                D = 30.0
                                A = energy[1]
                                B = energy[2]
                            T = D + 30.0*(B**2-A**2)/(B**2+A**2)
                            print "Detected: ", T, A, B, D
                            print "Energy:", energy
                            T_all.append(T)
                            break
                #if len(T_all) > 0:
                #    print "Avg T:", sum(T_all)/len(T_all)
                            
finally:
    
    stop = datetime.now()
    d.streamStop()
    d.close()

    total = dataCount * d.packetsPerRequest * d.streamSamplesPerPacket
    print "%s requests with %s packets per request with %s samples per request = %s samples total." % ( dataCount, d.packetsPerRequest, d.streamSamplesPerPacket, total )
    print "%s samples were lost due to errors." % missed
    total -= missed
    print "Adjusted number of samples = %s" % total
    
    runTime = (stop-start).seconds + float((stop-start).microseconds)/1000000
    print "The experiment took %s seconds." % runTime
    print "%s samples / %s seconds = %s Hz" % ( total, runTime, float(total)/runTime )
    
    
