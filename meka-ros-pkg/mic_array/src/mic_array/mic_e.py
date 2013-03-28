#! /usr/bin/python


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
num_chan=3
samp_freq=math.floor(10000/6)

filt = remez(40, array([0, 70, 71, 270, 271, math.floor(samp_freq/2)]),
            array([0, 1, 0]), Hz = samp_freq)

p = 2
scale = 100.0
window_size = int(math.floor(0.1*samp_freq))
#FIOEIOAnalog = ( 2 ** num_chan ) - 1
FIOEIOAnalog = ( 2 ** 6 ) - 1
fios = FIOEIOAnalog & (0xFF)
d.configIO( FIOAnalog = fios )
if window_size < 400:
    buf_size = 400
else:
    buf_size = window_size
buf_0 = RingBuffer(buf_size)
buf_1 = RingBuffer(buf_size)
buf_2 = RingBuffer(buf_size)

gain_0 = 1.0
gain_1 = 1.0
gain_2 = 0.75
#for i in range(num_chan):
for j in range(buf_size):
    buf_0.append(0.0)
    buf_1.append(0.0)
    buf_2.append(0.0)
#d.streamConfig(NumChannels = num_chan, SamplesPerPacket = 25, PChannels = range(num_chan), NChannels = [ 31 ]*num_chan, Resolution = 2, SampleFrequency = samp_freq )
d.streamConfig(NumChannels = 6, PChannels = range(6), NChannels = [ 31 ]*6, Resolution = 1, SampleFrequency = samp_freq)

print "samps per pkt:", d.streamSamplesPerPacket
print "pkts per req:", d.packetsPerRequest
#buff_size = (d.streamSamplesPerPacket * d.packetsPerRequest)/6
#print "buff size:", buff_size
energy = [0.0] * num_chan
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
            
                dataCount += 1
                                  
                buf_0_filt = convolve(filt, buf_0.get())
                buf_1_filt = convolve(filt, buf_1.get())
                buf_2_filt = convolve(filt, buf_2.get())
                for i in range(len(buf_0_filt)):
                    buf_0_filt[i] = buf_0_filt[i] ** p
                    buf_1_filt[i] = buf_1_filt[i] ** p
                    buf_2_filt[i] = buf_2_filt[i] ** p
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
    
    
