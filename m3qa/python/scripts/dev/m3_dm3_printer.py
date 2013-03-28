#! /usr/bin/python

import serial
import time

ser = serial.Serial('/dev/ttyUSB0', 19200, bytesize=8, parity='N', stopbits=1, timeout=None, xonxoff=0, rtscts=0)
i=0
start=time.time()
while (1):
    line = ser.readline()   # read a '\n' terminated line
    #ser.flushInput()
    #ser.flush()
    print time.time()-start,' : ',line
    #time.sleep(0.1)
#ser.close()
