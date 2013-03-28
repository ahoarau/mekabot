#! /usr/bin/python

#Copyright  2010 Meka Robotics
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

import m3.toolbox_ros as m3tr
import m3.toolbox as m3t
import time

#if __name__ == "__main__":  
    ##main() 
    #print 'Select camera'
    #eye=m3t.user_select_components_interactive(['right','left','middle'])
    #f=m3tr.M3FaceDetectThread(eye)
    #f.start()
    #try:
	#while True:
	    #print time.time()
	    #time.sleep(0.5)
    #except KeyboardInterrupt:
	#pass
    #f.stop()
  
if __name__ == "__main__":  
    f=m3tr.M3MicrophoneArrayThread()
    f.start()
    try:
	while True:
	    print time.time()
	    time.sleep(0.5)
    except KeyboardInterrupt:
	pass
    f.stop()

# ######################################################
    
#def callback(data):
    #if not len(data.rects) == 0:
        #print 'Face detected at:'
        #print data.rects

#def listener(eye):
    #rospy.init_node('m3_face_detect', anonymous=True)
    #if eye=='left':
	#rospy.Subscriber("/facedetect0/faces", RectArray, callback)
    #if eye=='right':
	#rospy.Subscriber("/facedetect1/faces", RectArray, callback)
    #rospy.spin()

#def main(eye='left'):
    ##p1 = subprocess.Popen(['roslaunch', 'm3_defs_ros', eye+'_pt_grey.launch'])
    ##p2 = subprocess.Popen(['roslaunch', 'm3_defs_ros', eye+'_face_detect.launch'])
    #try:
        #listener(eye)
    #except:
        #print 'Exiting...'

    ##os.system("pkill -P " + str(p2.pid))
    ##os.kill(p2.pid,9)
    
    ##os.system("pkill -P " + str(p1.pid))
    ##os.kill(p1.pid,9)

