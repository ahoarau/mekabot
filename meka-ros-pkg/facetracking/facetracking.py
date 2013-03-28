#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
Two-steps vision program:
  1. detect one face, draw a red rectangle around it (Haar cascades)
  2. once detected, it tracks it using Camshift  (green ellipse). If track fails,
     it goes back to step 1. in order to get a new face rectangle.
'''

'''import roslib
roslib.load_manifest('n900_cam')'''
import roslib; 	roslib.load_manifest('face_detector_mono')
import sys
import rospy
import cv
import os
import time
from std_msgs.msg import String
from roslib.msg import Header
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from face_detector_mono.msg import RectArray
from face_detector_mono.msg import Rect

min_size = (20, 20)
image_scale = 2
haar_scale = 1.2
min_neighbors = 2
haar_flags = 0

def is_rect_nonzero(r):
    (_,_,w,h) = r
    return (w > 0) and (h > 0)

class face_track(object):

  def __init__(self):
    self.track_window = None
    self.tracking = False
    self.rects = []
    self.hist = None
    self.bridge = CvBridge()
    rospy.init_node('face_tracker', anonymous=True)
    self.image_sub = rospy.Subscriber("/camera2/camera/image_rect_color",Image,self.callback)
    self.cascade = cv.Load(os.getenv("M3_ROBOT") + "/ros_config/cameras/haarcascade_frontalface_alt.xml")
    
    self.pub = rospy.Publisher('/facetrack2/faces', RectArray)

  def detect(self,img):
    # allocate temporary images
    gray = cv.CreateImage((img.width,img.height), 8, 1)
    small_img = cv.CreateImage((cv.Round(img.width / image_scale),
  	  	       cv.Round (img.height / image_scale)), 8, 1)
  
    # convert color input image to grayscale
    cv.CvtColor(img, gray, cv.CV_BGR2GRAY)
  
    # scale input image for faster processing
    cv.Resize(gray, small_img, cv.CV_INTER_LINEAR)
  
    cv.EqualizeHist(small_img, small_img)
  
    rects = []
    if(self.cascade):
        t = cv.GetTickCount()
        faces = cv.HaarDetectObjects(small_img, self.cascade, cv.CreateMemStorage(0),
                                     haar_scale, min_neighbors, haar_flags, min_size)
        t = cv.GetTickCount() - t
        if faces:
            for ((x, y, w, h), n) in faces:
                # the input to cv.HaarDetectObjects was resized, so scale the 
                # bounding box of each face and convert it to two CvPoints
                pt1 = (int(x * image_scale), int(y * image_scale))
                pt2 = (int((x + w) * image_scale), int((y + h) * image_scale))
                cv.Rectangle(img, pt1, pt2, cv.RGB(255, 0, 0), 1, 8, 0)
                # convert to cvRect: (x,y,width,height)
                rects.append((pt1[0],pt1[1],pt2[0]-pt1[0],pt2[1]-pt1[1]))
                #rects.append((x,y,w,h))
      
    return img,rects

  def hue_histogram_as_image(self, hist):
      """ Returns a nice representation of a hue histogram """

      histimg_hsv = cv.CreateImage( (320,200), 8, 3)
      
      mybins = cv.CloneMatND(hist.bins)
      cv.Log(mybins, mybins)
      (_, hi, _, _) = cv.MinMaxLoc(mybins)
      cv.ConvertScale(mybins, mybins, 255. / hi)

      w,h = cv.GetSize(histimg_hsv)
      hdims = cv.GetDims(mybins)[0]
      for x in range(w):
          xh = (180 * x) / (w - 1)  # hue sweeps from 0-180 across the image
          val = int(mybins[int(hdims * x / w)] * h / 255)
          cv.Rectangle( histimg_hsv, (x, 0), (x, h-val), (xh,255,64), -1)
          cv.Rectangle( histimg_hsv, (x, h-val), (x, h), (xh,255,255), -1)

      histimg = cv.CreateImage( (320,200), 8, 3)
      cv.CvtColor(histimg_hsv, histimg, cv.CV_HSV2BGR)
      return histimg

  def track(self,img,init_rect):
      gray = cv.CreateImage((img.width,img.height), 8, 1)
      # convert color input image to grayscale
      cv.CvtColor(img, gray, cv.CV_BGR2GRAY)
      if not self.hist:
          self.hist = cv.CreateHist([80], cv.CV_HIST_ARRAY, [(0,80)], 1 )
      # init with what detect() found if not tracking with existing data
      if not self.track_window or not is_rect_nonzero(self.track_window):
          print "BAD TRACK WINDOW: %s" % repr(self.track_window)
          self.track_window = init_rect

      self.tracking = True
      # Convert to HSV and keep the hue
      ##hsv = cv.CreateImage(cv.GetSize(img), 8, 3)
      hsv = cv.CreateImage(cv.GetSize(gray), 8, 3)
      cv.CvtColor(img, hsv, cv.CV_BGR2HSV)
      self.hue = cv.CreateImage(cv.GetSize(img), 8, 1)
      self.sat = cv.CreateImage(cv.GetSize(img), 8, 1)
      self.value = cv.CreateImage(cv.GetSize(img), 8, 1)
      cv.Split(hsv, self.hue, self.sat, self.value, None)

      # Compute back projection
      backproject = cv.CreateImage(cv.GetSize(img), 8, 1)

      # Run the cam-shift
      cv.CalcBackProject( [self.hue], backproject, self.hist )
      #cv.ShowImage("Hist pre",self.hue_histogram_as_image(self.hist))
      cv.NormalizeHist(self.hist,100)
      #cv.ShowImage("Hist norm",self.hue_histogram_as_image(self.hist))
      #crit = ( cv.CV_TERMCRIT_EPS | cv.CV_TERMCRIT_ITER, 10, 0.01)
      crit = ( cv.CV_TERMCRIT_EPS, 0, 0.01)
      '''print "track_window: %s" % repr(self.track_window)
      print "init_rect: %s" % repr(init_rect)'''
      (iters, (area, value, rect), track_box) = cv.CamShift(backproject,self.track_window , crit)
      #print "\titers: %s, aera: %s, value: %s, rect: %s, track_box: %s" % (iters,area,value,rect,track_box)
      self.track_window = rect
      #print "new track_window: %s" % repr(self.track_window)

      sel = cv.GetSubRect(self.hue,init_rect)
      cv.CalcArrHist( [sel], self.hist, 0)
      (_, max_val, _, _) = cv.GetMinMaxHistValue(self.hist)
      #print "max_val: %s" % max_val
      if max_val != 0:
          cv.ConvertScale(self.hist.bins, self.hist.bins, 255. / max_val)

      cv.EllipseBox( img, track_box, cv.CV_RGB(0,255,0), 3, cv.CV_AA, 0 )
      '''print "track_box[0]: %s" % repr(track_box[0])
      print "track_box[1]: %s" % repr(track_box[1])'''
      x,y = track_box[0]
      w,h = (track_box[1][0]-track_box[0][0],track_box[1][1]-track_box[0][1])
      #cv.Rectangle(img, (rect[0],rect[1]),(rect[2],rect[3]), cv.RGB(0, 0, 255), 1, 8, 0)
      cv.Rectangle(img, (x,y),(w,h), cv.RGB(0, 0, 255), 1, 8, 0)

      cv.ShowImage("Face Tracking", img)
      '''cv.ShowImage("BackProject",backproject)
      cv.ShowImage("Hue",self.hue)
      cv.ShowImage("Sat",self.sat)
      cv.ShowImage("Value",self.value)
      cv.ShowImage("Hist final",self.hue_histogram_as_image(self.hist))'''

      if not is_rect_nonzero(self.track_window):
          print "Stop tracking: %s" % repr(self.track_window)
          self.tracking = False
      #return img, self.hue_histogram_as_image(self.hist), [int(x),int(y),int(w),int(h)]
      return img, self.hue_histogram_as_image(self.hist), track_box


  def callback(self,data):
    t0 = time.clock()
    try:
      cv_image = self.bridge.imgmsg_to_cv(data, "bgr8")
    except CvBridgeError, e:
      print e
    track_rect = None
    if self.rects and self.tracking:
        # seb: only first rect aera (I assume only one face on the image)
        cv_detimg,rects = self.detect(cv_image)
        if len(rects) != 0:
            cv.ShowImage("Face Detecting", cv_detimg)        
            self.rects = rects
            self.track_window = None
        cv_trackimg,his, track_rect = self.track(cv_image,self.rects[0])            
        cv.WaitKey(1)
    else:
        cv_detimg,self.rects = self.detect(cv_image)
        cv.ShowImage("Face Detecting", cv_detimg)
        self.tracking = True
        '''if len(self.rects) != 0:            
            track_rect = [[self.rects[0][0], self.rects[0][1]], [self.rects[0][2], self.rects[0][3]]]'''
        cv.WaitKey(1)
    if track_rect != None:
        print track_rect
        self.pub.publish(RectArray(Header(0,rospy.Time.now(),'0'),[Rect(track_rect[0][0],track_rect[0][1],track_rect[1][0],track_rect[1][1])]))
    print 't:', time.clock() - t0


def main(args):
  ic = face_track()
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
