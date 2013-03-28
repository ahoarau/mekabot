# -*- coding: utf-8 -*-
import cgtypes as cg
import camera_model as cm
import seeim as si
import math
import Numeric as na
import numarray.linear_algebra as nal
import Numeric as nu
from sal_joint_defs import *
#import image_conversion as ic

# ####################################################################################################################################

""" 
[All coord. sys if view facing robot]
 
{H}: hand coord system.
{F}: wrist coord system (aligned with elbow but centered at wrist)
{J}: elbow coord systems
{W}: world coord. system [X: Left, Y: towards, Z: Up]
{T}: eye coord system (eye tilt in kinematic chain of Domo) [X: Towards, Y: Down, Z: Right]
{E}: eye coord system (eye pan in kinematic chain of Domo) [X: Right, Y: Towards, Z: Down]
{C}: standard camera coord system [X: Left, Y:Down, Z towards] (http://homepages.inf.ed.ac.uk/rbf/CVonline/LOCAL_COPIES/OWENS/LECT9/node2.html)
{I}: image coord system, centered at principal point [X:right, Y: down]
{P}: pixel coord. system, translation to upper left. corner from {I}
{G}: gyro coordinate system
{GEO}: earth coordinates system used by gyro (North=X, East=Y, Down=Z), but magnetic north not available
{S}: eye base coordinates system (skull) (rigidly attached to gyro)
{A}: camera coord systems adjusted by calibration [X: Left, Y:Down, Z towards]
{D}: canonical head coordsystem (all theta=0, midpoint between two eyes, no rotation from camera frames)
Matrix aRb*xb returns vector x, in coord system B, as viewed from coord system A
"""
 
class KinematicsHeadCamera:
         """Kinematic transforms for image to world and back"""
         def __init__(self,w=320,h=240,load_calibration=True):
                  self.h=int(h)
                  self.w=int(w)
                  self.cam_id={'right_eye':'domo_2_8mm_right_eye', 'left_eye':'domo_2_8mm_left_eye'}
                  self.hkine=KinematicsHand()
                  self.f={}
                  self.px={}
                  self.py={}
                  self.wRh = {}
                  self.wRj={}
                  self.wRf ={}
                  self.fRw={}
                  self.hRw = {}
                  self.jRw={}
                  self.wRe = {}
                  self.wRc={}
                  self.cRw={}
                  self.eRa={}
                  self.jointpos={}
		  
                  self.wRc_prev={}
                  self.wRh_prev={}
                  self.hRw_prev={}
                  self.cRw_prev={}
                  self.hRc={}
                  self.wRt = cg.mat4(1)
                  self.wRs= cg.mat4(1)
                  self.J={}
                  self.eRc={}
                  self.cRe={}
                  self.geoRg=cg.mat4(1)
                  self.camRcam={}
                  self.theta={}
                  self.stereo_baseline=None
                  for e in ['left_eye','right_eye']:
                           self.wRe[e] = cg.mat4(1)
                           self.wRc[e]=  cg.mat4(1)
                           self.cRw[e]=  cg.mat4(1)
                           self.wRc_prev[e]=  cg.mat4(1)
                           self.cRw_prev[e]=  cg.mat4(1)
                           self.eRc[e]=cg.mat4().rotation(math.pi, cg.vec3(0.0, 0.0, 1.0))*cg.mat4().rotation(math.pi/2, cg.vec3(1.0, 0.0, 0.0)) #from standard camera frame {C} to domo eye frame {E}
                           self.cRe[e]=self.eRc[e].transpose()
                           self.camRcam[e]=cg.mat4(1)
                           self.px[e], self.py[e], self.f[e]= cm.camera_parameters(si.RGBIm(self.h,self.w),self.cam_id[e])
                           self.eRa[e]= cg.mat4(1)
                                  
                  self.theta['head']=[0.0]*8
		  self.jointpos['head']=[0.0]*8*3
                  self.gyro_ang=[0.0]*3
                  
                  for a in ['left_arm','right_arm']:
                           self.wRh[a] = cg.mat4(1)
                           self.hRw[a] = cg.mat4(1)
                           self.wRj[a] = cg.mat4(1)
                           self.jRw[a] = cg.mat4(1)
                           self.fRw[a] = cg.mat4(1)
                           self.wRf[a] = cg.mat4(1)
                           self.hRw_prev[a]=cg.mat4(1)
                           self.wRh_prev[a]=cg.mat4(1)
                           self.hRc=cg.mat4(1)
                           self.J[a]=nu.zeros([6,6],nu.Float)   
			   self.theta[a]=[0.0]*6
			   self.jointpos[a]=[0.0]*6*3
			   
		  for h in ['left_hand','right_hand']:
			   self.theta[h]=[0.0]*4

                  self.eye_name='right_eye'
                  self.arm_name='right_arm'
                                     
                  self.x_axis = cg.vec3(1.0, 0.0, 0.0)
                  self.y_axis = cg.vec3(0.0, 1.0, 0.0)
                  self.z_axis = cg.vec3(0.0, 0.0, 1.0)
                  
                  if False:
                           #Auto construct wRd
                           import sal_robot as sr
                           import sal_joint_defs as sj
                           kine=sr.SalKine()
                           kine.step_head_forward([0.0]*8)
                           jp=list(kine.prio['head']['jointpos'])
                           jr=cg.vec3(jp[sj.jEyePanRight*3:sj.jEyePanRight*3+3])
                           jl=cg.vec3(jp[sj.jEyePanLeft*3:sj.jEyePanLeft*3+3])
                           j=cg.vec4(list((jr+jl)/2.0)+[1])
                           self.wRd=cg.mat4(1)
                           self.wRd.setColumn(3,j)
                           self.dRw=self.wRd.inverse()
                  else:
                           #Precomputed
                           self.wRd=cg.mat4(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 2.4159999999999999, 0.0, -1.0, 0.0, 8.9250000000000007, 0.0, 0.0, 0.0, 1.0)
                           self.dRw=self.wRd.inverse()
                           
                           
                  if load_calibration:
                           self.load_calibration()
         # ######### Public    ########
         """ Inputs are sequences. Outputs are lists. """		  
                  
         #All functions use the current arm_name/eye_name. Change them with these
         
         def set_arm(self,a):
                  self.arm_name=a
                  self.hkine.set_hand(a.split('_')[0]+'_hand')
                  
         def set_eye(self,e):
                  self.eye_name=e
                            
         def set_image_size(self,w,h):
                  self.w=w
                  self.h=h
                  for e in ['right_eye','left_eye']:
                           self.px[e], self.py[e], self.f[e]= cm.camera_parameters(si.RGBIm(self.h,self.w),self.cam_id[e])
	 
	 def pixel_in_image(self,px):
		  return px[0]>=0 and px[0]<self.w and px[1]>-0 and px[1]<self.h
	 
         def pixel2world(self,p,r=100.0):
                  """r is the distance along the ray passing through pixel , in world units"""
                  w= self.image2world(self.pixel_to_image_frame(p),r)
                  return list(w)[0:3]
                  
         def world2pixel(self,wx):
                  return self.image_frame_to_pixel(self.world2image(cg.vec4(list(wx)+[1.0])))
 
         
         def pixel2hand(self,p,r):
                  wx= self.image2world(self.pixel_to_image_frame(p),r)
                  hx = self.hRw[self.arm_name]*wx
                  return list(hx)[0:3]
         
         def pixel2elbow(self,p,r):
                  wx= self.image2world(self.pixel_to_image_frame(p),r)
                  jx = self.jRw[self.arm_name]*wx
                  return list(jx)[0:3]
         
         def pixel2wrist(self,p,r):
                  wx= self.image2world(self.pixel_to_image_frame(p),r)
                  fx = self.fRw[self.arm_name]*wx
                  return list(fx)[0:3]
         
         def hand_center2pixel(self):
                  return self.hand2pixel([0,0,0])
         
         def hand_visible(self,hx=None):
                  if hx is None:
                           hx=[0,0,0]
                  c= self.hand2pixel(hx)
                  return self.pixel_in_image(c)
         
         def hand2pixel(self,hx):
		  wx = self.wRh[self.arm_name]*cg.vec4(list(hx)+[1.0])
                  p = self.image_frame_to_pixel(self.world2image(wx))
                  return p
         
         def wrist2pixel(self,fx): #the wrist frame aligned with elbow but translated to wrist (z: elbow axis, y: forearm to elbow;-x orthog. towards top of forearm)
                  wx=self.wRf[self.arm_name]*cg.vec4(list(fx)+[1.0])
                  p = self.image_frame_to_pixel(self.world2image(wx))
                  return p

	 def elbow2world(self,jx):
		  wx = self.wRj[self.arm_name]*cg.vec4(list(jx)+[1.0])
		  return list(wx)[0:3]
	 
         def elbow2pixel(self,jx):
                  wx = self.wRj[self.arm_name]*cg.vec4(list(jx)+[1.0])
                  p = self.image_frame_to_pixel(self.world2image(wx))
                  return p
         
         def wrist2world(self,fx): 
                  wx=self.wRf[self.arm_name]*cg.vec4(list(fx)+[1.0])
                  return list(wx)[0:3]

         
         def hand2world(self,hx):
                  wx = self.wRh[self.arm_name]*cg.vec4(list(hx)+[1.0])
                  return list(wx)[0:3]
         
         def pixel2camera(self,px,r=100.0):
                  cx=self.image2camera(self.pixel_to_image_frame(px),r)
                  return list(cx)[0:3]
         
         def world2hand(self,wx):
                  hx = self.hRw[self.arm_name]*cg.vec4(list(wx)+[1.0])
                  return list(hx)[0:3]
         
         def hand2camera(self,hx):
                  cRh = self.cRw[self.eye_name]*self.wRh[self.arm_name]
                  cx = cRh *cg.vec4(list(hx)+[1.0])
                  return list(cx)[0:3]
         
         def camera2hand(self,cx):
                  hRc = self.hRw[self.arm_name]*self.wRc[self.eye_name]
                  ch = hRc*cg.vec4(list(cx)+[1.0])
                  return list(ch)[0:3]
         
         def distance2camera(self,wx):
                  """wx in world frame, distance to camera frame, in world units"""
                  cx = self.world2camera(cg.vec4(list(wx)+[1.0]))
                  cx[3]=0
                  return cx.length()
         
         def fingertip2world(self,finger,wf=None):
                  return self.hkine.fingertip2world(finger,wf)
         
         def fingertip2pixel(self,finger,wf=None):
                  xw=self.fingertip2world(finger,wf)
                  return self.world2pixel(xw)
	 
	 def fingertip2camera(self,finger,wf):
		  wx=cg.vec4(list(self.fingertip2world(finger,wf))+[1.0])
		  cx=self.world2camera(wx)
                  return list(cx)[0:3]
	 
	 def joint2world(self,jointname,joint_idx):
		  return self.jointpos[jointname][3*joint_idx:3*joint_idx+3]
	 
         # ##########################
         """
         Head canonicalposition:  all theta= zero, looking straight ahead, 
         Spherical coordinate frame is centered at midpoint between the two eyes in cannonical position
         Spherical coords are in range: [yaw,pitch,r]=[[-pi,+pi],[-pi,+pi],0-inf]
         Defined as yaw followed by pitch
         yaw=positive in direction of right arm
         pitch=positive in direction of ground
         """
         def pixel2world_spherical(self,p,r=100.0,size=None):
                  if size is not None:
                           q=[p[0]*self.w/size[0],p[1]*self.h/size[1]]
                  else:
                           q=p
                  cx=self.image2camera(self.pixel_to_image_frame(q),r)
                  wx = self.wRc[self.eye_name]*cx
                  dx = self.dRw*wx
                  sph=self.cartesian2spherical(dx)
                  return sph #yaw,pitch,r

         
         def world_spherical2pixel(self,s,size=None): #s: yaw,pitch,r
                  dx=cg.vec4(self.spherical2cartesian(s)+[1]) 
                  wx=self.wRd*dx
                  cx = self.cRw[self.eye_name]*wx
                  p=self.image_frame_to_pixel(self.camera2image(cx))
                  if size is not None:
                           return [p[0]*size[0]/self.w,p[1]*size[1]/self.h]
                  else:
                           return p
      
         
         def world_spherical2world(self,s):
                  dx=cg.vec4(self.spherical2cartesian(s)+[1]) 
                  wx=self.wRd*dx
                  return list(wx)[0:3]

         def world2world_spherical(self,xw):
                  dx=self.dRw*cg.vec4(list(xw)+[1])
                  sph=self.cartesian2spherical(dx)
                  return sph
         
         def gaze2world_spherical(self,r): #where midpoint between two eyes intersects the egosphere
                  p0=cg.vec3(list(self.wRt*cg.vec4([0,0,0,1]))[0:3])
                  p1=cg.vec3(list(self.wRt*cg.vec4([1,0,0,1]))[0:3]) #x points from between eyes out
                  d=p1-p0
                  pc=cg.vec3(list(self.wRd*cg.vec4([0,0,0,1]))[0:3])
                  pa,pb=self.intersection_ray_sphere(p0,d,pc,r)
                  if pa[1]>0: #y direction should be pointing to forward half of egosphere 
                           pi=pa
                  if pb[1]>0:
                           pi=pb 
                  dx=self.dRw*cg.vec4(list(pi)+[1])
                  sph=self.cartesian2spherical(dx)                 
                  return sph
         
	 def  gaze2tool_spherical(self,tool): #Spherical coordinates of camera gaze on the tooltip
		  th=cg.vec3(tool)		  
		  ah=cg.vec3(self.camera2hand([0,0,0]))
		  at=ah-th #the camera in tool tip frame
		  theta,phi,rho=self.cartesian2spherical(at)
		  return theta,phi,rho
		  
         #http://itc.utk.edu/itc/grants/twt2000/modules/mbreinig/3dcoordinates.htm
         def cartesian2spherical(self,c):
                  x=c[0]
                  y=c[1]
                  z=c[2] 
                  rho = math.sqrt(x**2 + y**2 + z**2)
                  theta = math.atan2(x,z) #math.atan2(y, x)
                  if rho != 0.0:
                           phi = math.asin( y / rho ) #math.acos( z / rho )
                  else:
                           print 'Bad cartesian2spherical',c,x,y,z,rho,theta
                           phi = 0 #math.pi / 2.0 * math.sgn(y) #math.pi / 2.0 * math.sgn(z)
                  return [theta,phi,rho]  #yaw,pitch,distance
                           
         def spherical2cartesian(self,s):
                  theta=s[0] #zenith  (yaw)
                  phi=s[1]   #azimuth (pitch)
                  rho=s[2] #distance
                  x = rho * math.cos(phi) * math.sin(theta) #rho * math.sin(phi) * math.cos(theta)
                  y = rho * math.sin(phi) #rho * math.sin(phi) * math.sin(theta)
                  z = rho * math.cos(phi) * math.cos(theta) #rho * math.cos(phi)
                  return [x,y,z]
         # ##########################
         
         """See Notes on binocular disparity, Ben Backus. online"""
         def vergence_angle(self): 
                 return self.theta['head'][7]-self.theta['head'][6] #pan r-l
        
         def vergence_depth(self):#cyclopean depth from midpoint of baseline		  
                  return self.stereo_baseline/(2.0*math.tan(math.pi*self.vergence_angle()/360.0))
         
         def headcentric_disparity(self,par,pal):
                  tar=math.atan2(par[0]-self.w/2,self.f['right_eye']) #angle betwee optical axis and pixel
                  tal=math.atan2(pal[0]-self.w/2,self.f['left_eye'])
                  return 180.0*(tal-tar)/math.pi #more positive, closer in
         
         def vergence_relative(self,par,pal,pbr,pbl):
                  """Positive relative vergence if A is closer than B"""
                  tar=math.atan2(par[0],self.f['right_eye'])
                  tbr=math.atan2(pbr[0],self.f['right_eye'])
                  tal=math.atan2(pal[0],self.f['left_eye'])
                  tbl=math.atan2(pbl[0],self.f['left_eye'])
                  return 180.0*((tar-tal)-(tbr-tbl))/math.pi
                                               
         def stereo_pixel2world(self,pl,pr):
                  old_eye=self.eye_name
                  self.eye_name='left_eye'
                  a0=self.pixel2world(pl,r=0.0)
                  a1=self.pixel2world(pl,r=100.0)
                  self.eye_name='right_eye'
                  b0=self.pixel2world(pr,r=0.0)
                  b1=self.pixel2world(pr,r=100.0)
                  self.eye_name=old_eye
                  
                  wp,dist=self.min_point_rays([a0,a1],[b0,b1])
                  return wp
         
         def epipolar_poly(self,dest_eye,src_eye,px_src,start_z,end_z,width):
		  a=cg.vec3(self.stereo_pixel2pixel(dest_eye,src_eye,px_src,start_z))
		  b=cg.vec3(self.stereo_pixel2pixel(dest_eye,src_eye,px_src,end_z))
		  if (a-b).length()>0:
			   delta=(a-b).ortho()
			   delta=delta.normalize()*width/2.0
			   r=[list(a-delta)[0:2],list(b-delta)[0:2],list(b+delta)[0:2],list(a+delta)[0:2]]
			   return r
		  return None
	 
         def epipolar_line(self,dest_eye,src_eye,px):
                  #line  through points [a,b], in dest eye,  for pixel px in src_eye
                  a=self.stereo_pixel2pixel(dest_eye,src_eye,px,0.0)
                  b=self.stereo_pixel2pixel(dest_eye,src_eye,px,100.0)
                  return a,b
         
         def stereo_pixel2pixel(self,dest_eye,src_eye,px,r=100.0):
                  #r is depth on src eye coord system
                  old_eye=self.eye_name
                  self.eye_name=src_eye
                  a0=self.pixel2world(px,r)
                  self.eye_name=dest_eye
                  pxr=self.world2pixel(a0)
                  self.eye_name=old_eye
                  return pxr
                           
         def cartesian_estimate_of_camera_rotation(self):
                  """amount unit vector translates centered at camera, between frames"""
                  w1=cg.vec3(list(self.wRc[self.eye_name]*cg.vec4([0,0,1,0]))[0:3])
                  w0=cg.vec3(list(self.wRc_prev[self.eye_name]*cg.vec4([0,0,1,0]))[0:3])
                  return w1-w0
       
         def affine_estimate_of_camera_rotation(self,prev_prio=None):
                  """estimate the optic flow from rotation between current camera pose and previous as an affine matrix
                  with formed from the top two rows of the rotation matrix (assumes large depth)
                  returned matrix operates on raw pixel coordinats (u,v,1) """
                  if prev_prio is not None:
                           wRe,wRc,cRw=self.compute_wRc(self.eye_name,prev_prio)
                           self.wRc_prev[self.eye_name]=wRc
                           
                  c2Rc1 = (self.cRw[self.eye_name].getMat3())*(self.wRc_prev[self.eye_name].getMat3())
                  a=na.array(list(c2Rc1.getRow(0))+list(c2Rc1.getRow(1)))
                  a.shape=[2,3]
                  cx=self.w/2.0
                  cy=self.h/2.0
                  f=self.f[self.eye_name]
                  #transform such operates on raw pixels (cck thesis, pg82)
                  a[0,2]=f*a[0,2]-(a[0,0]*cx+a[0,1]*cy)+cx
                  a[1,2]=f*a[1,2]-(a[1,0]*cx+a[1,1]*cy)+cy
                  return a
         
         def affine_estimate_of_hand_rotation(self):
                  """estimate the optic flow from rotation between current hand pose and previous as an affine matrix
                  with formed from the top two rows of the rotation matrix (assumes large depth)
                  returned matrix operates on raw pixel coordinats (u,v,1) """
                  h1Rc1=self.hRw_prev[self.arm_name]*self.wRc_prev[self.eye_name]
                  c2Rh2=self.cRw[self.eye_name]*self.wRh[self.arm_name]
                  c2Rc1=c2Rh2*h1Rc1 #virtual camera rotation between frames given hand movement
                  c2Rc1=c2Rc1.getMat3()
                  a=na.array(list(c2Rc1.getRow(0))+list(c2Rc1.getRow(1)))
                  a.shape=[2,3]
                  cx=self.w/2.0
                  cy=self.h/2.0
                  f=self.f[self.eye_name]
                  a[0,2]=f*a[0,2]-(a[0,0]*cx+a[0,1]*cy)+cx
                  a[1,2]=f*a[1,2]-(a[1,0]*cx+a[1,1]*cy)+cy
                  return a
         
         def affine_estimate_of_hand_translation(self,hx=[0,0,0]):
                  px=self.hand2pixel(hx)
                  wx = self.wRh_prev[self.arm_name]*cg.vec4(list(hx)+[1.0])
                  cx=self.cRw_prev[self.eye_name]*wx
                  px_prev=self.image_frame_to_pixel(self.camera2image(cx))
		  aff_est=na.array([[1,0,px[0]-px_prev[0]],[0,1,px[1]-px_prev[1]]],na.Float)
                  return aff_est
         
         
         #These are not strictly correct but good for small angles
	 # ######################################
	 
	 def tool_camera_align(self,tool,palm=[0,0,0],debug=False):
		  #make tool point normal to image plane
		  ca=cg.vec3(list(self.camera2world(cg.vec4([0,0,0,1])))[0:3])
		  cb=cg.vec3(list(self.camera2world(cg.vec4([0,0,1,1])))[0:3])
		  delta_roll,delta_pitch= self.tool_vector_align(tool,ca-cb,palm,debug)
		  return delta_roll,delta_pitch
	 	 
	 def tool_camera_point(self,tool,palm=[0,0,0],debug=False):
		  #make tool point at camera
		  ca=cg.vec3(list(self.camera2world(cg.vec4([0,0,0,1])))[0:3])
		  cb=cg.vec3(self.hand2world(palm))
		  delta_roll,delta_pitch= self.tool_vector_align(tool,(ca-cb).normalize(),palm,debug)
		  return delta_roll,delta_pitch
	 
	 def tool_vector_align(self,tool,v,palm=[0,0,0],debug=False): 
		  # align tool direction with a unit vector in world coords
		  #get minimal axis-angle rotation
		  #project axis  onto reach joint rotation axis		  
		  target=cg.vec3(self.world2hand(cg.vec3(self.hand2world([0,0,0]))+cg.vec3(v).normalize())) #vector in hand frame
		  tool=(cg.vec3(tool)-cg.vec3(palm)).normalize()#tool in hand frame
		  a=180.0*tool.angle(target)/math.pi #angle between
		  try:
			   n=tool.cross(target).normalize()
		  except ZeroDivisionError:
			   return 0.0,0.0
		  if debug:
			   print 'Target',target
			   print 'Vector',v
			   print 'Angle',a
			   print 'ToolVector',tool
			   print 'Normal',n
			   print
		  dy=n*cg.vec3(0,1,0) #roll axis
		  dz=n*cg.vec3(0,0,1) #pitch axis
		  delta_pitch=dz*a
		  delta_roll=-1*dy*a #joint backwards
		  return delta_roll,delta_pitch
		  
	 
	 def tool_gravity_align(self,tool,palm=[0,0,0],debug=False):
		  #X is in direction of cylinder grasp axis, align with gravity
		  return self.tool_vector_align(tool,[0,0,1],palm,debug)


         # #######################################################################
         """
         A Wrench can be at the tool tip or the end-effector (hand)
         It can be specified in the hands coordinate system (hand wrench, tool wrench) or the world coord. system
         """

         def tool2worldjacobian(self,tip):
                  #Compute Jt for fw=Jt*ft, for a force a tip, where tip,ft are given in hand coords 
                  hJt=self.tool2handJacobian(tip)
                  wJh=self.hand2worldJacobian()
                  Jt=nu.matrixmultiply(wJh,hJt) #this takes wrench at tip and creates world wrench at hand
                  return Jt
                  
         def hand2worldJacobian(self):
                  #Compute Jt for fw=Jt*fh 
                  R=nu.array(self.wRh[self.arm_name].getMat3(),nu.Float32)
                  t=nu.concatenate([R,nu.zeros([3,3],nu.Float32)],1)
                  tt=nu.concatenate([nu.zeros([3,3],nu.Float32),R],1)
                  Jt=nu.concatenate([t,tt],0) 
                  return Jt
         
         def tool2handJacobian(self,pt):
                  #Compute Jt for fh=Jt*ft , for a force a pt in hand coord frame
                  R=nu.identity(3,nu.Float32) #rotate from tool frame to hand frame
                  Px=nu.array([[0,-pt[2],pt[1]],[pt[2],0,-pt[0]],[-pt[1],pt[0],0]],nu.Float32) #cross product
                  t=nu.concatenate([R,nu.zeros([3,3],nu.Float32)],1)
                  tt=nu.concatenate([Px,R],1)
                  Jt=nu.concatenate([t,tt],0) #converts force at tip to force at hand
                  return Jt
                  
         def hand_wrench2joint_torques(self,fh):
                  Jt=self.hand2worldJacobian()
                  fw=nu.matrixmultiply(Jt,nu.array(fh,nu.Float))
                  return self.world_wrench2joint_torques(fw)
         
         def tool_wrench_world2joint_torques(self,tip,ftw):
                  Jt=self.tool2handJacobian(tip)
                  fhw=nu.matrixmultiply(Jt,nu.array(ftw,nu.Float))
                  return self.world_wrench2joint_torques(fhw)
         
         def tool_wrench2joint_torques(self,tip,ft):
                  Jt=self.tool2worldjacobian(tip)
                  fw=nu.matrixmultiply(Jt,nu.array(ft,nu.Float))
                  return self.world_wrench2joint_torques(fw)
         
         def world_wrench2joint_torques(self,fw):
                  # t=Jt*fw, where fw is a wrench at the hand in world coords
                  JT=nu.transpose(self.J[self.arm_name])
                  return nu.matrixmultiply(JT,nu.array(fw))
       
         # #######################################################################
         

                
         # NOTE: display functions may not be working currently...
         
         def display_epipolar_poly(self,im_rgb,dest_eye,src_eye,px_src,start_z,end_z,width):
		  if False: #deprecated
			   p=self.epipolar_poly(dest_eye,src_eye,px_src,start_z,end_z,width)
			   if p is not None:
				    poly=ic.list2poly(p)
				    color=si.RGBBytePixel()
				    color.Set(255,255,255)
				    poly.DrawOnImage(im_rgb,color,False)
		  
         def display_finger(self,display,im,where,finger,r=10):
                  ph=self.hand_center2pixel()
                  p2=self.world2pixel(self.hkine.joint2world(finger,2))
                  p3=self.world2pixel(self.hkine.joint2world(finger,3))
                  p4=self.world2pixel(self.hkine.joint2world(finger,4))
                  display.draw_line(ph,p2,color=(0,0,1),where=where,im=im)
                  display.draw_line(p2,p3,color=(0,0,1),where=where,im=im)
                  display.draw_line(p3,p4,color=(0,0,1),where=where,im=im)
                  #display.draw_circle(p4,radius=r,z=0.01,color=(0, 0.0, 1.0),where=where,im=im)
                  
                  
         def display_wrist_frame(self,display,im,where=(0,0),
                                 tool_point=None, points=None):
                  """draw the wrist frame on the image"""
                  x=cg.vec3(10,0,0)
                  y=cg.vec3(0,10,0)
                  z=cg.vec3(0,0,10)

                  pO=self.hand_center2pixel()
                  pX=self.hand2pixel(x)
                  pY=self.hand2pixel(y)
                  pZ=self.hand2pixel(z)
                  
                  if tool_point is not None:
                           print 'Toolpoint',tool_point
                           pT=self.hand2pixel(tool_point)
                           display.draw_line(pO,pT,color=(1,1,1),where=where,im=im)
                           display.draw_circle(pT,radius=10,z=0.01,where=where,color=(1, 1.0, 0.0),im=im)
                           
                  if points is not None:
                      colors = display.r_pal
                      for i, p in enumerate(points):
                          pT=self.hand2pixel(p)
                          color = colors[i % len(colors)]
                          display.draw_line(pO,pT,color=color,where=where,im=im)
                          
                  display.draw_line(pO,pX,color=(1,0,0),where=where,im=im)
                  display.draw_line(pO,pY,color=(0,1,0),where=where,im=im)
                  display.draw_line(pO,pZ,color=(0,0,1),where=where,im=im)
                  display.draw_circle(pO,radius=10,z=0.01,color=(0, 1.0, 0.0),where=where,im=im)
                  
         def display_wrist_cube(self,display,im,where=(0,0)):
                  x_axis = cg.vec3(1.0, 0.0, 0.0)
                  Rgl = cg.mat4().rotation(-math.pi, x_axis) #rotate from image 3D to GL 3D
                  cRh = self.cRw[self.eye_name]*self.wRh[self.arm_name]
                  Rot = Rgl * cRh #camera coords to open gl coords
                  ch = Rot*cg.vec4(0,0,0,1) / self.f[self.eye_name]
                  Rot[0,3]=0
                  Rot[1,3]=0
                  Rot[2,3]=0
                  display.draw_cube([0,0,0], Rot=Rot, size=1.0,where=where, color=(1.0,1.0,1.0)) #[ch[0],ch[1],ch[2]]
                  
         def display_eye_base_cube(self,display,im,where=(0,0)):
                  Rgl =  cg.mat4().rotation(-math.pi/2, cg.vec3(1.0, 0.0, 0.0)) * cg.mat4().rotation(math.pi, cg.vec3(0.0, 0.0, 1.0)) #rotate from image 3D to GL 3D
                  Rot = Rgl * self.wRs  #eyebase coords to open gl coords
                  Rot[0,3]=0
                  Rot[1,3]=0
                  Rot[2,3]=0
                  display.draw_cube([0,0,0], Rot=Rot, size=1.0,where=where, color=(1.0,1.0,1.0)) 
                  display.draw_axes(Rot=Rot, where=where)
                  
         def display_head_cube(self,display,im,where=(0,0)):
                  x_axis = cg.vec3(1.0, 0.0, 0.0)
                  Rgl = cg.mat4().rotation(-math.pi, x_axis) #rotate from image 3D to GL 3D
                  Rot = Rgl * self.cRw #camera coords to open gl coords
                  Rot[0,3]=0
                  Rot[1,3]=0
                  Rot[2,3]=0
                  display.draw_cube([0,0,0], Rot=Rot, size=1.0,where=where, color=(1.0,1.0,1.0)) #[ch[0],ch[1],ch[2]]
                  display.draw_axes(Rot=Rot, where=where)
        
         def display_gyro_cube(self,display,im,where=(0,0)):
                  x_axis = cg.vec3(1.0, 0.0, 0.0)
                  Rgl = cg.mat4().rotation(-math.pi, x_axis) #rotate from image 3D to GL 3D
                  Rot = Rgl * self.wRg #camera coords to open gl coords
                  Rot[0,3]=0
                  Rot[1,3]=0
                  Rot[2,3]=0
                  #display.draw_cube([0,0,0], Rot=Rot, size=1.0,where=where, color=(1.0,1.0,1.0)) #[ch[0],ch[1],ch[2]]
                  display.draw_axes(Rot=Rot, where=where)
                
         
         # ######### Private   ########
         """Inputs are cg.vec4,cg.mat4, returns cg.vec4"""

         
         def image2world(self,i,r=100.0):
                  return self.camera2world(self.image2camera(i,r))
         
         def image2camera(self,i,r=100.0):
                  a=cg.vec3(i[0],i[1],self.f[self.eye_name])
                  a=a.normalize() * r
                  return cg.vec4(list(a)+[1.0])
         
         def camera2world(self,x):
                  r= self.wRc[self.eye_name]*x
                  return r
         
         def world2image(self,x):
                  return self.camera2image(self.world2camera(x))
         
         def world2camera(self,x): 
                  return self.cRw[self.eye_name]*x

         
         def camera2image(self,c):
                  if False:
                           if c[2]<1.0:
                                    pass #print 'Potentially bad value in camera2image...',c
                  z=max(1.0,c[2]) #keep from blowing up, assume always on correct side of image plane (if not, or near, to far away anyhow)
                  xi = self.f[self.eye_name]*c[0]/z 
                  yi = self.f[self.eye_name]*c[1]/z
                  return [xi,yi] 
                  
         def pixel_to_image_frame(self,p):
                  return [p[0]-self.w/2,p[1]-self.h/2]
         
         def image_frame_to_pixel(self,i):
                  return [i[0]+self.w/2,i[1]+self.h/2]


                  
         # #################
         
         def load_prio(self,prio):
                  self.load_prio_head(prio)
                  self.load_prio_arms(prio)
		  if False: #works but not using so optimize
			  self.load_prio_hands(prio)
                  
         def load_prio_hands(self,prio):
		  for h in ['left_hand','right_hand']:
			   self.theta[h]=list(prio[h]['theta'])
                  self.hkine.load_prio(prio)
                  
         def load_calibration(self):
                  #Call after instance creation if required
                  import sal_joint_defs as sj
                  for e in ['left_eye','right_eye']:
                           self.eRa[e]= sj.camera_calibration[e]['eRa']
                           
         def load_prio_arms(self,prio):
                  self.load_prio_arm('left_arm',prio)
                  self.load_prio_arm('right_arm',prio)
           
         def load_wrist_adjust(self,xf,prio):
                  #translate the wrist joint by a small amount
                  wRk=cg.mat4(prio[self.arm_name]['jointorient'][jWristRoll*16:jWristRoll*16+16])
                  kRw=wRk.inverse()
                  kRh=kRw*self.wRh[self.arm_name]
		  xw=self.wRf[self.arm_name]*cg.vec4(list(xf)+[1])
                  wRk.setColumn(3,xw)
                  self.wRh[self.arm_name]=wRk*kRh
                  self.hRw[self.arm_name]=self.wRh[self.arm_name].inverse()
                  
         def load_prio_arm(self,a,prio):
                  try:
                           self.wRh_prev[a]=self.wRh[a]
                           self.hRw_prev[a]=self.hRw[a]
                           self.wRh[a] = cg.mat4(prio[a]['jointorient'][jWristPitch*16:jWristPitch*16+16])
                           self.wRj[a] = cg.mat4(prio[a]['jointorient'][jElbow*16:jElbow*16+16])
                           self.hRw[a] = self.wRh[a].inverse()
                           self.jRw[a] = self.wRj[a].inverse()
                           self.J[a] =   nu.array(prio[a]['jacobian'])
                           self.J[a]=self.J[a].resize([6,6])
                           wR3=cg.mat4(prio[a]['jointorient'][3*16:3*16+16])
                           wR4=cg.mat4(prio[a]['jointorient'][4*16:4*16+16])
                           wR3.setColumn(3,wR4.getColumn(3)) #translate from elbow to wrist
                           self.wRf[a]=wR3
                           self.fRw[a]=self.wRf[a].inverse()
                           self.theta[a]=list(prio[a]['theta'])
			   self.jointpos[a]=na.array(prio[a]['jointpos'])
                  except ZeroDivisionError,TypeError:
                           print 'Bad prio in viz_kine',a #May happen at startup
                           self.wRh[a] = cg.mat4(1)
                           self.hRw[a] = cg.mat4(1)
                           self.wRj[a] = cg.mat4(1)
                           self.jRw[a] = cg.mat4(1)
                           self.wRf[a] = cg.mat4(1)    
                           self.fRw[a] = cg.mat4(1)    
                           self.theta[a]=[0.0]*6
			   self.jointpos[a]=na.array([0.0]*6*3)

         def load_prio_head(self,prio):
                  try:
                           for e in ['left_eye','right_eye']:
                                    self.wRc_prev[e]=self.wRc[e]
                                    self.cRw_prev[e]=self.cRw[e]
                                    wRe,wRc,cRw=self.compute_wRc(e,prio)
                                    self.wRe[e] = wRe
                                    self.wRc[e]=  wRc 
                                    self.cRw[e]=  cRw
			   self.jointpos['head']=na.array(prio['head']['jointpos'])
                           self.wRs=  cg.mat4(prio['head']['jointorient'][jHeadRoll*16:jHeadRoll*16+16]) #eye_base
                           self.wRt=cg.mat4(prio['head']['jointorient'][jEyeTilt*16:jEyeTilt*16+16]) #eyetilt
                           #Transform between eyes
                           wRcr=self.wRc['right_eye']
                           wRcl=self.wRc['left_eye']
                           clRw=wRcl.inverse()
                           crRw=wRcr.inverse()
                           self.camRcam['right_eye']=clRw*wRcr #right camera to left camera
                           self.camRcam['left_eye']=crRw*wRcl #left camera to right camera
                           self.theta['head']=list(prio['head']['theta'])
                           if self.stereo_baseline is None:
                                    l=cg.vec3(list(self.wRc['right_eye'].getColumn(3))[0:3])
                                    r=cg.vec3(list(self.wRc['left_eye'].getColumn(3))[0:3])
                                    self.stereo_baseline=(l-r).length()
                           
                           self.gyro_ang=list(prio['head']['gyro'])
                           Rx = cg.mat4().rotation(math.pi*self.gyro_ang[2]/180.0, self.x_axis) #roll
                           Ry = cg.mat4().rotation(math.pi*self.gyro_ang[1]/180.0, self.y_axis) #pitch
                           Rz = cg.mat4().rotation(math.pi*self.gyro_ang[0]/180.0, self.z_axis) #yaw
                           self.geoRg = Rz * Ry * Rx

                  except ZeroDivisionError,TypeError:
                           print 'Bad prio in viz_kine','head' #May happen at startup
                           self.wRt = cg.mat4(1)
                           self.wRs=  cg.mat4(1)
                           self.theta['head']=[0.0]*8
                  
                           for e in ['left_eye','right_eye']:
                                    self.wRe[e] = cg.mat4(1)
                                    self.wRc[e]=  cg.mat4(1)
                                    self.cRw[e]=  cg.mat4(1)	
                                    self.camRcam[e]=cg.mat4(1)
        
         
         def compute_wRc(self,e,prio):
                   if e=='right_eye':
                                      jidx=jEyePanRight
                   if e=='left_eye':
                                      jidx=jEyePanLeft
                   wRe= cg.mat4(prio['head']['jointorient'][jidx*16:jidx*16+16])
                   wRe= wRe*self.eRa[e] #calibration adjustment of camera orientation
                   wRc= wRe*self.eRc[e] 
                   cRw=  wRc.inverse() 
                   return wRe,wRc,cRw
         

                                    
         def get_wRc_gyro_adjusted(self):
                  sRw=self.wRs.inverse()
                  wRc={}
                  cRw={}
                  sRc={}
                  sRw=self.wRs.inverse()
                  for e in ['right_eye','left_eye']:
                           sRc[e]=sRw*self.wRc[e]
                  try:
                           gg = cg.vec3(list(self.geoRg.transpose()*cg.vec4(0,0,-1,0))[0:3]) #gravity in gyro frame
                           gs = cg.vec3(list(sRw*cg.vec4(0,0,1,0))[0:3]) #gravity in skull frame
                           a=gg.angle(gs)
                           #print 'Ang',180.0*a/math.pi
                           n=gs.cross(gg).normalize()
                           q=cg.quat().fromAngleAxis(a,n)
                           Radj=q.toMat4() #rotate skull such that gravities align
                           wRs=(Radj*sRw).inverse()
                           for e in ['right_eye','left_eye']: #write over the relevant frames
                                    wRc[e]=wRs*sRc[e]
                                    cRw[e]=wRc[e].inverse()
                           return {'wRc':wRc,'cRw':cRw}
                  except ZeroDivisionError: 
                                    print 'ZeroDivisionError: get_wRc_gyro_adjusted'
                                    return {'wRc':self.wRc,'cRw':self.cRw} 
                 

                          
       
         def pixel_intersect_world_plane(self,pixel,n,pt): #where does pixel ray intersect the plane defined by normal and point on plane, in world coords
                  return self.ray_intersect_plane([cg.vec3(self.pixel2world(pixel,0)),cg.vec3(self.pixel2world(pixel,100))],cg.vec3(n),cg.vec3(pt))

         def pixel_intersect_camera_plane(self,pixel,z_camera):
		  #location in camera coords at given z-depth
		  n=cg.vec3(0,0,1)
		  pt=cg.vec3(0,0,z_camera)
		  ray=[cg.vec3(self.pixel2camera(pixel,0)),cg.vec3(self.pixel2camera(pixel,100))]
		  return self.ray_intersect_plane(ray,n,pt)
		  
	 def ray_intersect_plane(self,ray,n,pt):#where does ray intersect the plane defined by normal and point on plane,
		  p1=ray[0]
                  p2=ray[1]
                  u=(n*(pt-p1))/(n*(p2-p1))
                  p=p1+u*(p2-p1)
		  return p
	 
         def min_point_ray(self,ray,pt):
                  #Find the point on ray that is closest to pt
                  u=ray[1]-ray[0]
                  v= pt-ray[0]
                  d = ray[0]+((u*v)/(u*u))*u
                  return d
         
         def min_point_line(self,ray,pt):
		  #Find the point on line ,within endpoints,  that is closest to pt
		  pt=self.min_point_ray(ray,pt)
		  e=pt-ray[0]
		  d=ray[1]-ray[0]
		  m=d.length()
		  dot=(e/m)*(d/m)
		  if dot<0:
			   return ray[0]
		  if dot>1:
			   return ray[1]
		  return pt
	 
         def min_point_rays(self,ray1, ray2):
                  # Find the minimal point between the two rays
                  # this assumes that each ray is represented by two points ([a,b])
                  # in a list, the first of which (a) is at the start of the ray
                  # the second of which (b) sits on the ray. (a != b)
                  a1, b1 = ray1
                  a2, b2 = ray2
                  a1 = na.array(a1)
                  b1 = na.array(b1)
                  a2 = na.array(a2)
                  b2 = na.array(b2)

                  A = na.array([a1-b1, b2-a2])
                  A.transpose()
                  b = b2 - b1
                  b.transpose()

                  # what if the rays are the same??? (this needs to be tested)
                  alpha, fit_error, rank, singular_values = nal.linear_least_squares(A, b)

                  p1 = ((a1 - b1) * alpha[0]) + b1
                  p2 = ((a2 - b2) * alpha[1]) + b2

                  if rank < 2:
                           return None, None
                  if na.dot(p1 - a1, b1 - a1) < 0.0:
                           # the minimum point is behind the start of the ray
                           return None, None
                  if na.dot(p2 - a2, b2 - a2) < 0.0:
                           # the minimum point is behind the start of the ray
                           return None, None

                  distance_between_rays = na.sqrt(((p2 - p1)**2.0).sum())

                  return list((p1 + p2)/2.0), distance_between_rays
         

         def intersection_ray_sphere(self,p0,d,pc,r):
                  #http://www.csee.umbc.edu/~olano/435f02/ray-sphere.html
                  # p0: start of ray
                  # d: direction of ray
                  # pc: center of sphere
                  # r: radius of sphere
                  p0=cg.vec3(p0)
                  d=cg.vec3(d)
                  pc=cg.vec3(pc)
                  
                  a=d*d
                  b = 2*d*(p0-pc)
                  c = ((p0 - pc) *(p0 - pc)) - r*r
                  discr=(b**2)-(4*a*c)
                  if discr<=0:
                           print 'No intersection'
                           return None
                  
                  if discr>0: #two intersections
                           ta=(-b+math.sqrt(discr))/(2*a)
                           tb=(-b-math.sqrt(discr))/(2*a)
                           pa=ta*d+p0
                           pb=tb*d+p0
                           return pa,pb
                  
                  
class KinematicsHand:
         def __init__(self):
                  #Fingers identified as A,B,C
                  #A is thumb
                  #B is across from thumb
                #C is below B
                #H is hand frame at end of arm kinematic chain
                #For each finger:
                #0 is base frame offset from end of arm and z-aligned with finger roll
                #1 is finger roll frame
                #2 is first joint of finger which has pot measured
                #3 is second joint, fixed ratio to first
                #4 is fingertip, fixed offset from second (z is rotation axis, y is along length of finger)
                
                self.hand='right_hand'
                self.arms={'right_hand':'right_arm','left_hand':'left_arm'}
                self.wRh = {}
                for a in ['left_arm','right_arm']:
                           self.wRh[a] = cg.mat4(1)
                self.frames={}
                self.z_axis = cg.vec3(0.0, 0.0, 1.0)	
                
         def set_hand(self,hand):
                self.hand=hand
                
         def fingertip2world(self,finger,xf=None):
                  if xf is None:
                           xf=[0,0,0]
                  frame=self.frame2world(self.hand,finger,4)
                  xw=frame*cg.vec4(list(xf)+[1.0])
                  return list(xw)[0:3]
                  
         def joint2world(self,finger,j):
                  frame=self.frame2world(self.hand,finger,j)
                  xw=frame*cg.vec4([0,0,0,1])
                  return list(xw)[0:3]
         # ############################################
        
         def frame2world(self,hand,finger,joint):
                  return self.wRh[hand]*self.frames[hand][finger][joint]
        
         def load_prio(self,prio):
                  for a in ['left_hand','right_hand']:
                          try:
                                  self.wRh[a] = cg.mat4(prio[self.arms[a]]['jointorient'][5*16:5*16+16])
                          except ZeroDivisionError:
                                  self.wRh[a]=cg.mat4(1)
                        
                  mx=0.5 #motor offset is 0.5inches exactly
                  pm=1.375 #palm offset
                  df=0.8 #offset from roll to first finger joint
                  l0=1.5 #link 0
                  l1=2.0 #link1, including rubber finger tip
                  t0=90.0 #theta offset between l0,l1
                  ratio_c = 0.65 #coupling ratio
                  
                  hand='right_hand'
                  self.frames[hand]={}
                  
                  # ################# Right Hand A ###########
                  finger='A'
                  self.frames[hand][finger]={}
                  #base
                  self.frames[hand][finger][0]=cg.mat4([1,0,0,-mx]+[0,0,1,pm]+[0,-1,0,mx]+[0,0,0,1])
                  #thumb roll
                  theta=-1*prio[hand]['theta'][3]
                  Rz = cg.mat4().rotation(math.pi*theta/180.0, self.z_axis) 
                  self.frames[hand][finger][1]=self.frames[hand][finger][0]*Rz 
                  #joint0
                  theta=prio[hand]['theta'][0]
                  Rz = cg.mat4().rotation(math.pi*theta/180.0, self.z_axis) 
                  self.frames[hand][finger][2]=self.frames[hand][finger][1]*cg.mat4([0,0,-1,0,   0,1,0,-df,   1,0,0,0,   0,0,0,1])*Rz
                  #joint1
                  Rz = cg.mat4().rotation(((ratio_c*theta)+t0)*math.pi/180.0, self.z_axis) 
                  self.frames[hand][finger][3]=self.frames[hand][finger][2]*cg.mat4([1,0,0,0,   0,1,0,-l0,   0,0,1,0,   0,0,0,1])*Rz
                  #joint2 (fingertip)
                  self.frames[hand][finger][4]=self.frames[hand][finger][3]*cg.mat4([1,0,0,0,   0,1,0,-l1,   0,0,1,0,   0,0,0,1])
                  
                  # ################# Right Hand B ###########
                  finger='B'
                  self.frames[hand][finger]={}
                  #base
                  self.frames[hand][finger][0]=cg.mat4([1,0,0,-mx]+[0,0,1,pm]+[0,-1,0,-mx]+[0,0,0,1])
                  #thumb roll
                  theta=180
                  Rz = cg.mat4().rotation(math.pi*theta/180.0, self.z_axis) 
                  self.frames[hand][finger][1]=self.frames[hand][finger][0]*Rz 
                  #joint0
                  theta=prio[hand]['theta'][1]
                  Rz = cg.mat4().rotation(math.pi*theta/180.0, self.z_axis) 
                  self.frames[hand][finger][2]=self.frames[hand][finger][1]*cg.mat4([0,0,-1,0,   0,1,0,-df,   1,0,0,0,   0,0,0,1])*Rz
                  #joint1
                  Rz = cg.mat4().rotation(((ratio_c*theta)+t0)*math.pi/180.0, self.z_axis) 
                  self.frames[hand][finger][3]=self.frames[hand][finger][2]*cg.mat4([1,0,0,0,   0,1,0,-l0,   0,0,1,0,   0,0,0,1])*Rz
                  #joint2 (fingertip)
                  self.frames[hand][finger][4]=self.frames[hand][finger][3]*cg.mat4([1,0,0,0,   0,1,0,-l1,   0,0,1,0,   0,0,0,1])
                  
                  # ################# Right Hand C ###########
                  finger='C'
                  self.frames[hand][finger]={}
                  #base
                  self.frames[hand][finger][0]=cg.mat4([1,0,0,mx]+[0,0,1,pm]+[0,-1,0,-mx]+[0,0,0,1])
                  #thumb roll
                  theta=180
                  Rz = cg.mat4().rotation(math.pi*theta/180.0, self.z_axis) 
                  self.frames[hand][finger][1]=self.frames[hand][finger][0]*Rz 
                  #joint0
                  theta=prio[hand]['theta'][2]
                  Rz = cg.mat4().rotation(math.pi*theta/180.0, self.z_axis) 
                  self.frames[hand][finger][2]=self.frames[hand][finger][1]*cg.mat4([0,0,-1,0,   0,1,0,-df,   1,0,0,0,   0,0,0,1])*Rz
                  #joint1
                  Rz = cg.mat4().rotation(((ratio_c*theta)+t0)*math.pi/180.0, self.z_axis) 
                  self.frames[hand][finger][3]=self.frames[hand][finger][2]*cg.mat4([1,0,0,0,   0,1,0,-l0,   0,0,1,0,   0,0,0,1])*Rz
                  #joint2 (fingertip)
                  self.frames[hand][finger][4]=self.frames[hand][finger][3]*cg.mat4([1,0,0,0,   0,1,0,-l1,   0,0,1,0,   0,0,0,1])
                  
                  hand='left_hand'
                  self.frames[hand]={}
                  
                  # ################# Left Hand A ###########
                  finger='A'
                  self.frames[hand][finger]={}
                  #base
                  self.frames[hand][finger][0]=cg.mat4([1,0,0,-mx]+[0,0,1,pm]+[0,-1,0,-mx]+[0,0,0,1])
                  #thumb roll
                  theta=180.0+prio[hand]['theta'][3]
                  Rz = cg.mat4().rotation(math.pi*theta/180.0, self.z_axis) 
                  self.frames[hand][finger][1]=self.frames[hand][finger][0]*Rz 
                  #joint0
                  theta=prio[hand]['theta'][0]
                  Rz = cg.mat4().rotation(math.pi*theta/180.0, self.z_axis) 
                  self.frames[hand][finger][2]=self.frames[hand][finger][1]*cg.mat4([0,0,-1,0,   0,1,0,-df,   1,0,0,0,   0,0,0,1])*Rz
                  #joint1
                  Rz = cg.mat4().rotation(((ratio_c*theta)+t0)*math.pi/180.0, self.z_axis) 
                  self.frames[hand][finger][3]=self.frames[hand][finger][2]*cg.mat4([1,0,0,0,   0,1,0,-l0,   0,0,1,0,   0,0,0,1])*Rz
                  #joint2 (fingertip)
                  self.frames[hand][finger][4]=self.frames[hand][finger][3]*cg.mat4([1,0,0,0,   0,1,0,-l1,   0,0,1,0,   0,0,0,1])
                  
                  # ################# Left Hand B ###########
                  finger='B'
                  self.frames[hand][finger]={}
                  #base
                  self.frames[hand][finger][0]=cg.mat4([1,0,0,-mx]+[0,0,1,pm]+[0,-1,0,mx]+[0,0,0,1])
                  #thumb roll
                  theta=0
                  Rz = cg.mat4().rotation(math.pi*theta/180.0, self.z_axis) 
                  self.frames[hand][finger][1]=self.frames[hand][finger][0]*Rz 
                  #joint0
                  theta=prio[hand]['theta'][1]
                  Rz = cg.mat4().rotation(math.pi*theta/180.0, self.z_axis) 
                  self.frames[hand][finger][2]=self.frames[hand][finger][1]*cg.mat4([0,0,-1,0,   0,1,0,-df,   1,0,0,0,   0,0,0,1])*Rz
                  #joint1
                  Rz = cg.mat4().rotation(((ratio_c*theta)+t0)*math.pi/180.0, self.z_axis) 
                  self.frames[hand][finger][3]=self.frames[hand][finger][2]*cg.mat4([1,0,0,0,   0,1,0,-l0,   0,0,1,0,   0,0,0,1])*Rz
                  #joint2 (fingertip)
                  self.frames[hand][finger][4]=self.frames[hand][finger][3]*cg.mat4([1,0,0,0,   0,1,0,-l1,   0,0,1,0,   0,0,0,1])
                  
                  # ################# Left Hand C ###########
                  finger='C'
                  self.frames[hand][finger]={}
                  #base
                  self.frames[hand][finger][0]=cg.mat4([1,0,0,mx]+[0,0,1,pm]+[0,-1,0,mx]+[0,0,0,1])
                  #thumb roll
                  theta=0
                  Rz = cg.mat4().rotation(math.pi*theta/180.0, self.z_axis) 
                  self.frames[hand][finger][1]=self.frames[hand][finger][0]*Rz 
                  #joint0
                  theta=prio[hand]['theta'][2]
                  Rz = cg.mat4().rotation(math.pi*theta/180.0, self.z_axis) 
                  self.frames[hand][finger][2]=self.frames[hand][finger][1]*cg.mat4([0,0,-1,0,   0,1,0,-df,   1,0,0,0,   0,0,0,1])*Rz
                  #joint1
                  Rz = cg.mat4().rotation(((ratio_c*theta)+t0)*math.pi/180.0, self.z_axis) 
                  self.frames[hand][finger][3]=self.frames[hand][finger][2]*cg.mat4([1,0,0,0,   0,1,0,-l0,   0,0,1,0,   0,0,0,1])*Rz
                  #joint2 (fingertip)
                  self.frames[hand][finger][4]=self.frames[hand][finger][3]*cg.mat4([1,0,0,0,   0,1,0,-l1,   0,0,1,0,   0,0,0,1])
                           
         
         
         
         