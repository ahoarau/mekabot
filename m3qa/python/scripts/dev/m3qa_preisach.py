#! /usr/bin/python
import time
import m3.toolbox as m3t
import Numeric as nu
import math
import yaml
import m3.rt_proxy as m3p
import m3.toolbox as m3t
import m3.component_factory as mcf


#Image stuff
from PIL import Image
from PIL import ImageDraw
import sys
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import os, sys
import Tkinter
import Image, ImageTk
import random
from m3qa.calibrate_actuator_ec_r3 import dpm3_thread


# ####################################################
#L+1: number of levels in grid dimension
L=50
#drawing width
dw=10
# ####################################################
import pygame
import Image
from pygame.locals import *
import sys
pygame.init()
window = pygame.display.set_mode((L*dw,L*dw))
screen = pygame.display.get_surface()
events = pygame.event.get()
im = Image.new("RGB", (L*dw,L*dw), (0,0,0))
draw = ImageDraw.Draw(im)
def draw_plane(u):
    for i in range(L,0,-1):
        for j in range(1,i+1):
	    draw_cell(i,j,u)
    pg_img = pygame.image.frombuffer(im.tostring(), im.size, im.mode)
    screen.blit(pg_img, (0,0))
    pygame.display.flip()

def in_cell(i,j,u):
    k=cell_to_idx(i,j)
    return(u>a[k]-cw and u<a[k]) and (u>b[k] and u<b[k]+cw)
def draw_cell(i,j,u):
    cy=L*dw-i*dw-dw/2.0
    cx=j*dw-dw/2.0
    k=cell_to_idx(i,j)
    if in_cell(i,j,u):
	draw.rectangle(((cx-dw/2.0)+1,(cy-dw/2.0)+1,(cx+dw/2.0)-1,(cy+dw/2.0)-1),fill=(0,0,255),outline=(255,255,0))
    elif g[k]>0:
	draw.rectangle(((cx-dw/2.0)+1,(cy-dw/2.0)+1,(cx+dw/2.0)-1,(cy+dw/2.0)-1),fill=(0,0,0),outline=(255,255,0))
    else:
	draw.rectangle((cx-dw/2.0,cy-dw/2.0,cx+dw/2.0,cy+dw/2.0),fill=(0,0,0),outline=(255,0,0))
	
# ####################################################

#u : input variable
#v: output variable
#a>b by definition
#mu(b,a)=0 if b<bo or a>ao (outer bounds of Preisach plane)
#w(t): Preisach operator (omega): integral over plane = sum(g(a,b)==1)-sum(g(a,b)==-1)


#K: number of elements in column vector representation
K=int(L*(L+1)/2.0)

#a: upper bound (alpha)
a=nu.array([0.0]*K)
#b: lower bound (beta)
b=nu.array([0.0]*K)
#mu: density function mu(b,a)
mu=nu.array([0.0]*K)

#g: Preisach hysteron (gamma): v=g_op(b,a,u)=+1 if u>a, -1 if u<b, g if b<=u<=a
#Start in full negative state: u(t<0)<b0
g=nu.array([-1.0]*K)

#a0: maximum input value
#a0=9
#b0: minimum input value
#b0=1

u_max=10000
u_min=-10000

#Cell width
cw=(u_max-u_min)/L
print 'Cell width is',cw,'mNm'

#un: input sequence of n discretized inputs
#n=?

def build():
    for i in range(L,0,-1):
        for j in range(1,i+1):
            k=cell_to_idx(i,j)
            alpha=u_min+i*cw
            beta=u_min+(j-1)*cw
            a[k]=alpha
            b[k]=beta
            #print 'i',i,'j',j,'a',alpha,'b',beta
    
def discretize(u):
    if u<=u_min:
        return 1
    if u>=u_max:
        return L
    return math.ceil((u-u_min)/cw)
    
def print_plane():
    for i in range(L,0,-1):
        for j in range(1,i+1):
            v=g[cell_to_idx(i,j)]
            k=cell_to_idx(i,j)
            print '(',v,')',
            #print '(',b[k],a[k],')',
            #print '(',i,',',j,')', #row, col
        print

def cell_to_idx(i,j):
    return int(i*(i+1)/2-(i-j+1))

#Hysteron function: input, beta, alpha, and current
#Made inclusive of boundaries. Does'nt follow convention
#But otherwise plane borders are ignored. What is the correct way to handle this?
def g_op(u,bb,aa,cc):
    if u>u_max or u<u_min:
	return 0.0
    if u>=aa:
        return 1.0
    if u<=bb:
        return -1.0
    return cc


def step_input(u):
    for k in range(K):
	y=g_op(u,b[k],a[k],g[k])
        g[k]=y
   
slew=m3t.M3Slew()
def ramp_to_torque(des,scope):
    act.set_mode_torque()
    if slew.val==0:
	slew.val=act.get_torque_mNm()
    rate=500.0#100mNm/step, ~30steps/s, 3Nm/s, ~7s full range
    x=slew.step(des,rate) 
    ut=[]
    wt=[]
    gt=[]
    while True:
	act.set_torque_mNm(x)
	proxy.step()
	time.sleep(0.03)
	ut.append(act.get_torque_mNm())
	wt.append(dpm3.get_load_mNm())
	step_input(ut[-1])
	draw_plane(u)
	#gt.append(list(g))
	scope.plot(ut[-1],x)
	x=slew.step(des,rate)
	if x==des: #Wait to settle 
	    for i in range(20):
		proxy.step()
		time.sleep(0.1)
		x=slew.step(des,rate)
		ut.append(act.get_torque_mNm())
		wt.append(dpm3.get_load_mNm())
		step_input(ut[-1])
		draw_plane(u)
		#gt.append(list(g))
		scope.plot(ut[-1],x)
	    break
	#print x,des,ut[-1]
    #act.set_mode_off()
    proxy.step()
    return ut,wt,gt

def triangle_wave():
    scope=m3t.M3Scope2(xwidth=100,yrange=None)
    des=[]
    cc=1.0
    scale=5000
    for i in range(10):
	des.append(cc)
	cc=cc*-1.0
    print 'Enable power. Hit enter to continue'
    raw_input()
    for ii in range(len(des)):
	print ii,'Des: ',des[ii]
	ramp_to_torque(des[ii]*scale,scope)

def fill_cells_random():
    scope=m3t.M3Scope2(xwidth=100,yrange=None)
    ns=30#5#K*2
    #Build desired vector
    log={'ns':ns,'type':'fill_cells_random',
         'actuator':'MA12J1','L':L,'K':K,'CW':cw,'a':list(a),'b':list(b),'u_max':u_max,'u_min':u_min,'ut':[],'wt':[],'gt':[],'des':[]}
    des=[u_min]
    for nn in range(ns):
	while True:
	    x=u_min+(u_max-u_min)*(random.random()) #randomly in range
	    if abs(des[-1]-x)>5*cw: #make excursion at least 5 cells
		des.append(x)
		break
    log['des']=des
    print 'Des',des
    print 'Enable power. Hit enter to continue'
    raw_input()
    for ii in range(len(des)):
	print ii,'Des: ',des[ii]
	uu,ww,gg=ramp_to_torque(des[ii],scope)
	print len(uu),len(ww),len(gg)
	log['ut'].append(list(uu)) #input
	log['wt'].append(list(ww)) #output
	log['gt'].append(list(gg)) #hysteron states
    print 'Save data [y]?'
    if m3t.get_yes_no('y'):
	fn=m3t.get_m3_config_path()+'data/preisach_data_random_ma12j1_'+m3t.time_string()+'.yml'
	print 'Enter annotation string [None]'
	note=m3t.get_string('None')
	log['note']=note
	f=file(fn,'w')
	print 'Saving...',fn
	f.write(yaml.safe_dump(log, default_flow_style=False,width=200))
	f.close()
    return log

def fill_cells_methodical():
    #fill by row, for each row-alpha, starting at u_max
    #reduce torque to u_min, then increase to row-alpha
    #then reduce to column-beta, from alpha down to u_min
    #this will fill an entire row. reduce the row index and repeat
    for i in range(L,0,-1): #row
	ramp_to_torque(u_min)
	alpha=a[cell_to_idx[i,0]]
	for j in range(L,0,-1): 
	    beta=b[cell_to_idx[i,j]]
	    des=beta+cw/2.0 #servo to center of cell
	    ramp_to_torque(des)
	    u=act.get_torque_mNm()
	    w=dpm3.get_load_mNm()
	    ut.append(u)
	    wt.append(w)
	    gt.append(list(g))
	    

build()
#print_plane()
draw_plane(0)


dpm3=dpm3_thread()
dpm3.start()

proxy = m3p.M3RtProxy()
proxy.start()
proxy.make_operational_all()
act=mcf.create_component('m3joint_ma12_j1')
pwr=mcf.create_component('m3pwr_pwr015')
proxy.publish_command(pwr)	
proxy.subscribe_status(act)
proxy.publish_command(act)
proxy.publish_param(act)
proxy.step()
pwr.set_motor_power_on()
dd=100.0
u=u_min
fill_cells_random()
#triangle_wave()
#try:
    #while True:
	#print '---------------------'
	#print 'f: free-run'
	#print 'c: collect-data'
	#c=m3t.get_keystroke()
	#if c=='f':
	    #pass
	#if c=='c':
	    #pass
	#u=act.get_torque_mNm()
	#lmNm=dpm3.get_load_mNm()
	#print 'tq: ',u
	#proxy.step()
	#step_input(u)
	#draw_plane(u)
	#time.sleep(0.03)
#except (KeyboardInterrupt,EOFError):
    #pass
pwr.set_motor_power_off()
dpm3.stop()
proxy.step()
proxy.stop()