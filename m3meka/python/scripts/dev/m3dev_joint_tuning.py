#! /usr/bin/python

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

import time
import m3.gui as m3g
import m3.rt_proxy as m3p
import m3.actuator as m3c
import m3.actuator_ec as m3ec
import m3.joint as m3
import m3.toolbox as m3t
import m3.unit_conversion as m3u
import m3.pwr
import m3.component_factory as m3f
import m3.actuator_pb2 as mrt 
import math

class M3Proc:
    def __init__(self):
        self.proxy = m3p.M3RtProxy()
        self.gui = m3g.M3Gui(stride_ms=125)#125
    def stop(self):
        self.proxy.stop()
    def start(self):
        self.proxy.start()
        joint_names=self.proxy.get_joint_components()
        if len(joint_names)==0:
            print 'No joint components available'
            self.proxy.stop()
            exit()
        joint_names=m3t.user_select_components_interactive(joint_names)
        actuator_ec_names=[]
        actuator_names=[]
        ctrl_names=[]
        for n in joint_names:
            ctrl = m3t.get_joint_ctrl_component_name(n)
            if ctrl != "":
                ctrl_names.append(ctrl)
            actuator = m3t.get_joint_actuator_component_name(n)
            if actuator != "":
                actuator_names.append(actuator)
                actuator_ec = m3t.get_actuator_ec_component_name(actuator)
                if actuator_ec != "":
                    actuator_ec_names.append(actuator_ec)

       

        self.joint=[]
        self.actuator_ec=[]
        self.actuator=[]
        self.ctrl=[]

        for n in actuator_ec_names:
            c=m3f.create_component(n)
            if c is not None:
                try:
                    self.actuator_ec.append(c)
                    self.proxy.subscribe_status(self.actuator_ec[-1])
                    self.proxy.publish_param(self.actuator_ec[-1]) 
                except:
                    print 'Component',n,'not available'

        for n in actuator_names:
            c=m3f.create_component(n)
            if c is not None:
                self.actuator.append(c)
                self.proxy.subscribe_status(self.actuator[-1])
                self.proxy.publish_param(self.actuator[-1]) 
                
        for n in ctrl_names:
            print "creating" + n
            c=m3f.create_component(n)
            if c is not None:
                self.ctrl.append(c)
                self.proxy.subscribe_status(self.ctrl[-1])
                self.proxy.publish_param(self.ctrl[-1]) 

        for n in joint_names:
            c=m3f.create_component(n)
            if c is not None:
                self.joint.append(c)
                self.proxy.subscribe_status(self.joint[-1])
                self.proxy.publish_command(self.joint[-1])
                self.proxy.publish_param(self.joint[-1]) 

        #Enable motor power
	pwr_rt=m3t.get_actuator_ec_pwr_component_name(actuator_ec_names[0])
	self.pwr=m3f.create_component(pwr_rt)
	if self.pwr is not None:
	    self.proxy.publish_command(self.pwr)
	    self.pwr.set_motor_power_on()

        #Start them all up
        self.proxy.make_operational_all()

        #Force safe-op of robot, etc are present
        types=['m3humanoid','m3hand','m3gripper']
        for t in types:
            cc=self.proxy.get_available_components(t)
            for ccc in cc:
                self.proxy.make_safe_operational(ccc)

        #Force safe-op of chain so that gravity terms are computed
        self.chain=None	
        if len(joint_names)>0:
            for j in joint_names:
                chain_name=m3t.get_joint_chain_name(j)
                if chain_name!="":
                    self.proxy.make_safe_operational(chain_name)
                    #self.chain=m3f.create_component(chain_name)
                    #self.proxy.publish_param(self.chain) #allow to set payload
                    #Force safe-op of chain so that gravity terms are computed
                    dynamatics_name = m3t.get_chain_dynamatics_component_name(chain_name)
                    if dynamatics_name != "":	    
                        self.proxy.make_safe_operational(dynamatics_name)
                        self.dyn=m3f.create_component(dynamatics_name)
                        self.proxy.publish_param(self.dyn) #allow to set payload


        #Force safe-op of robot so that gravity terms are computed
        robot_name = m3t.get_robot_name()
        if robot_name != "":
            self.proxy.make_safe_operational(robot_name)
            self.robot=m3f.create_component(robot_name)
            self.proxy.publish_param(self.robot) #allow to set payload	

        tmax=max([x.param.max_tq for x in self.actuator])
        tmin=min([x.param.min_tq for x in self.actuator])

        qmax=max([x.param.max_q for x in self.joint])
        qmin=min([x.param.min_q for x in self.joint])
        self.scope_torque=None
	self.scope_theta=None
        #Create gui
        self.mode=[0]*len(self.joint)
        self.tq_desire_a=[0]*len(self.joint)
	self.tq_desire_b=[0]*len(self.joint)
        self.pwm_desire=[0]*len(self.joint)
        self.theta_desire_a=[0]*len(self.joint)
        self.theta_desire_b=[0]*len(self.joint)
        self.thetadot_desire=[0]*len(self.joint)
        self.stiffness=[0]*len(self.joint)
        self.slew=[0]*len(self.joint)
        self.step_period=[2000.0]*len(self.joint)
        self.cycle_theta=False
        self.cycle_last_theta=False
	self.cycle_torque=False
        self.cycle_last_torque=False
        self.save=False
	self.do_scope_torque=False
	self.do_scope_theta=False
        self.brake=False
        self.save_last=False
        self.status_dict=self.proxy.get_status_dict()
        self.param_dict=self.proxy.get_param_dict()
        self.gui.add('M3GuiTree',   'Status',    (self,'status_dict'),[],[],m3g.M3GuiRead,column=2)
        self.gui.add('M3GuiTree',   'Param',   (self,'param_dict'),[],[],m3g.M3GuiWrite,column=3)
        self.gui.add('M3GuiModes',  'Mode',      (self,'mode'),range(len(self.joint)),[['Off','Pwm','Torque','Theta','Torque_GC','Theta_GC','Theta_MJ', 'Theta_GC_MJ'],1],m3g.M3GuiWrite)
        self.gui.add('M3GuiSliders','TorqueA (mNm)',  (self,'tq_desire_a'),range(len(self.joint)),[tmin,tmax],m3g.M3GuiWrite)
	self.gui.add('M3GuiSliders','TorqueB (mNm)',  (self,'tq_desire_b'),range(len(self.joint)),[tmin,tmax],m3g.M3GuiWrite)
        self.gui.add('M3GuiSliders','Pwm', (self,'pwm_desire'),range(len(self.joint)),[-3200,3200],m3g.M3GuiWrite) 
        self.gui.add('M3GuiSliders','Theta A(Deg)', (self,'theta_desire_a'),range(len(self.joint)),[qmin,qmax],m3g.M3GuiWrite,column=2) 
        self.gui.add('M3GuiSliders','Theta B(Deg)', (self,'theta_desire_b'),range(len(self.joint)),[qmin,qmax],m3g.M3GuiWrite,column=2) 	
        self.gui.add('M3GuiSliders','Thetadot (Deg)', (self,'thetadot_desire'),range(len(self.joint)),[0,120.0],m3g.M3GuiWrite,column=2) 		
        self.gui.add('M3GuiSliders','Stiffness ', (self,'stiffness'),range(len(self.joint)),[0,100],m3g.M3GuiWrite,column=3) 
        self.gui.add('M3GuiSliders','Slew ', (self,'slew'),range(len(self.joint)),[0,100],m3g.M3GuiWrite,column=3) 
        self.gui.add('M3GuiToggle', 'Save',      (self,'save'),[],[['On','Off']],m3g.M3GuiWrite)
	self.gui.add('M3GuiToggle', 'ScopeTorque',      (self,'do_scope_torque'),[],[['On','Off']],m3g.M3GuiWrite)
	self.gui.add('M3GuiToggle', 'ScopeTheta',      (self,'do_scope_theta'),[],[['On','Off']],m3g.M3GuiWrite)
        self.gui.add('M3GuiSliders','StepPeriod (ms) ', (self,'step_period'),range(len(self.joint)),[0,4000],m3g.M3GuiWrite) 	
        self.gui.add('M3GuiToggle', 'CycleTheta',      (self,'cycle_theta'),[],[['On','Off']],m3g.M3GuiWrite)
	self.gui.add('M3GuiToggle', 'CycleTorque',      (self,'cycle_torque'),[],[['On','Off']],m3g.M3GuiWrite)
        self.gui.add('M3GuiToggle', 'Brake',      (self,'brake'),[],[['On','Off']],m3g.M3GuiWrite)
        self.gui.start(self.step)

    def step(self):
        self.proxy.step()
        self.status_dict=self.proxy.get_status_dict()
        self.proxy.set_param_from_dict(self.param_dict)
        idx=0
        current=0
        #self.proxy.pretty_print_component(self.actuator_ec[0].name)
        #self.proxy.pretty_print_component(self.joint[0].name)
	if self.do_scope_torque and self.scope_torque is None and len(self.joint)==1:
	    self.scope_torque=m3t.M3Scope2(xwidth=100,yrange=None)
	if self.do_scope_theta and self.scope_theta is None and len(self.joint)==1:
	    self.scope_theta=m3t.M3Scope2(xwidth=100,yrange=None)
        for c in self.joint:
            print 'Theta',c.name,c.get_theta_deg()
            if self.brake:
                c.set_brake_off()
            else:
                c.set_brake_on()
            #print 'Force',c.get_torque_inLb()/28.3
            #print 'Temp: ',c.name,c.get_motor_temp_F(), c.get_amp_temp_F()
            print

            if not self.cycle_last_theta and self.cycle_theta:
                self.step_start=time.time()
            self.cycle_last_theta=self.cycle_theta

	    if not self.cycle_last_torque and self.cycle_torque:
                self.step_start=time.time()
            self.cycle_last_torque=self.cycle_torque
	    
            td=self.theta_desire_a[idx]
	    tqd=self.tq_desire_a[idx]
            if self.cycle_theta:
                dt=time.time()-self.step_start
                if math.fmod(dt,self.step_period[idx]/1000.0)>self.step_period[idx]/2000.0:
                    td=self.theta_desire_b[idx]
		    
	    if self.cycle_torque:
                dt=time.time()-self.step_start
                if math.fmod(dt,self.step_period[idx]/1000.0)>self.step_period[idx]/2000.0:
                    tqd=self.tq_desire_b[idx]

            c.set_theta_deg(td)
	    c.set_torque_mNm(tqd)
	    if self.do_scope_theta and self.scope_theta is not None:
		self.scope_theta.plot(c.get_theta_deg(),td)
	    if self.do_scope_torque and self.scope_torque is not None:
		self.scope_torque.plot(c.get_torque_mNm(),tqd)
            current=current+c.get_current_mA()
            c.set_control_mode(self.mode[idx])
            c.set_pwm(int(self.pwm_desire[idx]))
            #c.set_torque_mNm(self.tq_desire[idx])
            c.set_thetadot_deg(self.thetadot_desire[idx])
            #c.set_theta_deg(self.theta_desire[idx])
            c.set_stiffness(self.stiffness[idx]/100.0)
            c.set_slew_rate_proportion(self.slew[idx]/100.0)
            if (self.save and not self.save_last):
                c.write_config()
                if idx<len(self.actuator_ec):
                    self.actuator_ec[idx].write_config()
                self.actuator[idx].write_config()
            idx=idx+1
        #print 'Total current (mA)',current
        if (self.save and not self.save_last and self.chain is not None):
            self.chain.write_config()
        self.save_last=self.save

if __name__ == '__main__':
    t=M3Proc()
    try:
        t.start()
    except (KeyboardInterrupt,EOFError):
        pass
    t.stop()




