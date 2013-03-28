#! /usr/bin/env python

# Copyright (c) 2010, Washington University in St. Louis
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Washington University in St. Louis nor the 
#       names of its contributors may be used to endorse or promote products 
#       derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import roslib; roslib.load_manifest('meka_trajectory')
import rospy
from m3.unit_conversion import *
import m3.component_factory as m3f
import m3.toolbox as m3t
import m3.trajectory as m3jt
import m3.rt_proxy as m3p
import m3.toolbox as m3t
import actionlib
import time
import meka_trajectory.msg

class TrajAction(object):
        # create messages that are used to publish feedback/result
        _feedback = meka_trajectory.msg.TrajFeedback()
        _result   = meka_trajectory.msg.TrajResult()

        def __init__(self, name):
                self._action_name = name
                self._as = actionlib.SimpleActionServer(self._action_name, meka_trajectory.msg.TrajAction, execute_cb=self.execute_cb)
                self.proxy = m3p.M3RtProxy()
                self.proxy.start()
                bot_name=m3t.get_robot_name()		
                self.bot=m3f.create_component(bot_name)
                self.chains = self.bot.get_available_chains()
                
                self.bot.initialize(self.proxy)

        def execute_cb(self, goal):
                # helper variables
                r = rospy.Rate(25)
                success = True


                # publish info to the console for the user
                rospy.loginfo('%s: Executing traj server.' % (self._action_name))
                
                # start executing the action

                # check that preempt has not been requested by the client
                if self._as.is_preempt_requested():
                        rospy.loginfo('%s: Preempted' % self._action_name)
                        self._as.set_preempted()
                        success = False
                chains_in_use = []
                done = True
                # pair joints with their chain and put in order and then add
                print self.chains
                for c in self.chains:
                        self.bot.set_mode_off(c)
                        ndof = self.bot.get_num_dof(c)
                        vias = [0.0] * ndof
                        vias_vel = [0.0] * ndof;
                        joints = self.bot.get_joint_names(c)
                        
                        for k in range(len(goal.trajectory.points)):
                                is_chain_used = False
                                for i in range(len(goal.trajectory.joint_names)):
                                        for j in range(ndof):
                                                if goal.trajectory.joint_names[i] == joints[j]:
                                                        vias[j] = goal.trajectory.points[k].positions[i]
                                                        vias_vel[j] = goal.trajectory.points[k].velocities[i]
                                                        is_chain_used = True
                                if is_chain_used:
                                        self.bot.set_stiffness(c,[0.5]*ndof)
                                        self.bot.set_slew_rate_proportion(c, [1.0]*ndof)
                                        done=False
                                        print 'Adding via:', c, vias, vias_vel
                                        self.bot.add_splined_traj_via_deg(c,vias,vias_vel)
                                        chain_added_yet = False
                                        for u in chains_in_use:
                                                if c == u:
                                                        chain_added_yet = True
                                        if not chain_added_yet:
                                                chains_in_use.append(c)
                        self.proxy.step()

                for c in chains_in_use:
                        self.bot.set_mode_splined_traj_gc(c)
                self.proxy.step()

                ts=time.time()
                while not done:
                        done = True
                        for c in chains_in_use:
                                if not self.bot.is_splined_traj_complete(c):
                                        done=False

                        print 'Running...',time.time()-ts
                        self._as.publish_feedback(self._feedback)	
                        self.proxy.step()			
                        r.sleep()
                        if self._as.is_preempt_requested():
                                rospy.loginfo('%s: Preempted' % self._action_name)
                                self._as.set_preempted()
                                success = False
                                break

                for c in chains_in_use:
                        self.bot.set_mode_off(c)
                self.proxy.step()

                print 'Finished with traj action.'

                # publish the feedback

                # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes


                if success:
                        #self._result.sequence = self._feedback.sequence
                        rospy.loginfo('%s: Succeeded' % self._action_name)
                        self._as.set_succeeded(self._result)

if __name__ == '__main__':
        rospy.init_node('traj')
        TrajAction(rospy.get_name())
        rospy.spin()
