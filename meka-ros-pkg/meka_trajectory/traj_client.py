#! /usr/bin/env python

# Copyright (c) 2009, Willow Garage, Inc.
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
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
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

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import meka_trajectory.msg
import trajectory_msgs.msg

def traj_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    ndof = 7
    
    client = actionlib.SimpleActionClient('traj', meka_trajectory.msg.TrajAction)
    
    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()
    
    # Creates a goal to send to the action server.
    goal = meka_trajectory.msg.TrajGoal()
    goal.trajectory.joint_names.append('m3joint_ma10_j0')
    goal.trajectory.joint_names.append('m3joint_ma10_j1')
    goal.trajectory.joint_names.append('m3joint_ma10_j2')
    goal.trajectory.joint_names.append('m3joint_ma10_j3')
    goal.trajectory.joint_names.append('m3joint_ma10_j4')
    goal.trajectory.joint_names.append('m3joint_ma10_j5')
    goal.trajectory.joint_names.append('m3joint_ma10_j6')
    
    for i in range(2):
        goal.trajectory.points.append(trajectory_msgs.msg.JointTrajectoryPoint())
        goal.trajectory.points[-1].time_from_start = rospy.Duration(i)
        for j in range(ndof):
            goal.trajectory.points[-1].positions.append(i)
            goal.trajectory.points[-1].velocities.append(0.0)
            

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('traj_client_py')
        result = traj_client()
        print "Result:", result
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
