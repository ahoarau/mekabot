#!/bin/sh
export ROS_ROOT=/opt/ros/cturtle/ros
export PATH=$ROS_ROOT/bin:$PATH
export PYTHONPATH=$ROS_ROOT/core/roslib/src:$PYTHONPATH
if [ ! "$ROS_MASTER_URI" ] ; then export ROS_MASTER_URI=http://localhost:11311 ; fi
export ROS_PACKAGE_PATH=/opt/ros/cturtle/stacks/chomp_motion_planner:/opt/ros/cturtle/stacks/ompl_ros_interface:/opt/ros/cturtle/stacks/ompl:/opt/ros/cturtle/stacks/trajectory_filters:/opt/ros/cturtle/stacks/pr2_kinematics_with_constraints:/opt/ros/cturtle/stacks/pr2_kinematics:/opt/ros/cturtle/stacks/pr2_arm_navigation_tests:/opt/ros/cturtle/stacks/pr2_arm_navigation_apps:/opt/ros/cturtle/stacks/pr2_arm_navigation:/opt/ros/cturtle/stacks/motion_planning_visualization:/opt/ros/cturtle/stacks/motion_planning_environment:/opt/ros/cturtle/stacks/motion_planning_common:/opt/ros/cturtle/stacks/kinematics:/opt/ros/cturtle/stacks/collision_environment:/opt/ros/cturtle/stacks/arm_navigation:/opt/ros/cturtle/stacks
