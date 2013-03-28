cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()
#rosbuild_gensrv()
rosbuild_genmsg()
add_definitions(-pedantic -Wall)
rosbuild_add_executable(leaf_node src/leaf_node.cpp)

#rosbuild_add_library(Client src/Client.cpp)

