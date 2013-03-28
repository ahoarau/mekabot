cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()
rosbuild_gensrv()
rosbuild_genmsg()
#rosbuild_add_executable(imu_node src/imu_node.cpp)
#target_link_libraries(imu_node Client)

#rosbuild_add_library(Client src/Client.cpp)

