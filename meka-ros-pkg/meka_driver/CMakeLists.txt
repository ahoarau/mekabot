cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)
set(ROS_BUILD_TYPE Debug)

rosbuild_find_ros_package(actionlib_msgs)
include(${actionlib_msgs_PACKAGE_PATH}/cmake/actionbuild.cmake)
genaction()


rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()
#rosbuild_add_library(meka_client
#  src/humanoid_client.cpp  
#)

#common commands for building c++ executables and libraries
#rosbuild_add_library(meka_client src/humanoid_client.cpp)
#target_link_libraries(${PROJECT_NAME} meka_client)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(meka meka_client)

rosbuild_add_executable(meka src/MekaNode.cpp src/AbstractMeka.cpp src/Meka.cpp src/SimulatedMeka.cpp src/JointStatePublisher.cpp src/joint_trajectory_action_controller.cpp src/joint_movement_action_controller.cpp src/spline_functions.cpp)
#target_link_libraries(meka meka_client)
#rosbuild_add_executable(meka_arm_kinematics src/KNIConverter.cpp src/KNIKinematics.cpp)

#rosbuild_add_executable(meka src/MekaNode.cpp src/AbstractMeka.cpp src/Meka.cpp src/SimulatedMeka.cpp src/JointStatePublisher.cpp src/joint_trajectory_action_controller.cpp src/joint_movement_action_controller.cpp src/spline_functions.cpp)
#rosbuild_add_executable(katana_arm_kinematics src/KNIConverter.cpp src/KNIKinematics.cpp)
