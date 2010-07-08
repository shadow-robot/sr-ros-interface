cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

rosbuild_add_boost_directories()

##################
#  VIRTUAL ARM  #
##################

rosbuild_add_executable(shadowarm_virtual src/hand/shadowhand.cpp src/hand/virtual_arm.cpp src/sr_publisher.cpp src/sr_subscriber.cpp src/hand/virtual_arm_node.cpp src/sr_diagnosticer.cpp)
rosbuild_link_boost(shadowarm_virtual thread)


##################
#  VIRTUAL HAND  #
##################

rosbuild_add_executable(shadowhand_virtual src/hand/shadowhand.cpp src/hand/virtual_shadowhand.cpp src/sr_publisher.cpp src/sr_subscriber.cpp src/hand/virtual_shadowhand_node.cpp src/sr_diagnosticer.cpp)
rosbuild_link_boost(shadowhand_virtual thread)

##################
#    REAL HAND   #
##################

#uncomment those lines if you build for the real hand
#include_directories(/usr/realtime/include/)
#LINK_LIBRARIES(pthread)
#add_library(robot STATIC IMPORTED) 
#set_property(TARGET robot PROPERTY IMPORTED_LOCATION /usr/lib/robot.a)
#rosbuild_add_executable(shadowhand_real src/hand/shadowhand.cpp src/hand/real_shadowhand.cpp src/sr_publisher.cpp src/sr_subscriber.cpp src/hand/real_shadowhand_node.cpp src/sr_diagnosticer.cpp)
#rosbuild_link_boost(shadowhand_real thread)
#target_link_libraries(shadowhand_real robot)

#####################################
#   dynamic parameters reconfigure  #
#####################################

#rosbuild_find_ros_package(dynamic_reconfigure)
#include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
#gencfg()

###############
#   EXAMPLES  #
###############

rosbuild_add_executable(examples/link_joints examples/link_joints.cpp)