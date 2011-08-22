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
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

rosbuild_add_boost_directories()


#if you don't want to have the gazebo interface for the virtual hardware
# just comment this flag
IF($ENV{GAZEBO} EQUAL 1)
  SET( CMAKE_CXX_FLAGS "-D GAZEBO")
  MESSAGE("Compiling with Gazebo support")
ELSE($ENV{GAZEBO} EQUAL 1)
  MESSAGE("Compiling without Gazebo support")
ENDIF($ENV{GAZEBO} EQUAL 1)

#EtherCAT compatibility Interface
SET(ethercat_compat $ENV{ETHERCAT})
IF(DEFINED ethercat_compat)
  MESSAGE(" [x] EtherCAT compatibility interface.")
ELSE(DEFINED ethercat_compat)
  MESSAGE(" [ ] EtherCAT compatibility interface.")
ENDIF(DEFINED ethercat_compat)
##################
#  VIRTUAL ARM  #
##################

rosbuild_add_executable(shadowarm_virtual src/hand/sr_articulated_robot.cpp src/hand/virtual_arm.cpp src/sr_publisher.cpp src/sr_subscriber.cpp src/hand/virtual_arm_node.cpp src/sr_diagnosticer.cpp)
rosbuild_link_boost(shadowarm_virtual thread)


##################
#  VIRTUAL HAND  #
##################
rosbuild_add_executable(shadowhand_virtual src/hand/sr_articulated_robot.cpp src/hand/virtual_shadowhand.cpp src/sr_publisher.cpp src/sr_subscriber.cpp src/hand/virtual_shadowhand_node.cpp src/sr_diagnosticer.cpp)
rosbuild_link_boost(shadowhand_virtual thread)

#########################################
#  ETHERCAT COMPATIBILITY WRAPPER HAND  #
#########################################
IF(DEFINED ethercat_compat)
  rosbuild_add_executable(shadowhand_ethercat src/hand/sr_articulated_robot.cpp src/hand/etherCAT_compatibility_hand.cpp src/sr_publisher.cpp src/sr_subscriber.cpp src/hand/etherCAT_compatibility_hand_node.cpp src/sr_diagnosticer.cpp)
  rosbuild_link_boost(shadowhand_virtual thread)
ENDIF(DEFINED ethercat_compat)

##################
#    REAL HAND   #
##################

##uncomment those lines if you build for the real hand
#include_directories(/usr/realtime/include/)
#LINK_LIBRARIES(pthread)
#add_library(robot STATIC IMPORTED)
#set_property(TARGET robot PROPERTY IMPORTED_LOCATION /usr/lib/robot.a)
#rosbuild_add_executable(shadowhand_real src/hand/sr_articulated_robot.cpp src/hand/real_shadowhand.cpp src/sr_publisher.cpp src/sr_subscriber.cpp src/hand/real_shadowhand_node.cpp src/sr_diagnosticer.cpp)
#rosbuild_link_boost(shadowhand_real thread)
#target_link_libraries(shadowhand_real robot)

##################
#    REAL ARM    #
##################

#uncomment those lines if you build for the real hand
#include_directories(/usr/realtime/include/)
#LINK_LIBRARIES(pthread)
#rosbuild_add_executable(shadowarm_real src/hand/sr_articulated_robot.cpp src/hand/real_arm.cpp src/sr_publisher.cpp src/sr_subscriber.cpp src/hand/real_arm_node.cpp src/sr_diagnosticer.cpp)
#rosbuild_link_boost(shadowarm_real thread)
#target_link_libraries(shadowarm_real robot)

##################
#     VALVES     #
##################
#include_directories(/usr/realtime/include/)
#LINK_LIBRARIES(pthread)
#rosbuild_add_executable(sr_valves src/hand/valves.cpp src/hand/valves_node.cpp)
#rosbuild_link_boost(sr_valves thread)
#target_link_libraries(sr_valves robot)

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
