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

###################
#  CONFIGURATION  #
###################

#Gazebo
SET(gazebo_support $ENV{GAZEBO})
MESSAGE("--------- Shadow Robot Package Configuration:")
IF(gazebo_support)
  SET( CMAKE_CXX_FLAGS "-D GAZEBO")
  MESSAGE(" [x] Gazebo support")
ELSE(gazebo_support)
  MESSAGE(" [ ] Gazebo support")
ENDIF(gazebo_support)

# Real Hand
SET(real_hand $ENV{REAL_HAND})
IF(real_hand)
  MESSAGE(" [x] Real Hand.")
ELSE(real_hand)
  MESSAGE(" [ ] Real Hand.")
ENDIF(real_hand)

#Real Arm
SET(real_arm $ENV{REAL_ARM})
IF(real_arm)
  MESSAGE(" [x] Real Arm.")
ELSE(real_arm)
  MESSAGE(" [ ] Real Arm.")
ENDIF(real_arm)

#Valves
SET(muscle $ENV{MUSCLE})
IF(real_hand)
  IF(muscle)
    MESSAGE(" [x] Valves.")
  ELSE(muscle)
    MESSAGE(" [ ] Valves.")
  ENDIF(muscle)
ELSE(real_hand)
  MESSAGE(" [ ] Valves.")
ENDIF(real_hand)
MESSAGE("---------")

#EtherCAT compatibility Interface
SET(ethercat_compat $ENV{ETHERCAT})
IF(ethercat_compat)
  MESSAGE(" [x] EtherCAT compatibility interface.")
ELSE(ethercat_compat)
  MESSAGE(" [ ] EtherCAT compatibility interface.")
ENDIF(ethercat_compat)

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


###########################
#  VIRTUAL HAND  LIBRARY  #
###########################

rosbuild_add_library(SrVirtual src/hand/sr_articulated_robot.cpp src/hand/virtual_shadowhand_library.cpp)
rosbuild_link_boost(shadowhand_virtual thread)

##################
#    REAL HAND   #
##################
IF(real_hand)
  include_directories(/usr/realtime/include/)
  LINK_LIBRARIES(pthread)
  add_library(robot STATIC IMPORTED)
  set_property(TARGET robot PROPERTY IMPORTED_LOCATION /usr/lib/robot.a)
  rosbuild_add_executable(shadowhand_real src/hand/sr_articulated_robot.cpp src/hand/real_shadowhand.cpp src/sr_publisher.cpp src/sr_subscriber.cpp src/hand/real_shadowhand_node.cpp src/sr_diagnosticer.cpp)
  rosbuild_link_boost(shadowhand_real thread)
  target_link_libraries(shadowhand_real robot)
ENDIF(real_hand)

#########################################
#  ETHERCAT COMPATIBILITY WRAPPER HAND  #
#########################################
IF(ethercat_compat)
  rosbuild_add_executable(shadowhand_ethercat src/hand/sr_articulated_robot.cpp src/hand/etherCAT_compatibility_hand.cpp src/sr_publisher.cpp src/sr_subscriber.cpp src/hand/etherCAT_compatibility_hand_node.cpp src/sr_diagnosticer.cpp)
  rosbuild_link_boost(shadowhand_virtual thread)
ENDIF(ethercat_compat)

###################################
#  CAN COMPATIBILITY WRAPPER ARM  #
###################################
rosbuild_add_executable(shadowarm_can src/hand/sr_articulated_robot.cpp src/hand/CAN_compatibility_arm.cpp src/sr_publisher.cpp src/sr_subscriber.cpp src/hand/CAN_compatibility_arm_node.cpp)


##################
#    REAL ARM    #
##################
IF(real_arm)
  include_directories(/usr/realtime/include/)
  LINK_LIBRARIES(pthread)
  rosbuild_add_executable(shadowarm_real src/hand/sr_articulated_robot.cpp src/hand/real_arm.cpp src/sr_publisher.cpp src/sr_subscriber.cpp src/hand/real_arm_node.cpp src/sr_diagnosticer.cpp)
  rosbuild_link_boost(shadowarm_real thread)
  target_link_libraries(shadowarm_real robot)
ENDIF(real_arm)

##################
#     VALVES     #
##################
IF(valves)
  include_directories(/usr/realtime/include/)
  LINK_LIBRARIES(pthread)
  rosbuild_add_executable(sr_valves src/hand/valves.cpp src/hand/valves_node.cpp)
  rosbuild_link_boost(sr_valves thread)
  target_link_libraries(sr_valves robot)
ENDIF(valves)

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
