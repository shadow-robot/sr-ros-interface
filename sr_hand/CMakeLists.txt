# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(sr_hand)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS roslib roscpp rospy std_msgs std_srvs sensor_msgs sr_robot_msgs pr2_controllers_msgs pr2_mechanism_msgs sr_utilities diagnostic_msgs dynamic_reconfigure self_test urdf gazebo_msgs)

# include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
# CATKIN_MIGRATION: removed during catkin migration
# cmake_minimum_required(VERSION 2.4.6)

# CATKIN_MIGRATION: removed during catkin migration
# include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)


# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#add_message_files(
  FILES
  # TODO: List your msg files here
)
#uncomment if you have defined services
#add_service_files(
  FILES
  # TODO: List your msg files here
)


# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_add_boost_directories()

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

add_executable(shadowarm_virtual src/hand/sr_articulated_robot.cpp src/hand/virtual_arm.cpp src/sr_publisher.cpp src/sr_subscriber.cpp src/hand/virtual_arm_node.cpp src/sr_diagnosticer.cpp)
find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(shadowarm_virtual ${Boost_LIBRARIES})


##################
#  VIRTUAL HAND  #
##################
add_executable(shadowhand_virtual src/hand/sr_articulated_robot.cpp src/hand/virtual_shadowhand.cpp src/sr_publisher.cpp src/sr_subscriber.cpp src/hand/virtual_shadowhand_node.cpp src/sr_diagnosticer.cpp)
target_link_libraries(shadowhand_virtual ${Boost_LIBRARIES})


###########################
#  VIRTUAL HAND  LIBRARY  #
###########################

add_library(SrVirtual src/hand/sr_articulated_robot.cpp src/hand/virtual_shadowhand_library.cpp)
target_link_libraries(shadowhand_virtual ${Boost_LIBRARIES})

##################
#    REAL HAND   #
##################
IF(real_hand)
  include_directories(/usr/realtime/include/)
  LINK_LIBRARIES(pthread)
  add_library(robot STATIC IMPORTED)
  set_property(TARGET robot PROPERTY IMPORTED_LOCATION /usr/lib/robot.a)
  add_executable(shadowhand_real src/hand/sr_articulated_robot.cpp src/hand/real_shadowhand.cpp src/sr_publisher.cpp src/sr_subscriber.cpp src/hand/real_shadowhand_node.cpp src/sr_diagnosticer.cpp)
target_link_libraries(shadowhand_real ${Boost_LIBRARIES})
  target_link_libraries(shadowhand_real robot)
ENDIF(real_hand)

#########################################
#  ETHERCAT COMPATIBILITY WRAPPER HAND  #
#########################################
IF(ethercat_compat)
  add_executable(shadowhand_ethercat src/hand/sr_articulated_robot.cpp src/hand/etherCAT_compatibility_hand.cpp src/sr_publisher.cpp src/sr_subscriber.cpp src/hand/etherCAT_compatibility_hand_node.cpp src/sr_diagnosticer.cpp)
target_link_libraries(shadowhand_virtual ${Boost_LIBRARIES})
ENDIF(ethercat_compat)

###################################
#  CAN COMPATIBILITY WRAPPER ARM  #
###################################
add_executable(shadowarm_can src/hand/sr_articulated_robot.cpp src/hand/CAN_compatibility_arm.cpp src/sr_publisher.cpp src/sr_subscriber.cpp src/hand/CAN_compatibility_arm_node.cpp)


##################
#    REAL ARM    #
##################
IF(real_arm)
  include_directories(/usr/realtime/include/)
  LINK_LIBRARIES(pthread)
  add_executable(shadowarm_real src/hand/sr_articulated_robot.cpp src/hand/real_arm.cpp src/sr_publisher.cpp src/sr_subscriber.cpp src/hand/real_arm_node.cpp src/sr_diagnosticer.cpp)
target_link_libraries(shadowarm_real ${Boost_LIBRARIES})
  target_link_libraries(shadowarm_real robot)
ENDIF(real_arm)

##################
#     VALVES     #
##################
IF(valves)
  include_directories(/usr/realtime/include/)
  LINK_LIBRARIES(pthread)
  add_executable(sr_valves src/hand/valves.cpp src/hand/valves_node.cpp)
target_link_libraries(sr_valves ${Boost_LIBRARIES})
  target_link_libraries(sr_valves robot)
ENDIF(valves)

#####################################
#   dynamic parameters reconfigure  #
#####################################

#rosbuild_find_ros_package(dynamic_reconfigure)
#include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
#gencfg()


############################################################################
#   Hand command library (gives compatibility with CAN and ethercat hand)  #
############################################################################
add_library(HandCommander src/hand_commander.cpp)
target_link_libraries(HandCommander ${Boost_LIBRARIES})
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
#catkin_add_gtest(test/hand_commander_test test/hand_commander_test.cpp)
add_executable(test/hand_commander_test test/hand_commander_test.cpp)

# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_add_gtest_build_flags(test/hand_commander_test)
target_link_libraries(test/hand_commander_test HandCommander)
add_rostest(test/hand_commander_test.launch)
add_rostest(test/hand_commander_arm_and_hand_test.launch)
## Generate added messages and services with any dependencies listed here
generate_messages(
    #TODO DEPENDENCIES geometry_msgs std_msgs
)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
catkin_package(
    DEPENDS roslib roscpp rospy std_msgs std_srvs sensor_msgs sr_robot_msgs pr2_controllers_msgs pr2_mechanism_msgs sr_utilities diagnostic_msgs dynamic_reconfigure self_test urdf gazebo_msgs
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)