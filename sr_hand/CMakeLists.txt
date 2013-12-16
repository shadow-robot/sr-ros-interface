cmake_minimum_required(VERSION 2.8.3)
project(sr_hand)

find_package(catkin REQUIRED COMPONENTS roslib roscpp rospy std_msgs std_srvs sensor_msgs sr_robot_msgs pr2_controllers_msgs pr2_mechanism_msgs
  sr_utilities diagnostic_msgs dynamic_reconfigure self_test urdf gazebo_msgs)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

###################
#  CONFIGURATION  #
###################

#Gazebo
set(gazebo_support $ENV{GAZEBO})
message("--------- Shadow Robot Package Configuration:")
if(gazebo_support)
  set( CMAKE_CXX_FLAGS "-D GAZEBO")
  message(" [x] Gazebo support")
else(gazebo_support)
  message(" [ ] Gazebo support")
endif(gazebo_support)

# Real Hand
set(real_hand $ENV{REAL_HAND})
if(real_hand)
  message(" [x] Real Hand.")
else(real_hand)
  message(" [ ] Real Hand.")
endif(real_hand)

#Real Arm
set(real_arm $ENV{REAL_ARM})
if(real_arm)
  message(" [x] Real Arm.")
else(real_arm)
  message(" [ ] Real Arm.")
endif(real_arm)

#Valves
set(muscle $ENV{MUSCLE})
if(real_hand)
  if(muscle)
    message(" [x] Valves.")
  else(muscle)
    message(" [ ] Valves.")
  endif(muscle)
else(real_hand)
  message(" [ ] Valves.")
endif(real_hand)
message("---------")

#EtherCAT compatibility Interface
set(ethercat_compat $ENV{ETHERCAT})
if(ethercat_compat)
  message(" [x] EtherCAT compatibility interface.")
else(ethercat_compat)
  message(" [ ] EtherCAT compatibility interface.")
endif(ethercat_compat)

##################
#  VIRTUAL ARM  #
##################

add_executable(shadowarm_virtual src/hand/sr_articulated_robot.cpp src/hand/virtual_arm.cpp src/sr_publisher.cpp src/sr_subscriber.cpp
  src/hand/virtual_arm_node.cpp src/sr_diagnosticer.cpp)
find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(shadowarm_virtual ${Boost_LIBRARIES} ${catkin_LIBRARIES})


##################
#  VIRTUAL HAND  #
##################
add_executable(shadowhand_virtual src/hand/sr_articulated_robot.cpp src/hand/virtual_shadowhand.cpp src/sr_publisher.cpp src/sr_subscriber.cpp
  src/hand/virtual_shadowhand_node.cpp src/sr_diagnosticer.cpp)
target_link_libraries(shadowhand_virtual ${Boost_LIBRARIES} ${catkin_LIBRARIES})


###########################
#  VIRTUAL HAND  LIBRARY  #
###########################

add_library(SrVirtual src/hand/sr_articulated_robot.cpp src/hand/virtual_shadowhand_library.cpp)
target_link_libraries(SrVirtual ${Boost_LIBRARIES} ${catkin_LIBRARIES})

##################
#    REAL HAND   #
##################
if(real_hand)
  include_directories(/usr/realtime/include/)
  add_library(robot STATIC IMPORTED)
  set_property(TARGET robot PROPERTY IMPORTED_LOCATION /usr/lib/robot.a)
  add_executable(shadowhand_real src/hand/sr_articulated_robot.cpp src/hand/real_shadowhand.cpp src/sr_publisher.cpp src/sr_subscriber.cpp
    src/hand/real_shadowhand_node.cpp src/sr_diagnosticer.cpp)
  target_link_libraries(shadowhand_real ${Boost_LIBRARIES} ${catkin_LIBRARIES} pthread robot)
endif(real_hand)

#########################################
#  ETHERCAT COMPATIBILITY WRAPPER HAND  #
#########################################
if(ethercat_compat)
  add_executable(shadowhand_ethercat src/hand/sr_articulated_robot.cpp src/hand/etherCAT_compatibility_hand.cpp src/sr_publisher.cpp
    src/sr_subscriber.cpp src/hand/etherCAT_compatibility_hand_node.cpp src/sr_diagnosticer.cpp)
  target_link_libraries(shadowhand_ethercat ${Boost_LIBRARIES} ${catkin_LIBRARIES})
endif(ethercat_compat)

###################################
#  CAN COMPATIBILITY WRAPPER ARM  #
###################################
add_executable(shadowarm_can src/hand/sr_articulated_robot.cpp src/hand/CAN_compatibility_arm.cpp src/sr_publisher.cpp
  src/sr_subscriber.cpp src/hand/CAN_compatibility_arm_node.cpp)
target_link_libraries(shadowarm_can ${Boost_LIBRARIES} ${catkin_LIBRARIES})

##################
#    REAL ARM    #
##################
if(real_arm)
  include_directories(/usr/realtime/include/)
  add_executable(shadowarm_real src/hand/sr_articulated_robot.cpp src/hand/real_arm.cpp src/sr_publisher.cpp src/sr_subscriber.cpp
    src/hand/real_arm_node.cpp src/sr_diagnosticer.cpp)
  target_link_libraries(shadowarm_real ${Boost_LIBRARIES} ${catkin_LIBRARIES} pthread robot)
endif(real_arm)

##################
#     VALVES     #
##################
if(valves)
  include_directories(/usr/realtime/include/)
  add_executable(sr_valves src/hand/valves.cpp src/hand/valves_node.cpp)
  target_link_libraries(sr_valves ${Boost_LIBRARIES} ${catkin_LIBRARIES} pthread robot)
endif(valves)

############################################################################
#   Hand command library (gives compatibility with CAN and ethercat hand)  #
############################################################################
add_definitions(-fPIC)
add_library(HandCommander STATIC src/hand_commander.cpp)
target_link_libraries(HandCommander ${Boost_LIBRARIES} ${catkin_LIBRARIES})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

catkin_package(
    DEPENDS
    CATKIN_DEPENDS roslib roscpp rospy std_msgs std_srvs sensor_msgs sr_robot_msgs pr2_controllers_msgs pr2_mechanism_msgs sr_utilities
      diagnostic_msgs dynamic_reconfigure self_test urdf gazebo_msgs
    INCLUDE_DIRS include
    LIBRARIES
)

if(COMMAND add_rostest_gtest)
  link_directories(/opt/ros/hydro/lib)
  add_rostest_gtest(hand_commander_test test/hand_commander_test.test test/hand_commander_test.cpp)
  target_link_libraries(hand_commander_test HandCommander ${GTEST_LIBRARIES} ${catkin_LIBRARIES})
  add_rostest_gtest(hand_commander_arm_and_hand_test test/hand_commander_arm_and_hand_test.test test/hand_commander_test.cpp)
  target_link_libraries(hand_commander_arm_and_hand_test HandCommander ${GTEST_LIBRARIES} ${catkin_LIBRARIES})
endif(COMMAND add_rostest_gtest)

install(TARGETS shadowhand_virtual shadowarm_virtual SrVirtual shadowhand_real shadowhand_ethercat shadowarm_can shadowarm_real sr_valves HandCommander
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  OPTIONAL
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
  PATTERN ".svn" EXCLUDE
)