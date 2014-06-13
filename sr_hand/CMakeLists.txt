cmake_minimum_required(VERSION 2.8.3)
project(sr_hand)

find_package(catkin REQUIRED COMPONENTS rostest roslib roscpp rospy std_msgs std_srvs sensor_msgs sr_robot_msgs control_msgs controller_manager_msgs
  sr_utilities diagnostic_msgs dynamic_reconfigure self_test urdf gazebo_msgs)
find_package(gazebo REQUIRED)
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${SDFormat_INCLUDE_DIRS})

catkin_python_setup()

catkin_package(
    DEPENDS gazebo
    CATKIN_DEPENDS roslib roscpp rospy std_msgs std_srvs sensor_msgs sr_robot_msgs control_msgs controller_manager_msgs sr_utilities
      diagnostic_msgs dynamic_reconfigure self_test urdf gazebo_msgs
    INCLUDE_DIRS include
    LIBRARIES HandCommander SrVirtual
)

###################
#  CONFIGURATION  #
###################

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

#CAN compatibility Interface
set(can_compat $ENV{CAN_COMPATIBILITY})
if(can_compat)
  message(" [x] CAN compatibility interface.")
else(can_compat)
  message(" [ ] CAN compatibility interface.")
endif(can_compat)

##################
#  VIRTUAL ARM  #
##################

add_executable(shadowarm_virtual src/hand/sr_articulated_robot.cpp src/hand/virtual_arm.cpp src/sr_publisher.cpp src/sr_subscriber.cpp
  src/hand/virtual_arm_node.cpp src/sr_diagnosticer.cpp)
add_dependencies(shadowarm_virtual ${catkin_EXPORTED_TARGETS})
find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(shadowarm_virtual ${Boost_LIBRARIES} ${catkin_LIBRARIES})


##################
#  VIRTUAL HAND  #
##################
add_executable(shadowhand_virtual src/hand/sr_articulated_robot.cpp src/hand/virtual_shadowhand.cpp src/sr_publisher.cpp src/sr_subscriber.cpp
  src/hand/virtual_shadowhand_node.cpp src/sr_diagnosticer.cpp)
add_dependencies(shadowhand_virtual ${catkin_EXPORTED_TARGETS})
target_link_libraries(shadowhand_virtual ${Boost_LIBRARIES} ${catkin_LIBRARIES})


###########################
#  VIRTUAL HAND  LIBRARY  #
###########################

add_library(SrVirtual src/hand/sr_articulated_robot.cpp src/hand/virtual_shadowhand_library.cpp)
add_dependencies(SrVirtual ${catkin_EXPORTED_TARGETS})
target_link_libraries(SrVirtual ${Boost_LIBRARIES} ${catkin_LIBRARIES})

##################
#    REAL HAND   #
##################
if(real_hand)
  include_directories(/usr/realtime/include/)
  add_library(robot STATIC IMPORTED)
  add_dependencies(robot ${catkin_EXPORTED_TARGETS})
  set_property(TARGET robot PROPERTY IMPORTED_LOCATION /usr/lib/robot.a)
  add_executable(shadowhand_real src/hand/sr_articulated_robot.cpp src/hand/real_shadowhand.cpp src/sr_publisher.cpp src/sr_subscriber.cpp
    src/hand/real_shadowhand_node.cpp src/sr_diagnosticer.cpp)
  add_dependencies(shadowhand_real ${catkin_EXPORTED_TARGETS})
  target_link_libraries(shadowhand_real ${Boost_LIBRARIES} ${catkin_LIBRARIES} pthread robot)
endif(real_hand)

#########################################
#  ETHERCAT COMPATIBILITY WRAPPER HAND  #
#########################################
add_executable(shadowhand_ethercat src/hand/sr_articulated_robot.cpp src/hand/etherCAT_compatibility_hand.cpp src/sr_publisher.cpp
  src/sr_subscriber.cpp src/hand/etherCAT_compatibility_hand_node.cpp src/sr_diagnosticer.cpp)
add_dependencies(shadowhand_ethercat ${catkin_EXPORTED_TARGETS})
target_link_libraries(shadowhand_ethercat ${Boost_LIBRARIES} ${catkin_LIBRARIES})
message(" [x] EtherCAT compatibility interface.")

###################################
#  CAN COMPATIBILITY WRAPPER ARM  #
###################################
if(can_compat)
  add_executable(shadowarm_can src/hand/sr_articulated_robot.cpp src/hand/CAN_compatibility_arm.cpp src/sr_publisher.cpp
    src/sr_subscriber.cpp src/hand/CAN_compatibility_arm_node.cpp)
  add_dependencies(shadowarm_can ${catkin_EXPORTED_TARGETS})
  target_link_libraries(shadowarm_can ${Boost_LIBRARIES} ${catkin_LIBRARIES})
endif(can_compat)

##################
#    REAL ARM    #
##################
if(real_arm)
  include_directories(/usr/realtime/include/)
  add_executable(shadowarm_real src/hand/sr_articulated_robot.cpp src/hand/real_arm.cpp src/sr_publisher.cpp src/sr_subscriber.cpp
    src/hand/real_arm_node.cpp src/sr_diagnosticer.cpp)
  add_dependencies(shadowarm_real ${catkin_EXPORTED_TARGETS})
  target_link_libraries(shadowarm_real ${Boost_LIBRARIES} ${catkin_LIBRARIES} pthread robot)
endif(real_arm)

##################
#     VALVES     #
##################
if(valves)
  include_directories(/usr/realtime/include/)
  add_executable(sr_valves src/hand/valves.cpp src/hand/valves_node.cpp)
  add_dependencies(sr_valves ${catkin_EXPORTED_TARGETS})
  target_link_libraries(sr_valves ${Boost_LIBRARIES} ${catkin_LIBRARIES} pthread robot)
endif(valves)

############################################################################
#   Hand command library (gives compatibility with CAN and ethercat hand)  #
############################################################################
add_definitions(-fPIC)
add_library(HandCommander src/hand_commander.cpp)
add_dependencies(HandCommander ${catkin_EXPORTED_TARGETS})
target_link_libraries(HandCommander ${Boost_LIBRARIES} ${catkin_LIBRARIES})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")


if(COMMAND add_rostest_gtest)
  add_rostest_gtest(hand_commander_test test/hand_commander_test.test test/hand_commander_test.cpp)
  add_dependencies(hand_commander_test ${catkin_EXPORTED_TARGETS})
  target_link_libraries(hand_commander_test HandCommander ${GTEST_LIBRARIES} ${catkin_LIBRARIES})
  add_rostest(test/hand_commander_mock_test.test)
  add_rostest_gtest(hand_commander_arm_and_hand_test test/hand_commander_arm_and_hand_test.test test/hand_commander_test.cpp)
  add_dependencies(hand_commander_arm_and_hand_test ${catkin_EXPORTED_TARGETS})
  target_link_libraries(hand_commander_arm_and_hand_test HandCommander ${GTEST_LIBRARIES} ${catkin_LIBRARIES})
endif(COMMAND add_rostest_gtest)

if(valves)
  install(TARGETS shadowhand_virtual shadowarm_virtual SrVirtual shadowhand_ethercat HandCommander sr_valves
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
else(valves)
  install(TARGETS shadowhand_virtual shadowarm_virtual SrVirtual shadowhand_ethercat HandCommander
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endif()

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY examples/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/examples)

install(DIRECTORY launch/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(FILES scripts/sr_hand/grasps.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts/sr_hand)
