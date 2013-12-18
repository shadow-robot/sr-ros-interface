cmake_minimum_required(VERSION 2.8.3)
project(sr_mechanism_controllers)
find_package(catkin REQUIRED COMPONENTS rostest std_msgs std_srvs roscpp actionlib pr2_controllers_msgs sr_robot_msgs
  sr_utilities pr2_controller_interface pr2_mechanism_model realtime_tools robot_mechanism_controllers pr2_mechanism_controllers
  pluginlib rosconsole angles control_toolbox sr_hardware_interface xmlrpcpp)
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

#if you compile with "DEBUG=1 make", some debug data are going to be published
set(debug $ENV{DEBUG})
message(" ----- Shadow Robot EtherCAT driver configuration:")
if(DEFINED debug)
  set( CMAKE_CXX_FLAGS "-D DEBUG_PUBLISHER")
  message(" [x] Debug publisher")
else(DEFINED debug)
  message(" [ ] Debug publisher")
endif(DEFINED debug)
message(" ----- ")

add_executable(joint_trajectory_action_controller src/joint_trajectory_action_controller.cpp)
add_dependencies(joint_trajectory_action_controller ${catkin_EXPORTED_TARGETS})
target_link_libraries(joint_trajectory_action_controller ${catkin_LIBRARIES})
add_executable(joint_spline_trajectory_action_controller src/joint_spline_trajectory_action_controller.cpp)
add_dependencies(joint_spline_trajectory_action_controller ${catkin_EXPORTED_TARGETS})
target_link_libraries(joint_spline_trajectory_action_controller ${catkin_LIBRARIES})
add_executable(simple_spline_trajectory example/simple_spline_trajectory.cpp)
add_dependencies(simple_spline_trajectory ${catkin_EXPORTED_TARGETS})
target_link_libraries(simple_spline_trajectory ${catkin_LIBRARIES})

add_library(sr_mechanism_controllers
  src/srh_fake_joint_calibration_controller.cpp
  src/srh_joint_position_controller.cpp
  src/srh_joint_effort_controller.cpp
  src/srh_joint_velocity_controller.cpp
  src/srh_mixed_position_velocity_controller.cpp
  src/sr_friction_compensation.cpp
  src/srh_muscle_joint_position_controller.cpp
  src/srh_joint_muscle_valve_controller.cpp
  example/srh_syntouch_controllers.cpp
  src/sr_controller.cpp
)
add_dependencies(sr_mechanism_controllers ${catkin_EXPORTED_TARGETS})
target_link_libraries(sr_mechanism_controllers ${catkin_LIBRARIES} sr_calibration)

add_executable(simple_trajectory example/simple_trajectory.cpp)
add_dependencies(simple_trajectory ${catkin_EXPORTED_TARGETS})
target_link_libraries(simple_trajectory ${catkin_LIBRARIES})
add_executable(simple_trajectory_compare example/simple_trajectory_compare.cpp)
add_dependencies(simple_trajectory_compare ${catkin_EXPORTED_TARGETS})
target_link_libraries(simple_trajectory_compare ${catkin_LIBRARIES})

catkin_package(
  DEPENDS xmlrpcpp
  CATKIN_DEPENDS std_msgs std_srvs roscpp actionlib pr2_controllers_msgs sr_robot_msgs sr_utilities pr2_controller_interface pr2_mechanism_model
    realtime_tools robot_mechanism_controllers pr2_mechanism_controllers pluginlib rosconsole angles control_toolbox sr_hardware_interface
  INCLUDE_DIRS include
  LIBRARIES
)

if(COMMAND add_rostest_gtest)
  add_rostest_gtest(test_joint_pos_controller test/joint_pos_controller.test test/test_joint_pos_controller.cpp test/test_controllers.cpp
    src/srh_joint_position_controller.cpp src/sr_controller.cpp)
  add_dependencies(test_joint_pos_controller ${catkin_EXPORTED_TARGETS})
  target_link_libraries(test_joint_pos_controller sr_mechanism_controllers ${GTEST_LIBRARIES} ${catkin_LIBRARIES} tinyxml sr_calibration)
  add_rostest_gtest(test_joint_pos_controller_with_fc test/joint_pos_controller_with_fc.test test/test_joint_pos_controller.cpp test/test_controllers.cpp
    src/srh_joint_position_controller.cpp src/sr_controller.cpp)
  add_dependencies(test_joint_pos_controller_with_fc ${catkin_EXPORTED_TARGETS})
  target_link_libraries(test_joint_pos_controller_with_fc sr_mechanism_controllers ${GTEST_LIBRARIES} ${catkin_LIBRARIES} tinyxml sr_calibration)
endif(COMMAND add_rostest_gtest)

install(TARGETS joint_trajectory_action_controller joint_spline_trajectory_action_controller simple_spline_trajectory
  sr_mechanism_controllers simple_trajectory simple_trajectory_compare
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)