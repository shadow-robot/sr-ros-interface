# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(sr_mechanism_controllers)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS std_msgs std_srvs roscpp actionlib pr2_controllers_msgs sr_robot_msgs sr_utilities pr2_controller_interface pr2_mechanism_model realtime_tools robot_mechanism_controllers pr2_mechanism_controllers pluginlib rosconsole angles control_toolbox sr_hardware_interface xmlrpcpp)

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

#if you compile with "DEBUG=1 make", some debug data are going to be published
SET(debug $ENV{DEBUG})
MESSAGE(" ----- Shadow Robot EtherCAT driver configuration:")
IF(DEFINED debug)
  SET( CMAKE_CXX_FLAGS "-D DEBUG_PUBLISHER")
  MESSAGE(" [x] Debug publisher")
ELSE(DEFINED debug)
  MESSAGE(" [ ] Debug publisher")
ENDIF(DEFINED debug)
MESSAGE(" ----- ")


add_executable(joint_trajectory_action_controller src/joint_trajectory_action_controller.cpp)
add_executable(joint_spline_trajectory_action_controller src/joint_spline_trajectory_action_controller.cpp)

add_executable(simple_spline_trajectory example/simple_spline_trajectory.cpp)

add_library(${PROJECT_NAME}
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


###############
#     TESTS
SET(jenkins $ENV{JENKINS})

IF(DEFINED jenkins)
  add_library(gcov STATIC IMPORTED)
  set_property(TARGET gcov PROPERTY IMPORTED_LOCATION /usr/lib/gcc/i486-linux-gnu/4.4/libgcov.a)
ENDIF(DEFINED jenkins)

#building the actual test
add_executable(test/test_joint_position_controller test/test_joint_position_controller.cpp test/test_controllers.cpp src/srh_joint_position_controller.cpp src/sr_controller.cpp)
target_link_libraries(test/test_joint_position_controller tinyxml)

target_link_libraries(test/test_joint_position_controller ${PROJECT_NAME})
IF(DEFINED jenkins)

# CATKIN_MIGRATION
# use link_directories() include_directories(), add_definitions(), target_link_libraries() or set_target_properties
#   rosbuild_add_compile_flags(test/test_joint_position_controller -g -pg -fprofile-arcs -ftest-coverage)
  target_link_libraries(test/test_joint_position_controller gcov)
ENDIF(DEFINED jenkins)

# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_add_gtest_build_flags(test/test_joint_position_controller)

#add_rostest(test/joint_pos_controller.test)
#add_rostest(test/joint_pos_controller_with_fc.test)

#add_executable(simple_spline_trajectory example/simple_spline_trajectory.cpp)
#add_executable(simple_trajectory example/simple_trajectory.cpp)
#add_executable(simple_trajectory_compare example/simple_trajectory_compare.cpp)
## Generate added messages and services with any dependencies listed here
#generate_messages(
#    #TODO DEPENDENCIES geometry_msgs std_msgs
#)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
catkin_package(
    DEPENDS std_msgs std_srvs roscpp actionlib pr2_controllers_msgs sr_robot_msgs sr_utilities pr2_controller_interface pr2_mechanism_model realtime_tools robot_mechanism_controllers pr2_mechanism_controllers pluginlib rosconsole angles control_toolbox sr_hardware_interface xmlrpcpp
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)