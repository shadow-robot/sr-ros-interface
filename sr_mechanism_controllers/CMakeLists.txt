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


rosbuild_add_executable(joint_trajectory_action_controller src/joint_trajectory_action_controller.cpp)
rosbuild_add_executable(joint_spline_trajectory_action_controller src/joint_spline_trajectory_action_controller.cpp)

rosbuild_add_executable(simple_spline_trajectory example/simple_spline_trajectory.cpp)

rosbuild_add_library(${PROJECT_NAME}
  src/srh_fake_joint_calibration_controller.cpp
  src/srh_joint_position_controller.cpp
  src/srh_joint_effort_controller.cpp
  src/srh_joint_velocity_controller.cpp
  src/srh_mixed_position_velocity_controller.cpp
  src/sr_friction_compensation.cpp

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
rosbuild_add_executable(test/test_joint_position_controller test/test_joint_position_controller.cpp test/test_controllers.cpp src/srh_joint_position_controller.cpp src/sr_controller.cpp)
target_link_libraries(test/test_joint_position_controller tinyxml)

target_link_libraries(test/test_joint_position_controller ${PROJECT_NAME})
IF(DEFINED jenkins)
  rosbuild_add_compile_flags(test/test_joint_position_controller -g -pg -fprofile-arcs -ftest-coverage)
  target_link_libraries(test/test_joint_position_controller gcov)
ENDIF(DEFINED jenkins)
rosbuild_add_gtest_build_flags(test/test_joint_position_controller)

rosbuild_add_rostest(test/joint_pos_controller.test)
rosbuild_add_rostest(test/joint_pos_controller_with_fc.test)

#rosbuild_add_executable(simple_spline_trajectory example/simple_spline_trajectory.cpp)
#rosbuild_add_executable(simple_trajectory example/simple_trajectory.cpp)
#rosbuild_add_executable(simple_trajectory_compare example/simple_trajectory_compare.cpp)
