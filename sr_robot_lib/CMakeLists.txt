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

rosbuild_add_library(sr_hand_lib src/sr_hand_lib.cpp src/sr_robot_lib.cpp src/motor_updater.cpp)

###############
#     TESTS
SET(jenkins $ENV{JENKINS})

IF(DEFINED jenkins)
  add_library(gcov STATIC IMPORTED)
  set_property(TARGET gcov PROPERTY IMPORTED_LOCATION /usr/lib/gcc/i486-linux-gnu/4.4/libgcov.a)
ENDIF(DEFINED jenkins)

rosbuild_add_executable(test/test_robot_lib test/test_robot_lib.cpp src/sr_robot_lib.cpp src/sr_hand_lib.cpp src/motor_updater.cpp)
IF(DEFINED jenkins)
  rosbuild_add_compile_flags(test/test_robot_lib -g -pg -fprofile-arcs -ftest-coverage)
  target_link_libraries(test/test_robot_lib gcov)
ENDIF(DEFINED jenkins)
rosbuild_add_gtest_build_flags(test/test_robot_lib)
rosbuild_add_rostest(launch/sr_hand_lib.test)

rosbuild_add_executable(test/motor_updater_test test/motor_updater_test.cpp src/motor_updater.cpp)
rosbuild_add_gtest_build_flags(test/motor_updater_test)
IF(DEFINED jenkins)
  rosbuild_add_compile_flags(test/motor_updater_test -g -pg -fprofile-arcs -ftest-coverage)
  target_link_libraries(test/motor_updater_test gcov)
ENDIF(DEFINED jenkins)
rosbuild_add_rostest(test/motor_updater.test)