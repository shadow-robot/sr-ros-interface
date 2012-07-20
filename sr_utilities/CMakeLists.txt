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

rosbuild_gensrv()

rosbuild_add_library(sr_calibration src/calibration.cpp)

rosbuild_add_gtest(test/test_calibration test/test_calibration.cpp src/calibration.cpp)

SET(jenkins $ENV{JENKINS})
IF(DEFINED jenkins)
  add_library(gcov STATIC IMPORTED)
  set_property(TARGET gcov PROPERTY IMPORTED_LOCATION /usr/lib/gcc/i486-linux-gnu/4.4/libgcov.a)
ENDIF(DEFINED jenkins)

rosbuild_add_gtest(test/test_thread_safe_map test/test_thread_safe_map.cpp)
IF(DEFINED jenkins)
  rosbuild_add_compile_flags(test/test_thread_safe_map -g -pg -fprofile-arcs -ftest-coverage)
  target_link_libraries(test/test_thread_safe_map gcov)
ENDIF(DEFINED jenkins)

rosbuild_add_gtest(test/test_math_utils test/test_math_utils.cpp)
IF(DEFINED jenkins)
  rosbuild_add_compile_flags(test/test_math_utils  -g -pg -fprofile-arcs -ftest-coverage)
  target_link_libraries(test/test_math_utils gcov)
ENDIF(DEFINED jenkins)
