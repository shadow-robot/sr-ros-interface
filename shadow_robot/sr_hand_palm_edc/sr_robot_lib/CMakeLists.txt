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

rosbuild_add_library(sr_hand_lib src/sr_hand_lib.cpp src/calibration.cpp src/sr_robot_lib.cpp)

#tests:
rosbuild_add_gtest(test/test_calibration test/test_calibration.cpp src/calibration.cpp)

rosbuild_add_executable(test/test_robot_lib test/test_robot_lib.cpp src/sr_robot_lib.cpp src/sr_hand_lib.cpp src/calibration.cpp)
rosbuild_add_compile_flags(test/test_robot_lib -g)
rosbuild_add_gtest_build_flags(test/test_robot_lib)
rosbuild_add_rostest(launch/sr_hand_lib.test)