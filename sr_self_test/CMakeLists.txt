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

find_package(ImageMagick COMPONENTS Magick++ REQUIRED)
include_directories( ${ImageMagick_INCLUDE_DIRS})

#common commands for building c++ executables and libraries
#rosbuild_add_library(gnuplot_cpp src/gnuplot.cpp)
rosbuild_add_library(sr_self_test src/sr_self_test.cpp src/sr_test_runner.cpp src/test_joint_movement.cpp src/manual_test.cpp src/diagnostic_parser.cpp src/sensor_noise_test.cpp src/motor_test.cpp)
rosbuild_link_boost(sr_self_test system)
rosbuild_link_boost(sr_self_test iostreams)
rosbuild_link_boost(sr_self_test filesystem)
rosbuild_link_boost(sr_self_test thread)

#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#target_link_libraries(example ${PROJECT_NAME})

###
# Tests
###
rosbuild_add_executable(sr_self_test_test test/test_self_test.cpp)
rosbuild_add_gtest_build_flags(sr_self_test)
target_link_libraries(sr_self_test_test sr_self_test)
rosbuild_add_rostest(test/sr_self_test.test)
