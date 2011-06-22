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
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

rosbuild_add_library(sr_edc_ethercat_drivers src/sr0x.cpp src/sr06.cpp src/srbridge.cpp src/srcan.cpp src/srdmotor.cpp src/motor_trace_buffer.cpp src/utils/motor_updater.cpp)
target_link_libraries(sr_edc_ethercat_drivers sr_hand_lib)
rosbuild_add_link_flags(sr_edc_ethercat_drivers -lbfd)


rosbuild_add_executable(test/sr_edc_ethercat_drivers_test test/sr_edc_ethercat_drivers_test.cpp src/utils/motor_updater.cpp)
rosbuild_add_gtest_build_flags(test/sr_edc_ethercat_drivers_test)
rosbuild_add_compile_flags(test/sr_edc_ethercat_drivers_test -g -pg -fprofile-arcs -ftest-coverage)
add_library(gcov STATIC IMPORTED)
set_property(TARGET gcov PROPERTY IMPORTED_LOCATION /usr/lib/gcc/i486-linux-gnu/4.4/libgcov.a)
target_link_libraries(test/sr_edc_ethercat_drivers_test gcov)

rosbuild_add_rostest(test/test.launch)