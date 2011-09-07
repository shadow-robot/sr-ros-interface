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
#rosbuild_gensrv()


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

rosbuild_add_library(sr_edc_ethercat_drivers src/sr0x.cpp src/sr06.cpp src/srbridge.cpp src/motor_trace_buffer.cpp)
target_link_libraries(sr_edc_ethercat_drivers sr_hand_lib)
rosbuild_add_link_flags(sr_edc_ethercat_drivers -lbfd)


###############
#   EXAMPLES  #
###############
rosbuild_add_executable(examples/send_targets examples/send_targets.cpp)
