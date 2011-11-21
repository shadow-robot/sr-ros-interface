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

#Cybergrasp: uncomment this line if you want to use the cybergrasp
#rosbuild_add_executable(cybergrasp_remapper src/shadowhand_to_cybergrasp_remapper.cpp src/shadowhand_to_cybergrasp_remapper_node.cpp src/calibration_parser.cpp)
rosbuild_add_executable(cyberglove_remapper src/shadowhand_to_cyberglove_remapper.cpp src/shadowhand_to_cyberglove_remapper_node.cpp src/calibration_parser.cpp)
