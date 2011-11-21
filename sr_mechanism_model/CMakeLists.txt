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

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(TinyXML)
if ( NOT TINYXML_FOUND )
  message("Cannot find TinyXML neither in ROS nor in your system !")
else( NOT TINYXML_FOUND )
  message("Found TinyXML !")
  include_directories(${TINYXML_INCLUDE_DIR})
  set(LIBS ${LIBS} ${TINYXML_LIBRARIES})
endif( NOT TINYXML_FOUND )

rosbuild_add_library( sr_mechanism_model src/joint_0_transmission.cpp src/simple_transmission.cpp)
