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

find_package(ImageMagick COMPONENTS Magick++ REQUIRED)
include_directories( ${ImageMagick_INCLUDE_DIRS})

rosbuild_add_executable(sr_movements src/sr_movements.cpp src/movement_from_image.cpp src/partial_movement.cpp src/movement_publisher.cpp)
target_link_libraries(sr_movements ${ImageMagick_LIBRARIES})

rosbuild_add_rostest(test/test_movements.test)
rosbuild_add_rostest(test/test_joint_name_movement.test)

rosbuild_add_library(sr_movement src/movement_from_image.cpp src/partial_movement.cpp src/movement_publisher.cpp)
target_link_libraries(sr_movement ${ImageMagick_LIBRARIES})

